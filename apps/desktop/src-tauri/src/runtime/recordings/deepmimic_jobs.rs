//! Desktop-owned conversion workers, durable status and atomic motion publication.

use super::deepmimic::{export_source, scan, source_digest};
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use microduck_training::deepmimic::{
    BundleSummary, ConversionJob, ConversionRequest, ConversionStage, JobStatus, MotionManifest,
    valid_id,
};
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};

#[derive(Clone, Serialize, Deserialize)]
struct Record {
    job: ConversionJob,
    key: String,
}

struct Entry {
    record: Record,
    child: Option<Child>,
}

#[derive(Default)]
struct Store {
    loaded: bool,
    entries: BTreeMap<String, Entry>,
}

pub(crate) struct Manager {
    config: Option<PathBuf>,
    root: PathBuf,
    store: Mutex<Store>,
}

fn active(status: JobStatus) -> bool {
    matches!(status, JobStatus::Queued | JobStatus::Running)
}

fn conversion_key(request: &ConversionRequest, mcap_sha256: &str) -> Result<String, String> {
    Ok(format!(
        "{:x}",
        Sha256::digest(
            serde_json::to_vec(&json!({
                "mcap":mcap_sha256,"interval":request.interval_id,
                "start":request.start_s,"end":request.end_s,"playback":request.playback,
            }))
            .map_err(|e| e.to_string())?
        )
    ))
}

impl Manager {
    pub(crate) fn new(config: Option<&Path>) -> Self {
        Self {
            config: config.map(Path::to_owned),
            root: super::root(config),
            store: Mutex::new(Store::default()),
        }
    }

    fn motion_root(&self, recording_id: &str) -> PathBuf {
        self.root.join(recording_id).join("deepmimic")
    }

    fn roots(&self, recording_id: Option<&str>) -> Result<Vec<PathBuf>, String> {
        if let Some(id) = recording_id {
            if !valid_id(id) {
                return Err("Invalid recording ID".into());
            }
            let root = self.motion_root(id);
            return Ok(if root.is_dir() { vec![root] } else { vec![] });
        }
        if !self.root.exists() {
            return Ok(vec![]);
        }
        let mut roots = Vec::new();
        for entry in fs::read_dir(&self.root).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            let id = entry.file_name().to_string_lossy().into_owned();
            let root = entry.path().join("deepmimic");
            if valid_id(&id)
                && entry.file_type().map_err(|e| e.to_string())?.is_dir()
                && root.is_dir()
            {
                roots.push(root);
            }
        }
        Ok(roots)
    }

    fn pending(&self, job: &ConversionJob) -> PathBuf {
        self.motion_root(&job.request.recording_id)
            .join(format!(".pending-{}", job.id))
    }

    fn persist(&self, record: &Record) -> Result<(), String> {
        let jobs = self
            .motion_root(&record.job.request.recording_id)
            .join(".jobs");
        fs::create_dir_all(&jobs).map_err(|e| e.to_string())?;
        let path = jobs.join(format!("{}.json", record.job.id));
        let temporary = path.with_extension("pending");
        let mut file = File::create(&temporary).map_err(|e| e.to_string())?;
        file.write_all(&serde_json::to_vec(record).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        file.sync_all().map_err(|e| e.to_string())?;
        fs::rename(&temporary, &path).map_err(|e| e.to_string())
    }

    fn cleanup(&self, job: &ConversionJob) -> Result<(), String> {
        let pending = self.pending(job);
        if pending.exists() {
            let root = self
                .motion_root(&job.request.recording_id)
                .canonicalize()
                .map_err(|e| e.to_string())?;
            let resolved = pending.canonicalize().map_err(|e| e.to_string())?;
            if resolved.parent() != Some(root.as_path())
                || resolved.file_name() != pending.file_name()
            {
                return Err("Conversion temporary directory escapes motion root".into());
            }
            fs::remove_dir_all(&pending).map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    fn lock(&self) -> Result<MutexGuard<'_, Store>, String> {
        let mut store = self.store.lock().map_err(|e| e.to_string())?;
        if !store.loaded {
            let mut recovered = BTreeMap::new();
            let mut paths = Vec::new();
            for root in self.roots(None)? {
                let jobs = root.join(".jobs");
                if jobs.is_dir() {
                    for entry in fs::read_dir(jobs).map_err(|e| e.to_string())? {
                        paths.push(entry.map_err(|e| e.to_string())?.path());
                    }
                }
            }
            for path in paths {
                if path.extension().is_none_or(|ext| ext != "json") {
                    continue;
                }
                let mut record: Record = serde_json::from_slice(
                    &fs::read(&path).map_err(|e| e.to_string())?,
                )
                .map_err(|e| format!("Invalid conversion status {}: {e}", path.display()))?;
                if !valid_id(&record.job.id)
                    || !valid_id(&record.job.bundle_id)
                    || !valid_id(&record.job.request.recording_id)
                    || path.parent()
                        != Some(
                            self.motion_root(&record.job.request.recording_id)
                                .join(".jobs")
                                .as_path(),
                        )
                    || path.file_stem().and_then(|id| id.to_str()) != Some(&record.job.id)
                {
                    return Err("Invalid persisted conversion identity".into());
                }
                if active(record.job.status) {
                    match self.read_bundle(&record.job.bundle_id) {
                        Ok(manifest)
                            if manifest.source.recording_id == record.job.request.recording_id =>
                        {
                            record.job.status = JobStatus::Succeeded;
                            record.job.stage = ConversionStage::Saving;
                            record.job.processed_frames = manifest.clip.frames;
                            record.job.total_frames = Some(manifest.clip.frames);
                            record.job.error = None;
                        }
                        _ => {
                            record.job.status = JobStatus::Interrupted;
                            record.job.error =
                                Some("Conversion interrupted by application shutdown".into());
                        }
                    }
                    self.persist(&record)?;
                }
                self.cleanup(&record.job)?;
                recovered.insert(
                    record.job.id.clone(),
                    Entry {
                        record,
                        child: None,
                    },
                );
            }
            store.entries = recovered;
            store.loaded = true;
        }
        Ok(store)
    }

    fn find_bundle(&self, id: &str) -> Result<(PathBuf, MotionManifest), String> {
        if !valid_id(id) || id.starts_with('.') {
            return Err("Invalid motion bundle ID".into());
        }
        for root in self.roots(None)? {
            let path = root.join("manifest.json");
            if !path.is_file() {
                continue;
            }
            let Ok(bytes) = fs::read(&path) else {
                continue;
            };
            let Ok(declared) = serde_json::from_slice::<MotionManifest>(&bytes) else {
                continue;
            };
            if declared.bundle_id != id {
                continue;
            }
            let directory = root.canonicalize().map_err(|e| e.to_string())?;
            let manifest = validate_bundle(&directory, id)?;
            if root
                .parent()
                .and_then(|p| p.file_name())
                .and_then(|n| n.to_str())
                != Some(&manifest.source.recording_id)
            {
                return Err("Motion bundle belongs to another recording".into());
            }
            return Ok((directory, manifest));
        }
        Err("Motion bundle not found".into())
    }

    fn read_bundle(&self, id: &str) -> Result<MotionManifest, String> {
        self.find_bundle(id).map(|(_, manifest)| manifest)
    }

    pub(crate) fn status(&self, id: &str) -> Result<ConversionJob, String> {
        self.lock()?
            .entries
            .get(id)
            .map(|entry| entry.record.job.clone())
            .ok_or_else(|| "Unknown conversion job".into())
    }

    pub(crate) fn jobs(&self, recording_id: &str) -> Result<Vec<ConversionJob>, String> {
        Ok(self
            .lock()?
            .entries
            .values()
            .filter(|entry| entry.record.job.request.recording_id == recording_id)
            .map(|entry| entry.record.job.clone())
            .collect())
    }

    pub(crate) fn bundles(&self, recording_id: Option<&str>) -> Result<Vec<BundleSummary>, String> {
        drop(self.lock()?);
        let mut bundles = Vec::new();
        for root in self.roots(recording_id)? {
            let Ok(bytes) = fs::read(root.join("manifest.json")) else {
                continue;
            };
            let Ok(declared) = serde_json::from_slice::<MotionManifest>(&bytes) else {
                continue;
            };
            let id = declared.bundle_id;
            let Ok(manifest) = self.read_bundle(&id) else {
                continue;
            };
            if recording_id.is_some_and(|value| manifest.source.recording_id != value) {
                continue;
            }
            bundles.push(BundleSummary {
                id,
                name: manifest.name,
                recording_id: manifest.source.recording_id,
                body_id: manifest.body.id,
                control_s: manifest.timing.control_s,
                frames: manifest.clip.frames,
                duration_s: manifest.clip.duration_s,
                playback: manifest.clip.playback,
            });
        }
        Ok(bundles)
    }

    pub(crate) fn bundle_path(&self, id: &str) -> Result<PathBuf, String> {
        drop(self.lock()?);
        self.find_bundle(id).map(|(path, _)| path)
    }

    pub(crate) fn start(
        self: &Arc<Self>,
        request: ConversionRequest,
    ) -> Result<ConversionJob, String> {
        request.validate()?;
        let source = scan(self.config.as_deref(), &request.recording_id)?;
        if source.read_failure.is_some() {
            return Err("Recording is truncated or unreadable; inspect its interval report".into());
        }
        if !source
            .intervals
            .iter()
            .any(|interval| interval.id == request.interval_id)
        {
            return Err("Unknown source interval".into());
        }
        let key = conversion_key(&request, &source.mcap_sha256)?;
        let job = self.enqueue(request, key)?;
        if job.status != JobStatus::Queued {
            return Ok(job);
        }
        // Only this call can move a newly queued entry into Running under the task lock.
        let mut store = self.lock()?;
        let entry = store
            .entries
            .get_mut(&job.id)
            .ok_or("Missing queued conversion")?;
        if entry.record.job.status != JobStatus::Queued {
            return Ok(entry.record.job.clone());
        }
        entry.record.job.status = JobStatus::Running;
        if let Err(error) = self.persist(&entry.record) {
            entry.record.job.status = JobStatus::Failed;
            entry.record.job.error = Some(format!("Failed saving conversion status: {error}"));
            return Err(error);
        }
        let manager = Arc::clone(self);
        let id = job.id.clone();
        if let Err(error) = std::thread::Builder::new()
            .name(format!("deepmimic-{id}"))
            .spawn(move || manager.worker(&id))
        {
            entry.record.job.status = JobStatus::Failed;
            entry.record.job.error = Some(error.to_string());
            self.persist(&entry.record)?;
        }
        Ok(entry.record.job.clone())
    }

    fn enqueue(&self, request: ConversionRequest, key: String) -> Result<ConversionJob, String> {
        let mut store = self.lock()?;
        if let Some(existing) = store.entries.values().find(|entry| {
            entry.record.job.request.recording_id == request.recording_id
                && ((entry.record.key == key && active(entry.record.job.status))
                    || (entry.record.job.status == JobStatus::Succeeded
                        && self
                            .read_bundle(&entry.record.job.bundle_id)
                            .is_ok_and(|manifest| {
                                conversion_key(
                                    &entry.record.job.request,
                                    &manifest.source.mcap_sha256,
                                )
                                .is_ok_and(|existing| existing == key)
                            })))
        }) {
            return Ok(existing.record.job.clone());
        }
        let id = format!("convert-{}", uuid::Uuid::new_v4());
        let job = ConversionJob {
            id: id.clone(),
            bundle_id: format!("motion-{}", uuid::Uuid::new_v4()),
            request,
            status: JobStatus::Queued,
            stage: ConversionStage::Reading,
            processed_frames: 0,
            total_frames: None,
            error: None,
        };
        let record = Record {
            job: job.clone(),
            key,
        };
        self.persist(&record)?;
        store.entries.insert(
            id,
            Entry {
                record,
                child: None,
            },
        );
        Ok(job)
    }

    pub(crate) fn cancel(&self, id: &str) -> Result<ConversionJob, String> {
        let mut store = self.lock()?;
        let entry = store.entries.get_mut(id).ok_or("Unknown conversion job")?;
        if active(entry.record.job.status) {
            if let Some(child) = &mut entry.child
                && child.try_wait().map_err(|e| e.to_string())?.is_none()
            {
                crate::training::process::stop_child(child).map_err(|e| e.to_string())?;
            }
            entry.record.job.status = JobStatus::Cancelled;
            self.persist(&entry.record)?;
        }
        Ok(entry.record.job.clone())
    }

    fn update(&self, id: &str, stage: ConversionStage, frames: u32) -> Result<(), String> {
        let mut store = self.lock()?;
        let entry = store.entries.get_mut(id).ok_or("Unknown conversion job")?;
        if !active(entry.record.job.status) {
            return Err("Conversion cancelled".into());
        }
        entry.record.job.stage = stage;
        entry.record.job.processed_frames = frames;
        if frames > 0 {
            entry.record.job.total_frames = Some(frames);
        }
        self.persist(&entry.record)
    }

    fn worker(&self, id: &str) {
        let result = self.convert(id);
        self.finish_worker(id, result);
    }

    fn finish_worker(&self, id: &str, result: Result<(), String>) {
        if let Ok(mut store) = self.lock()
            && let Some(entry) = store.entries.get_mut(id)
        {
            if let Some(mut child) = entry.child.take() {
                if child.try_wait().ok().flatten().is_none() {
                    let _ = crate::training::process::stop_child(&mut child);
                }
                let _ = child.wait();
            }
            if active(entry.record.job.status) {
                entry.record.job.status = JobStatus::Failed;
                entry.record.job.error = Some(
                    result
                        .err()
                        .unwrap_or_else(|| "Conversion ended before publication".into()),
                );
                if let Err(error) = self.persist(&entry.record) {
                    entry.record.job.error =
                        Some(format!("Failed saving conversion status: {error}"));
                }
            }
        }
        if let Err(error) = self.status(id).and_then(|job| self.cleanup(&job)) {
            crate::platform::logging::diagnostic(
                self.config.as_deref(),
                "deepmimic",
                "error",
                "temporary_cleanup_failed",
                json!({"job_id":id,"error":error}),
            );
        }
    }

    fn convert(&self, id: &str) -> Result<(), String> {
        let job = self.status(id)?;
        self.update(id, ConversionStage::Reading, 0)?;
        let pending = self.pending(&job);
        fs::create_dir(&pending).map_err(|e| e.to_string())?;
        let context = export_source(self.config.as_deref(), &job.request, &pending)?;
        let frames = context["frames"]
            .as_u64()
            .and_then(|v| u32::try_from(v).ok())
            .ok_or("Invalid conversion frame count")?;
        self.update(id, ConversionStage::Validating, frames)?;
        let mut command = microduck_training::launch::deepmimic_conversion_command(
            "uv",
            &microduck_training::bundle::project()?,
            &pending,
            self.config.as_deref(),
            &json!({"bundle_id":job.bundle_id,"name":job.request.name,"playback":job.request.playback}),
        )?;
        self.run_converter(id, &mut command, frames)?;
        self.publish(id, &pending.join("bundle"))
    }

    fn run_converter(&self, id: &str, command: &mut Command, frames: u32) -> Result<(), String> {
        command.stdin(Stdio::null()).stdout(Stdio::piped()).stderr(
            File::create(
                self.motion_root(&self.status(id)?.request.recording_id)
                    .join(".jobs")
                    .join(format!("{id}.stderr.log")),
            )
            .map_err(|e| e.to_string())?,
        );
        #[cfg(windows)]
        {
            use std::os::windows::process::CommandExt;
            command.creation_flags(0x08000000);
        }
        let stdout = {
            let mut store = self.lock()?;
            let entry = store.entries.get_mut(id).ok_or("Unknown conversion job")?;
            if !active(entry.record.job.status) {
                return Err("Conversion cancelled".into());
            }
            let mut child = command
                .spawn()
                .map_err(|e| format!("Failed starting pinned converter: {e}"))?;
            let stdout = child.stdout.take().ok_or("Missing converter stdout")?;
            entry.child = Some(child);
            stdout
        };
        let mut log = File::create(
            self.motion_root(&self.status(id)?.request.recording_id)
                .join(".jobs")
                .join(format!("{id}.stdout.log")),
        )
        .map_err(|e| e.to_string())?;
        let mut process_error = None;
        for line in BufReader::new(stdout).lines() {
            let line = line.map_err(|e| e.to_string())?;
            writeln!(log, "{line}").map_err(|e| e.to_string())?;
            if let Ok(value) = serde_json::from_str::<serde_json::Value>(&line) {
                if let Some(error) = value["error"].as_str() {
                    process_error = Some(error.to_owned());
                }
                if let Ok(stage) = serde_json::from_value::<ConversionStage>(value["stage"].clone())
                {
                    self.update(
                        id,
                        stage,
                        value["processed_frames"]
                            .as_u64()
                            .and_then(|v| u32::try_from(v).ok())
                            .unwrap_or(frames),
                    )?;
                }
            }
        }
        loop {
            let mut store = self.lock()?;
            let entry = store.entries.get_mut(id).ok_or("Unknown conversion job")?;
            if !active(entry.record.job.status) {
                return Err("Conversion cancelled".into());
            }
            let status = entry
                .child
                .as_mut()
                .ok_or("Missing converter process")?
                .try_wait()
                .map_err(|e| e.to_string())?;
            if let Some(status) = status {
                entry.child = None;
                if !status.success() {
                    return Err(process_error.unwrap_or_else(|| {
                        format!("Converter exited {status}; see job stderr log")
                    }));
                }
                break;
            }
            drop(store);
            std::thread::sleep(Duration::from_millis(20));
        }
        Ok(())
    }

    fn publish(&self, id: &str, prepared: &Path) -> Result<(), String> {
        let job = self.status(id)?;
        let manifest = validate_bundle(prepared, &job.bundle_id)?;
        if manifest.source.recording_id != job.request.recording_id {
            return Err("Converter returned another recording".into());
        }
        let mut store = self.lock()?;
        let entry = store.entries.get_mut(id).ok_or("Unknown conversion job")?;
        if !active(entry.record.job.status) {
            return Err("Conversion cancelled before publication".into());
        }
        let destination = self.motion_root(&job.request.recording_id);
        let manifest_path = destination.join("manifest.json");
        if manifest_path.exists() {
            fs::remove_file(&manifest_path).map_err(|e| e.to_string())?;
        }
        let clips = destination.join("clips");
        if clips.exists() {
            fs::remove_dir_all(&clips).map_err(|e| e.to_string())?;
        }
        fs::rename(prepared.join("clips"), &clips).map_err(|e| e.to_string())?;
        fs::rename(
            prepared.join("quality-report.json"),
            destination.join("quality-report.json"),
        )
        .map_err(|e| e.to_string())?;
        // The manifest is the publication marker, installed after all referenced files.
        fs::rename(prepared.join("manifest.json"), manifest_path).map_err(|e| e.to_string())?;
        entry.record.job.status = JobStatus::Succeeded;
        entry.record.job.stage = ConversionStage::Saving;
        entry.record.job.processed_frames = manifest.clip.frames;
        entry.record.job.total_frames = Some(manifest.clip.frames);
        self.persist(&entry.record)
    }
}

fn validate_bundle(directory: &Path, id: &str) -> Result<MotionManifest, String> {
    let manifest: MotionManifest = serde_json::from_slice(
        &fs::read(directory.join("manifest.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    manifest.validate()?;
    if manifest.bundle_id != id {
        return Err("Motion bundle ID disagrees with directory".into());
    }
    let root = directory.canonicalize().map_err(|e| e.to_string())?;
    let clip = root
        .join(&manifest.clip.file)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !clip.starts_with(&root) || source_digest(&clip)? != manifest.clip.sha256 {
        return Err("Motion clip path or hash mismatch".into());
    }
    let quality: serde_json::Value = serde_json::from_slice(
        &fs::read(root.join("quality-report.json")).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    for (key, value) in serde_json::to_value(&manifest.quality)
        .map_err(|e| e.to_string())?
        .as_object()
        .ok_or("Invalid quality object")?
    {
        if &quality[key] != value {
            return Err(format!("Quality report disagrees with manifest: {key}"));
        }
    }
    Ok(manifest)
}

#[cfg(test)]
mod tests {
    use super::*;
    use microduck_training::deepmimic::Playback;

    fn manager() -> Arc<Manager> {
        let config = crate::platform::helpers::test_directory()
            .join(format!("deepmimic-jobs-{}", uuid::Uuid::new_v4()))
            .join("config");
        Arc::new(Manager::new(Some(&config)))
    }

    fn request() -> ConversionRequest {
        ConversionRequest {
            recording_id: "fixture-recording".into(),
            interval_id: "fixture-interval".into(),
            start_s: 0.0,
            end_s: 0.04,
            name: "Fixture".into(),
            playback: Playback::Finite,
        }
    }

    #[test]
    fn duplicate_active_jobs_share_an_id_and_finished_jobs_can_be_repeated() {
        let manager = manager();
        let first = manager.enqueue(request(), "same-key".into()).unwrap();
        let again = manager.enqueue(request(), "same-key".into()).unwrap();
        assert_eq!(first.id, again.id);
        manager.cancel(&first.id).unwrap();
        let next = manager.enqueue(request(), "same-key".into()).unwrap();
        assert_ne!(first.id, next.id);
        assert!(manager.status("../missing").is_err());
        assert!(manager.bundle_path("../missing").is_err());
        assert_eq!(manager.jobs("fixture-recording").unwrap().len(), 2);
    }

    #[test]
    fn restart_marks_unpublished_work_interrupted_and_removes_only_its_pending_directory() {
        let manager = manager();
        let job = manager.enqueue(request(), "restart-key".into()).unwrap();
        fs::create_dir(manager.pending(&job)).unwrap();
        let other = manager
            .motion_root("fixture-recording")
            .join(".pending-unrelated");
        fs::create_dir(&other).unwrap();
        let restarted = Manager::new(manager.config.as_deref());
        assert_eq!(
            restarted.status(&job.id).unwrap().status,
            JobStatus::Interrupted
        );
        assert!(!manager.pending(&job).exists());
        assert!(other.exists());
    }

    #[cfg(windows)]
    #[test]
    fn process_failure_persists_a_failed_status() {
        let manager = manager();
        let job = manager.enqueue(request(), "exit-key".into()).unwrap();
        let mut command = Command::new("cmd.exe");
        command.args(["/d", "/c", "exit", "7"]);
        let result = manager.run_converter(&job.id, &mut command, 3);
        assert!(result.as_ref().unwrap_err().contains("Converter exited"));
        manager.finish_worker(&job.id, result);
        assert_eq!(manager.status(&job.id).unwrap().status, JobStatus::Failed);
        assert_eq!(
            Manager::new(manager.config.as_deref())
                .status(&job.id)
                .unwrap()
                .status,
            JobStatus::Failed
        );
    }

    #[cfg(windows)]
    #[test]
    fn cancel_stops_a_live_converter_process_and_preserves_cancelled_status() {
        let manager = manager();
        let job = manager
            .enqueue(request(), "cancel-process-key".into())
            .unwrap();
        let other = Arc::clone(&manager);
        let id = job.id.clone();
        let worker = std::thread::spawn(move || {
            let mut command = Command::new("powershell.exe");
            command.args(["-NoProfile", "-Command", "Start-Sleep -Seconds 20"]);
            let result = other.run_converter(&id, &mut command, 3);
            other.finish_worker(&id, result);
        });
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            if manager.lock().unwrap().entries[&job.id].child.is_some() {
                break;
            }
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(10));
        }
        manager.cancel(&job.id).unwrap();
        worker.join().unwrap();
        assert_eq!(
            manager.status(&job.id).unwrap().status,
            JobStatus::Cancelled
        );
        assert!(manager.lock().unwrap().entries[&job.id].child.is_none());
    }
}
