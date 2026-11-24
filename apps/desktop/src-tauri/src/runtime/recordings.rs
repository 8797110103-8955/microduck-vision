//! Structured runtime capture, MCAP storage, comparison and teacher-data export.

use std::collections::{BTreeMap, HashMap};
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, mpsc};
use std::thread::JoinHandle;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use serde_json::{Value, json};

use crate::platform::logging;
use crate::runtime::policy::inference::PolicyTrace;
use crate::types::{
    LoopStatus, RecordingComparison, RecordingExecution, RecordingExport, RecordingFrame,
    RecordingInfo, RecordingMetric, RecordingPolicy, RecordingPreviewFrame, RecordingWindow,
};

pub(crate) fn now_ms() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
        * 1000.0
}

pub(crate) fn app_session_id() -> &'static str {
    static ID: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    ID.get_or_init(|| format!("{}-{}", now_ms() as u64, std::process::id()))
}

pub(crate) fn root(config: Option<&Path>) -> PathBuf {
    logging::log_directory(config).with_file_name("recordings")
}

enum Packet {
    Frame(Box<RecordingFrame>, Option<Value>),
    Stop(String),
}

struct Active {
    sender: mpsc::SyncSender<Packet>,
    info: Arc<Mutex<RecordingInfo>>,
    thread: JoinHandle<()>,
    origin: Instant,
    boundary: Option<(u64, u64, u64)>,
    metadata: Option<Value>,
    segment: u32,
}

#[derive(Default)]
pub(crate) struct Recorder {
    active: Mutex<Option<Active>>,
}

fn save_info(info: &RecordingInfo) -> Result<(), String> {
    let temporary = Path::new(&info.path).join("manifest.pending");
    fs::write(
        &temporary,
        serde_json::to_vec_pretty(info).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    fs::rename(temporary, Path::new(&info.path).join("manifest.json")).map_err(|e| e.to_string())
}

fn lifecycle(config: Option<&Path>, info: &RecordingInfo, event: &str, context: Value) {
    logging::diagnostic(
        config,
        "recording",
        if info.error.is_some() {
            "error"
        } else {
            "info"
        },
        event,
        json!({
            "app_session_id": info.app_session_id, "recording_id": info.id,
            "duck_id": info.duck_id, "path": info.path, "result": info.state,
            "frames": info.frames, "dropped_frames": info.dropped_frames,
            "error": info.error, "context": context,
        }),
    );
}

impl Recorder {
    pub(crate) fn context(&self, status: &LoopStatus) -> Value {
        let active = self.active.lock().unwrap_or_else(|e| e.into_inner());
        let key = (
            status.runtime_generation,
            status.execution_generation,
            status.config_revision,
        );
        json!({
            "runtime_id":status.runtime_generation,"tick":status.tick,
            "duck_id":status.active_body.as_ref().and_then(|b| b.physical_machine_id.as_ref().or(b.virtual_machine_id.as_ref())),
            "recording_id":active.as_ref().map(|a| a.info.lock().unwrap_or_else(|e| e.into_inner()).id.clone()),
            "segment_id":active.as_ref().map(|a| a.segment + u32::from(a.boundary != Some(key))),
            "episode_id":active.as_ref().map(|a| a.segment + u32::from(a.boundary != Some(key))),
        })
    }
    pub(crate) fn start(
        &self,
        config: Option<&Path>,
        status: &LoopStatus,
        name: String,
        metadata: Value,
    ) -> Result<RecordingInfo, String> {
        let mut active = self.active.lock().map_err(|e| e.to_string())?;
        if active
            .as_ref()
            .is_some_and(|active| active.thread.is_finished())
            && let Some(previous) = active.take()
        {
            let _ = previous.thread.join();
        }
        if active.is_some() {
            return Err("已有运行采集，请先停止".into());
        }
        let body = status
            .active_body
            .as_ref()
            .ok_or("请先启动一只鸭子的控制环")?;
        let duck_id = body
            .physical_machine_id
            .as_ref()
            .or(body.virtual_machine_id.as_ref())
            .ok_or("运行中的鸭子没有身份")?
            .clone();
        static SEQUENCE: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let prefix: String = name
            .chars()
            .take(64)
            .map(|character| {
                if is_id_character(character) {
                    character
                } else {
                    '-'
                }
            })
            .collect();
        let prefix = prefix.trim_matches(['-', '.', '_']);
        let prefix = if prefix.is_empty() { "onnx" } else { prefix };
        let id = format!(
            "{prefix}-{}-{}-{}",
            app_session_id(),
            now_ms() as u64,
            SEQUENCE.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        );
        let path = root(config).join(&id);
        let mut info = RecordingInfo {
            id,
            app_session_id: app_session_id().into(),
            duck_id,
            name: name.trim().to_owned(),
            path: path.to_string_lossy().into_owned(),
            started_at_ms: now_ms(),
            ended_at_ms: None,
            state: "recording".into(),
            frames: 0,
            dropped_frames: 0,
            error: None,
        };
        if info.name.is_empty() {
            info.name = info.id.clone();
        }
        if let Err(error) = fs::create_dir_all(&path)
            .map_err(|e| e.to_string())
            .and_then(|()| save_info(&info))
        {
            info.state = "failed".into();
            info.error = Some(error.clone());
            lifecycle(
                config,
                &info,
                "recording_start_failed",
                json!({"runtime_id":status.runtime_generation}),
            );
            return Err(error);
        }
        let (sender, receiver) = mpsc::sync_channel(256);
        let (ready, ready_result) = mpsc::sync_channel(1);
        let shared = Arc::new(Mutex::new(info.clone()));
        let writer_info = Arc::clone(&shared);
        let config = config.map(Path::to_owned);
        let thread = std::thread::spawn(move || {
            let result =
                write_recording(&writer_info, receiver, ready, metadata, config.as_deref());
            let mut info = writer_info.lock().unwrap_or_else(|e| e.into_inner());
            info.ended_at_ms = Some(now_ms());
            match result {
                Ok(reason) => {
                    info.state = if reason == "stopped" || reason == "runtime_closed" {
                        "complete"
                    } else {
                        "interrupted"
                    }
                    .into();
                    if info.dropped_frames > 0 {
                        info.state = "incomplete".into();
                    }
                }
                Err(error) => {
                    info.state = "failed".into();
                    info.error = Some(error);
                }
            }
            if let Err(error) = save_info(&info) {
                info.state = "failed".into();
                info.error = Some(error);
            }
            lifecycle(config.as_deref(), &info, "recording_finished", json!({}));
        });
        if let Err(error) = ready_result
            .recv()
            .map_err(|e| e.to_string())
            .and_then(|result| result)
        {
            let _ = thread.join();
            return Err(error);
        }
        *active = Some(Active {
            sender,
            info: shared,
            thread,
            origin: Instant::now(),
            boundary: None,
            metadata: None,
            segment: 0,
        });
        Ok(info)
    }

    pub(crate) fn capture(
        &self,
        status: &LoopStatus,
        trace: Option<&PolicyTrace>,
        metadata: impl FnOnce() -> Value,
        started: Instant,
    ) {
        let Ok(mut guard) = self.active.lock() else {
            return;
        };
        let Some(active) = guard.as_mut() else { return };
        let key = (
            status.runtime_generation,
            status.execution_generation,
            status.config_revision,
        );
        let metadata = metadata();
        let boundary = active.boundary != Some(key) || active.metadata.as_ref() != Some(&metadata);
        let context = boundary.then_some(metadata.clone());
        if boundary {
            active.segment += 1;
            active.boundary = Some(key);
            active.metadata = Some(metadata);
        }
        let body = status.active_body.as_ref();
        let duck = body
            .and_then(|body| {
                body.physical_machine_id
                    .as_ref()
                    .or(body.virtual_machine_id.as_ref())
            })
            .cloned()
            .unwrap_or_default();
        let frame = RecordingFrame {
            recording_id: active
                .info
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .id
                .clone(),
            duck_id: duck,
            segment_id: active.segment,
            episode_id: active.segment,
            elapsed_s: active.origin.elapsed().as_secs_f64(),
            control_started_s: started
                .saturating_duration_since(active.origin)
                .as_secs_f64(),
            status: status.clone(),
            policy: trace.map(RecordingPolicy::from),
        };
        match active
            .sender
            .try_send(Packet::Frame(Box::new(frame), context))
        {
            Ok(()) => {}
            Err(mpsc::TrySendError::Full(_)) => {
                // Retry a dropped segment header with the next sample.
                if boundary {
                    active.boundary = None;
                    active.segment -= 1;
                }
                active
                    .info
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .dropped_frames += 1;
            }
            Err(mpsc::TrySendError::Disconnected(_)) => {}
        }
    }

    pub(crate) fn stop(&self, reason: &str) -> Result<Option<RecordingInfo>, String> {
        let mut guard = self.active.lock().map_err(|e| e.to_string())?;
        let Some(active) = guard.take() else {
            return Ok(None);
        };
        let _ = active.sender.send(Packet::Stop(reason.into()));
        active
            .thread
            .join()
            .map_err(|_| "采集写入线程异常退出".to_owned())?;
        let info = active.info.lock().map_err(|e| e.to_string())?.clone();
        Ok(Some(info))
    }

    pub(crate) fn current(&self) -> Option<RecordingInfo> {
        self.active
            .lock()
            .ok()?
            .as_ref()?
            .info
            .lock()
            .ok()
            .map(|info| info.clone())
    }

    pub(crate) fn is_active(&self) -> bool {
        self.active.lock().is_ok_and(|active| active.is_some())
    }
}

fn write_recording(
    info: &Arc<Mutex<RecordingInfo>>,
    receiver: mpsc::Receiver<Packet>,
    ready: mpsc::SyncSender<Result<(), String>>,
    metadata: Value,
    config: Option<&Path>,
) -> Result<String, String> {
    let initial = info.lock().map_err(|e| e.to_string())?.clone();
    let path = Path::new(&initial.path);
    let opened = (|| {
        let raw = File::create(path.join("runtime.mcap")).map_err(|error| error.to_string())?;
        let motion = File::create(path.join("motion.jsonl")).map_err(|error| error.to_string())?;
        Ok::<_, String>((McapLog::new(BufWriter::new(raw))?, BufWriter::new(motion)))
    })();
    let (mut writer, mut motion) = match opened {
        Ok(writers) => {
            let _ = ready.send(Ok(()));
            writers
        }
        Err(error) => {
            let _ = ready.send(Err(error.clone()));
            return Err(error);
        }
    };
    writer.event(
        "recording_started",
        json!({"recording":initial,"metadata":metadata}),
    )?;
    lifecycle(config, &initial, "recording_started", metadata);
    let mut reason = "app_shutdown".to_owned();
    let mut last_save = Instant::now();
    let mut last_context = json!({});
    for packet in receiver {
        match packet {
            Packet::Frame(frame, metadata) => {
                if let Some(metadata) = metadata {
                    if !last_context
                        .as_object()
                        .is_some_and(|object| object.is_empty())
                    {
                        writer.event("segment_ended", last_context.clone())?;
                    }
                    let context = json!({
                        "runtime_id":frame.status.runtime_generation,
                        "segment_id":frame.segment_id,"episode_id":frame.episode_id,
                        "duck_id":frame.duck_id,"tick":frame.status.tick,
                        "metadata": metadata,
                    });
                    writer.event("segment_started", context.clone())?;
                    lifecycle(config, &initial, "recording_segment_started", context);
                    last_context = json!({"runtime_id":frame.status.runtime_generation,
                        "segment_id":frame.segment_id,"episode_id":frame.episode_id,
                        "tick":frame.status.tick,"duck_id":frame.duck_id});
                }
                writer.frame(&frame)?;
                if let Some(sample) = motion_sample(&frame) {
                    serde_json::to_writer(&mut motion, &sample)
                        .map_err(|error| error.to_string())?;
                    writeln!(motion).map_err(|error| error.to_string())?;
                }
                info.lock().map_err(|e| e.to_string())?.frames += 1;
                if last_save.elapsed().as_secs() >= 1 {
                    writer.writer.flush().map_err(|e| e.to_string())?;
                    motion.flush().map_err(|error| error.to_string())?;
                    let snapshot = info.lock().map_err(|e| e.to_string())?.clone();
                    save_info(&snapshot)?;
                    last_save = Instant::now();
                }
            }
            Packet::Stop(result) => {
                reason = result;
                break;
            }
        }
    }
    let final_info = info.lock().map_err(|e| e.to_string())?.clone();
    writer.event("segment_ended", last_context.clone())?;
    writer.event(
        "recording_stopped",
        json!({"recording":final_info,"reason":reason,"context":last_context}),
    )?;
    lifecycle(
        config,
        &final_info,
        "recording_stopped",
        json!({"reason":reason,"context":last_context}),
    );
    let raw_result = writer.finish();
    let motion_result = motion.flush().map_err(|error| error.to_string());
    raw_result.and(motion_result)?;
    Ok(reason)
}

struct McapLog<W: Write + std::io::Seek> {
    writer: mcap::Writer<W>,
    frames: u16,
    events: u16,
    sequence: u32,
}

impl<W: Write + std::io::Seek> McapLog<W> {
    fn new(output: W) -> Result<Self, String> {
        let mut writer = mcap::Writer::new(output).map_err(|e| e.to_string())?;
        let schema = writer.add_schema("microduck.RuntimeFrame", "jsonschema", br#"{
            "type":"object","required":["recordingId","duckId","segmentId","episodeId","elapsedS","status"],
            "properties":{"recordingId":{"type":"string"},"duckId":{"type":"string"},
            "segmentId":{"type":"integer"},"episodeId":{"type":"integer"},"elapsedS":{"type":"number"},
            "controlStartedS":{"type":"number"},"status":{"type":"object"},"policy":{"type":["object","null"]}}
        }"#).map_err(|e| e.to_string())?;
        let frames = writer
            .add_channel(schema, "/duck/runtime", "json", &BTreeMap::new())
            .map_err(|e| e.to_string())?;
        let event_schema = writer.add_schema("microduck.RecordingEvent", "jsonschema", br#"{
            "type":"object","required":["event","context"],"properties":{"event":{"type":"string"},"context":{"type":"object"}}
        }"#).map_err(|e| e.to_string())?;
        let events = writer
            .add_channel(event_schema, "/duck/events", "json", &BTreeMap::new())
            .map_err(|e| e.to_string())?;
        Ok(Self {
            writer,
            frames,
            events,
            sequence: 0,
        })
    }
    fn write(&mut self, channel_id: u16, value: &impl serde::Serialize) -> Result<(), String> {
        let timestamp = (now_ms() * 1_000_000.0) as u64;
        self.writer
            .write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence: self.sequence,
                    log_time: timestamp,
                    publish_time: timestamp,
                },
                &serde_json::to_vec(value).map_err(|e| e.to_string())?,
            )
            .map_err(|e| e.to_string())?;
        self.sequence += 1;
        Ok(())
    }
    fn frame(&mut self, frame: &RecordingFrame) -> Result<(), String> {
        self.write(self.frames, frame)
    }
    fn event(&mut self, event: &str, context: Value) -> Result<(), String> {
        self.write(self.events, &json!({"event":event,"context":context}))
    }
    fn finish(mut self) -> Result<(), String> {
        self.writer.finish().map_err(|e| e.to_string())?;
        self.writer.into_inner().flush().map_err(|e| e.to_string())
    }
}

#[cfg(test)]
pub(crate) fn write_trace(
    path: &Path,
    frames: &[RecordingFrame],
    metadata: Value,
) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let mut log = McapLog::new(BufWriter::new(
        File::create(path).map_err(|e| e.to_string())?,
    ))?;
    log.event(
        "segment_started",
        json!({"segment_id":1,"metadata":metadata}),
    )?;
    for frame in frames {
        log.frame(frame)?;
    }
    log.event("recording_stopped", json!({"reason":"test_completed"}))?;
    log.finish()
}

#[cfg(test)]
pub(crate) fn report_reference(report: &Value) {
    eprintln!("{report}");
    if let Ok(path) = std::env::var("MICRODUCK_COMPARISON_REPORT") {
        let path = PathBuf::from(path);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("create report directory");
        }
        fs::write(path, serde_json::to_vec_pretty(report).unwrap())
            .expect("write requested comparison report");
    }
}

fn is_id_character(character: char) -> bool {
    character.is_alphanumeric() || matches!(character, '-' | '_' | '.')
}

fn recording_path(config: Option<&Path>, id: &str) -> Result<PathBuf, String> {
    if id.is_empty() || matches!(id, "." | "..") || !id.chars().all(is_id_character) {
        return Err("无效的采集 ID".into());
    }
    Ok(root(config).join(id))
}

pub(crate) fn reveal_file(config: Option<&Path>, id: &str, motion: bool) -> Result<(), String> {
    let file = if motion {
        "motion.jsonl"
    } else {
        "runtime.mcap"
    };
    let path = recording_path(config, id)?.join(file);
    tauri_plugin_opener::reveal_item_in_dir(path)
        .map_err(|error| format!("定位采集文件失败：{error}"))
}

pub(crate) fn list(
    config: Option<&Path>,
    recorder: &Recorder,
) -> Result<Vec<RecordingInfo>, String> {
    let directory = root(config);
    if !directory.exists() {
        return Ok(Vec::new());
    }
    let current = recorder.current();
    let mut records = Vec::new();
    for entry in fs::read_dir(directory).map_err(|e| e.to_string())? {
        let path = entry
            .map_err(|e| e.to_string())?
            .path()
            .join("manifest.json");
        if !path.is_file() {
            continue;
        }
        let mut info: RecordingInfo =
            serde_json::from_slice(&fs::read(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if let Some(active) = &current
            && active.id == info.id
        {
            info = active.clone();
        } else if info.state == "recording" {
            info.state = "interrupted".into();
        }
        records.push(info);
    }
    records.sort_by(|a, b| b.started_at_ms.total_cmp(&a.started_at_ms));
    Ok(records)
}

fn frame_stream(
    config: Option<&Path>,
    window: &RecordingWindow,
) -> Result<impl Iterator<Item = Result<RecordingFrame, String>>, String> {
    if !window.start_s.is_finite()
        || !window.end_s.is_finite()
        || window.start_s < 0.0
        || window.end_s < window.start_s
    {
        return Err("时间范围无效".into());
    }
    let file = std::io::BufReader::new(
        File::open(recording_path(config, &window.id)?.join("runtime.mcap"))
            .map_err(|e| e.to_string())?,
    );
    let window = window.clone();
    Ok(
        mcap::io::MessageReader::new(file).filter_map(move |message| {
            let message = match message {
                Ok(message) => message,
                Err(mcap::McapError::UnexpectedEof | mcap::McapError::UnexpectedEoc) => {
                    return None;
                }
                Err(error) => return Some(Err(error.to_string())),
            };
            if message.channel.topic != "/duck/runtime" {
                return None;
            }
            let frame: RecordingFrame = match serde_json::from_slice(&message.data) {
                Ok(frame) => frame,
                Err(error) => return Some(Err(error.to_string())),
            };
            ((window.segment_id == 0 || frame.segment_id == window.segment_id)
                && frame.elapsed_s >= window.start_s
                && frame.elapsed_s <= window.end_s)
                .then_some(Ok(frame))
        }),
    )
}

#[cfg(test)]
pub(crate) fn frames(
    config: Option<&Path>,
    window: &RecordingWindow,
) -> Result<Vec<RecordingFrame>, String> {
    frame_stream(config, window)?.collect()
}

pub(crate) fn preview_frames(
    config: Option<&Path>,
    window: &RecordingWindow,
) -> Result<Vec<RecordingPreviewFrame>, String> {
    let mut samples = Vec::new();
    let mut stride = 1;
    for (index, frame) in frame_stream(config, window)?.enumerate() {
        let frame = frame?;
        if index % stride != 0 {
            continue;
        }
        samples.push(frame);
        if samples.len() >= 4000 {
            samples = samples.into_iter().step_by(2).collect();
            stride *= 2;
        }
    }
    if samples.len() > 2000 {
        samples = samples.into_iter().step_by(2).collect();
    }
    let segments = segment_metadata(config, &window.id)?;
    let execution = segments
        .iter()
        .map(|(&id, metadata)| Ok((id, recorded_execution(metadata)?)))
        .collect::<Result<BTreeMap<_, _>, String>>()?;
    Ok(samples
        .into_iter()
        .map(|frame| RecordingPreviewFrame {
            execution: execution.get(&frame.segment_id).cloned().flatten(),
            frame,
        })
        .collect())
}

fn recorded_execution(metadata: &Value) -> Result<Option<RecordingExecution>, String> {
    let physical = &metadata["physical_execution"];
    let simulation = metadata
        .get("simulation_execution")
        .filter(|value| !value.is_null())
        .unwrap_or(&physical["fallback_simulation"]);
    if physical.is_null() && simulation.is_null() {
        return Ok(None);
    }
    let fields = (|| {
        Ok(RecordingExecution {
            simulation: serde_json::from_value(simulation.clone())?,
            physical: serde_json::from_value(physical["physical"].clone())?,
            slots: physical
                .get("slots")
                .cloned()
                .map(serde_json::from_value)
                .transpose()?
                .unwrap_or_default(),
        })
    })();
    fields
        .map(Some)
        .map_err(|error: serde_json::Error| error.to_string())
}
/// Compare canonical fields using one-to-one timestamp matching within one control period.
pub(crate) fn compare(
    config: Option<&Path>,
    left: RecordingWindow,
    right: RecordingWindow,
) -> Result<RecordingComparison, String> {
    let a = frame_stream(config, &left)?;
    let mut b = frame_stream(config, &right)?.peekable();
    let mut count_a = 0;
    let mut count_b = 0;
    let mut metrics: HashMap<String, (u32, f64, f64)> = HashMap::new();
    let mut matched = 0;
    let mut warnings = Vec::new();
    let left_metadata = segment_metadata(config, &left.id)?;
    let right_metadata = segment_metadata(config, &right.id)?;
    for frame in a {
        let frame = frame?;
        count_a += 1;
        let time = frame.elapsed_s - left.start_s;
        while let Some(next) = b.peek() {
            match next {
                Ok(next) if next.elapsed_s - right.start_s < time - 0.02 => {
                    b.next();
                    count_b += 1;
                }
                Err(error) => return Err(error.clone()),
                _ => break,
            }
        }
        let Some(other) = b.peek() else { continue };
        let other = other.as_ref().map_err(Clone::clone)?;
        if (other.elapsed_s - right.start_s - time).abs() > 0.02 {
            continue;
        }
        let other = b.next().expect("peeked frame")?;
        count_b += 1;
        matched += 1;
        accumulate(
            &mut metrics,
            "positions_rad",
            &frame.status.positions,
            &other.status.positions,
        );
        accumulate(
            &mut metrics,
            "targets_rad",
            &frame.status.targets,
            &other.status.targets,
        );
        if let (Some(a), Some(b)) = (&frame.policy, &other.policy) {
            let same_contract = left_metadata
                .get(&frame.segment_id)
                .and_then(|m| m.get("policy_contract"))
                .zip(
                    right_metadata
                        .get(&other.segment_id)
                        .and_then(|m| m.get("policy_contract")),
                )
                .is_some_and(|(a, b)| a == b);
            if same_contract && a.action.len() == b.action.len() {
                accumulate(
                    &mut metrics,
                    "action",
                    &a.action.iter().map(|v| f64::from(*v)).collect::<Vec<_>>(),
                    &b.action.iter().map(|v| f64::from(*v)).collect::<Vec<_>>(),
                );
            } else if warnings.is_empty() {
                warnings.push("策略契约不同，已跳过原始 action 比较".into());
            }
        }
    }
    for frame in b {
        frame?;
        count_b += 1;
    }
    if count_a == 0 || count_b == 0 {
        return Err("所选区间没有可比较的数据".into());
    }
    let mut metrics = metrics
        .into_iter()
        .map(|(field, (samples, sum, max_error))| RecordingMetric {
            field,
            samples,
            rmse: (sum / f64::from(samples)).sqrt(),
            max_error,
        })
        .collect::<Vec<_>>();
    metrics.sort_by(|a, b| a.field.cmp(&b.field));
    let path =
        recording_path(config, &left.id)?.join(format!("comparison-{}.json", now_ms() as u64));
    let report = RecordingComparison {
        metrics,
        matched_frames: matched,
        unmatched_frames: count_a + count_b - matched * 2,
        path: path.to_string_lossy().into_owned(),
        warnings,
    };
    fs::write(
        &path,
        serde_json::to_vec_pretty(&json!({"left":left,"right":right,"report":report}))
            .map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(report)
}

pub(crate) fn accumulate(
    metrics: &mut HashMap<String, (u32, f64, f64)>,
    field: &str,
    a: &[f64],
    b: &[f64],
) {
    if a.is_empty() || a.len() != b.len() {
        return;
    }
    let metric = metrics.entry(field.into()).or_default();
    for (a, b) in a.iter().zip(b) {
        if a.is_finite() && b.is_finite() {
            let error = (a - b).abs();
            metric.0 += 1;
            metric.1 += error * error;
            metric.2 = metric.2.max(error);
        }
    }
}

fn motion_sample(frame: &RecordingFrame) -> Option<Value> {
    let policy = frame.policy.as_ref()?;
    (frame.status.runtime_state == crate::types::RuntimeState::Running
        && frame.status.sensor_failures == 0
        && frame.status.active_body.is_some()
        && policy.valid.iter().any(|valid| *valid)
        && !frame.status.positions.is_empty()
        && frame.status.positions.iter().all(|value| value.is_finite()))
    .then(|| {
        json!({"recording_id":frame.recording_id,"duck_id":frame.duck_id,
            "segment_id":frame.segment_id,"episode_id":frame.episode_id,
            "tick":frame.status.tick,"time_s":frame.elapsed_s,"positions":frame.status.positions,
            "velocities":policy.velocities,"valid":policy.valid,"simulation":frame.status.simulation})
    })
}

pub(crate) fn export(
    config: Option<&Path>,
    window: RecordingWindow,
    kind: &str,
) -> Result<RecordingExport, String> {
    if kind != "policy" && kind != "motion" {
        return Err("未知的教师数据类型".into());
    }
    let segments = segment_metadata(config, &window.id)?;
    let mut contract = None;
    if kind == "policy" {
        for frame in frame_stream(config, &window)? {
            let frame = frame?;
            if frame.policy.is_none() {
                continue;
            }
            let next = segments
                .get(&frame.segment_id)
                .and_then(|metadata| metadata.get("policy_contract"))
                .filter(|value| !value.is_null())
                .ok_or("片段缺少策略数据定义")?;
            if contract.as_ref().is_some_and(|current| current != next) {
                return Err("所选片段包含不同策略契约，请分别导出".into());
            }
            contract = Some(next.clone());
        }
    }
    let path =
        recording_path(config, &window.id)?.join(format!("teacher-{kind}-{}", now_ms() as u64));
    fs::create_dir(&path).map_err(|e| e.to_string())?;
    let mut writer =
        BufWriter::new(File::create(path.join("samples.jsonl")).map_err(|e| e.to_string())?);
    let mut samples = 0;
    let mut excluded = 0;
    for frame in frame_stream(config, &window)? {
        let frame = frame?;
        let good = frame.status.runtime_state == crate::types::RuntimeState::Running
            && frame.status.sensor_failures == 0
            && frame.status.active_body.is_some();
        let sample = match (&frame.policy, kind) {
            (Some(policy), "policy")
                if good
                    && !policy.valid.is_empty()
                    && policy.valid.iter().all(|valid| *valid)
                    && !policy.observation.is_empty()
                    && !policy.action.is_empty()
                    && contract.as_ref().is_some_and(|contract| {
                        contract["obs_len"].as_u64() == Some(policy.observation.len() as u64)
                            && contract["action_len"].as_u64() == Some(policy.action.len() as u64)
                    })
                    && policy
                        .observation
                        .iter()
                        .chain(&policy.action)
                        .all(|v| v.is_finite()) =>
            {
                json!({"recording_id":frame.recording_id,"duck_id":frame.duck_id,
                    "segment_id":frame.segment_id,"episode_id":frame.episode_id,
                    "tick":frame.status.tick,"time_s":frame.elapsed_s,
                    "observation":policy.observation,"teacher_action":policy.action})
            }
            (_, "motion") => match motion_sample(&frame) {
                Some(sample) => sample,
                None => {
                    excluded += 1;
                    continue;
                }
            },
            _ => {
                excluded += 1;
                continue;
            }
        };
        serde_json::to_writer(&mut writer, &sample).map_err(|e| e.to_string())?;
        writeln!(writer).map_err(|e| e.to_string())?;
        samples += 1;
    }
    writer.flush().map_err(|e| e.to_string())?;
    fs::write(
        path.join("dataset.json"),
        serde_json::to_vec_pretty(&json!({
            "format":"microduck-teacher","kind":kind,"source":window,"samples":samples,
            "excluded_frames":excluded,"joint_units":"rad","action_space":"raw_policy",
            "normalization":"recorded_policy_input","source_mcap":"../runtime.mcap",
            "policy_contract":contract,"segments":segments,
        }))
        .map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    Ok(RecordingExport {
        path: path.to_string_lossy().into_owned(),
        samples,
        excluded_frames: excluded,
    })
}

fn segment_metadata(config: Option<&Path>, id: &str) -> Result<BTreeMap<u32, Value>, String> {
    let data = std::io::BufReader::new(
        File::open(recording_path(config, id)?.join("runtime.mcap")).map_err(|e| e.to_string())?,
    );
    let mut segments = BTreeMap::new();
    for message in mcap::io::MessageReader::new(data) {
        let message = match message {
            Ok(message) => message,
            Err(mcap::McapError::UnexpectedEof | mcap::McapError::UnexpectedEoc) => break,
            Err(error) => return Err(error.to_string()),
        };
        if message.channel.topic != "/duck/events" {
            continue;
        }
        let event: Value = serde_json::from_slice(&message.data).map_err(|e| e.to_string())?;
        if event["event"] == "segment_started"
            && let Some(segment) = event["context"]["segment_id"].as_u64()
        {
            segments.insert(segment as u32, event["context"]["metadata"].clone());
        }
    }
    Ok(segments)
}

impl From<&PolicyTrace> for RecordingPolicy {
    fn from(trace: &PolicyTrace) -> Self {
        Self {
            observation: trace.observation.clone(),
            action: trace.action.clone(),
            raw_targets: trace.raw_targets.clone(),
            applied_targets: trace.body_targets.clone(),
            twist: trace.command.twist,
            head: trace.command.head,
            body: [
                trace.command.body.z,
                trace.command.body.roll,
                trace.command.body.pitch,
            ],
            positions: trace.backend_state.positions.clone(),
            velocities: trace.backend_state.velocities.clone(),
            valid: trace.backend_state.valid_positions.clone(),
            gyro: trace.imu.gyro,
            gravity: trace.imu.gravity,
            imu_source: trace.imu_source.into(),
            saturation_reasons: trace
                .saturation_reasons
                .iter()
                .map(|s| (*s).into())
                .collect(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_keeps_model_fingerprints_and_slot_routes_from_each_recorded_segment() {
        let state = crate::runtime::session::state::DesktopState::default();
        let machine = state.virtual_machines.lock().unwrap()[0].id.clone();
        let mut status = crate::runtime::session::preview::open(&state, machine).unwrap();
        let simulation = state
            .runtime_loop
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .recording_metadata();
        let simulated = simulation["simulation_execution"].clone();
        let physical = json!({
            "physical_execution": {
                "physical": {
                    "execution_kind": "physical", "model_kind": "hd1910", "servo_model": "hd1910",
                    "execution_fingerprint": "physical-test-fingerprint"
                },
                "fallback_simulation": simulated,
                "slots": [
                    {"slot_id": "left_hip_yaw", "physical_id": 1, "source": "physical"},
                    {"slot_id": "left_hip_roll", "physical_id": null, "source": "simulation"}
                ]
            }
        });
        let config =
            crate::platform::helpers::test_directory().join(uuid::Uuid::new_v4().to_string());
        let recorder = Recorder::default();
        let info = recorder
            .start(
                Some(&config),
                &status,
                "provenance".into(),
                simulation.clone(),
            )
            .unwrap();
        status.execution_generation = 1;
        recorder.capture(&status, None, || simulation, Instant::now());
        status.execution_generation = 2;
        recorder.capture(&status, None, || physical, Instant::now());
        recorder.stop("stopped").unwrap().unwrap();
        let frames = preview_frames(
            Some(&config),
            &RecordingWindow {
                id: info.id,
                segment_id: 0,
                start_s: 0.0,
                end_s: 100.0,
            },
        )
        .unwrap();
        assert_eq!(frames.len(), 2);
        let from_simulation = frames[0].execution.as_ref().unwrap();
        let from_physical = frames[1].execution.as_ref().unwrap();
        let model = from_simulation.simulation.as_ref().unwrap();
        assert_eq!(
            model.model_kind,
            crate::types::ModelKindKey::Hd1910Compatible
        );
        assert!(from_simulation.physical.is_none());
        assert_eq!(
            from_physical
                .simulation
                .as_ref()
                .unwrap()
                .execution_fingerprint,
            model.execution_fingerprint
        );
        assert_eq!(
            from_physical
                .physical
                .as_ref()
                .unwrap()
                .execution_fingerprint,
            "physical-test-fingerprint"
        );
        assert_eq!(from_physical.slots[0].physical_id, Some(1));
        assert_eq!(from_physical.slots[1].source, "simulation");
    }

    #[test]
    fn mcap_roundtrip_preserves_segments_comparison_and_teacher_labels() {
        let state = crate::runtime::session::state::DesktopState::default();
        let machine = state.virtual_machines.lock().unwrap()[0].id.clone();
        let status = crate::runtime::session::preview::open(&state, machine).unwrap();
        let directory = crate::platform::helpers::test_directory().join(format!(
            "microduck-recording-tests-{}-{}",
            app_session_id(),
            now_ms() as u64
        ));
        let config = directory.join("config");
        let recorder = Recorder::default();
        let metadata = json!({"policy_contract":{"obs_len":2,"action_len":1,"recurrent":false}});
        let info = recorder
            .start(Some(&config), &status, "velstand".into(), metadata.clone())
            .unwrap();
        assert!(info.id.starts_with("velstand-"));
        assert_eq!(root(Some(&config)), directory.join("recordings"));
        let trace = PolicyTrace {
            phase: loop_body::LoopPhase::Driving,
            command: duck_control::obs::Command::default(),
            imu: duck_control::imu::ImuData::default(),
            imu_source: "backend",
            observation: vec![0.2, 0.3],
            action: vec![0.7],
            body_targets: Some(vec![0.1; 15]),
            raw_targets: Some(vec![0.2; 15]),
            backend_state: loop_body::BackendState {
                positions: vec![0.1; 15],
                velocities: vec![0.0; 15],
                imu: None,
                sequence: 1,
                sampled_at: loop_body::SampleTime(1),
                valid_positions: vec![true; 15],
                valid_velocities: vec![true; 15],
                source: Some("test".into()),
            },
            execution_positions: None,
            saturation_reasons: vec![],
            stages: vec!["read", "write"],
        };
        let mut status = status;
        let mut trace = trace;
        for tick in 0..4 {
            status.tick = tick;
            status.execution_generation = if tick < 2 { 1 } else { 2 };
            trace.backend_state.valid_positions[9] = tick != 3;
            recorder.capture(&status, Some(&trace), || metadata.clone(), Instant::now());
        }
        let stopped = recorder.stop("stopped").unwrap().unwrap();
        assert_eq!(stopped.state, "complete");
        assert_eq!(stopped.frames, 4);
        let motion = fs::read_to_string(Path::new(&info.path).join("motion.jsonl")).unwrap();
        let samples: Vec<Value> = motion
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert_eq!(samples.len(), 4);
        assert_eq!(samples[3]["tick"], 3);
        assert_eq!(samples[3]["episode_id"], 2);
        assert_eq!(samples[3]["valid"][9], false);
        let window = RecordingWindow {
            id: info.id,
            segment_id: 0,
            start_s: 0.0,
            end_s: 100.0,
        };
        let frames = frames(Some(&config), &window).unwrap();
        assert_eq!(frames.len(), 4);
        assert_eq!(frames[0].segment_id, 1);
        assert_eq!(frames[3].segment_id, 2);
        assert_eq!(frames[0].policy.as_ref().unwrap().action, vec![0.7]);
        let report = compare(Some(&config), window.clone(), window.clone()).unwrap();
        assert_eq!(report.matched_frames, 4);
        assert!(report.metrics.iter().all(|metric| metric.max_error == 0.0));
        let exported = export(Some(&config), window, "policy").unwrap();
        assert_eq!(exported.samples, 3);
        let samples = fs::read_to_string(Path::new(&exported.path).join("samples.jsonl")).unwrap();
        let sample: Value = serde_json::from_str(samples.lines().next().unwrap()).unwrap();
        assert_eq!(sample["teacher_action"][0].as_f64().unwrap() as f32, 0.7);
        let events = fs::read_to_string(logging::current_log_file(Some(&config))).unwrap();
        for event in [
            "recording_started",
            "recording_segment_started",
            "recording_stopped",
            "recording_finished",
        ] {
            assert!(events.contains(event));
        }
        assert!(events.contains(&stopped.id));
    }
}
