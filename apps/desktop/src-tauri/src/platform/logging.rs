//! JSON Lines diagnostics for application events and hardware control-loop I/O.

use std::fs::{OpenOptions, create_dir_all};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::runtime::policy::inference::PolicyTrace;

use crate::platform::state::types::JointRangeState;

pub(crate) fn log_directory(config_dir: Option<&Path>) -> PathBuf {
    config_dir
        .and_then(Path::parent)
        .map(|root| root.join("logs"))
        .unwrap_or_else(|| std::env::temp_dir().join("microduck-vision").join("logs"))
}

fn date_stamp() -> String {
    let days = (timestamp_millis() / 86_400_000) as i64;
    let z = days + 719_468;
    let era = if z >= 0 {
        z / 146_097
    } else {
        (z - 146_096) / 146_097
    };
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let year = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let month_part = (5 * doy + 2) / 153;
    let day = doy - (153 * month_part + 2) / 5 + 1;
    let month = month_part + if month_part < 10 { 3 } else { -9 };
    let year = year + if month <= 2 { 1 } else { 0 };
    format!("{year:04}-{month:02}-{day:02}")
}

pub(crate) fn current_log_file(config_dir: Option<&Path>) -> PathBuf {
    log_directory(config_dir).join(format!("{}.jsonl", date_stamp()))
}

fn timestamp_millis() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or_default()
}

fn append_line(path: &Path, line: &str) {
    let event = line.split_whitespace().next().unwrap_or("event");
    let level =
        if event.contains("error") || event.contains("fault") || line.contains("result=failed") {
            "error"
        } else if event.contains("missed") || event.contains("violation") {
            "warn"
        } else {
            "info"
        };
    enqueue(
        path,
        serde_json::json!({
            "timestamp_ms":timestamp_millis() as u64, "app_session_id":crate::runtime::recordings::app_session_id(),
            "category":event.split('_').next().unwrap_or("control"),"level":level,
            "event":event,"message":line,
            "context":contexts().lock().ok().and_then(|contexts| contexts.get(path).cloned()),
        }),
    );
}

pub(crate) fn diagnostic(
    config: Option<&Path>,
    category: &str,
    level: &str,
    event: &str,
    context: serde_json::Value,
) {
    enqueue(
        &current_log_file(config),
        serde_json::json!({
            "timestamp_ms":timestamp_millis() as u64,"app_session_id":crate::runtime::recordings::app_session_id(),
            "category":category,"level":level,"event":event,"context":context,
        }),
    );
}

pub(crate) fn frontend_diagnostic(config: Option<&Path>, message: String) {
    let message = message.chars().take(8192).collect::<String>();
    if let Ok(context) = serde_json::from_str::<serde_json::Value>(&message)
        && context.get("event").and_then(serde_json::Value::as_str) == Some("debug_pose_applied")
    {
        diagnostic(config, "debug", "info", "debug_pose_applied", context);
    } else {
        diagnostic(
            config,
            "frontend",
            "error",
            "frontend_error",
            serde_json::json!({"message":message}),
        );
    }
}

#[cfg(test)]
fn write_event(path: &Path, event: &serde_json::Value) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        create_dir_all(parent)?;
    }
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    serde_json::to_writer(&mut file, event)?;
    writeln!(file)
}

#[cfg(not(test))]
enum DiagnosticPacket {
    Write(PathBuf, serde_json::Value),
    Flush(std::sync::mpsc::SyncSender<()>),
}

#[cfg(not(test))]
fn diagnostic_queue() -> &'static std::sync::mpsc::SyncSender<DiagnosticPacket> {
    use std::collections::HashMap;
    use std::io::BufWriter;
    use std::sync::{OnceLock, mpsc};
    static QUEUE: OnceLock<mpsc::SyncSender<DiagnosticPacket>> = OnceLock::new();
    QUEUE.get_or_init(|| {
        let (sender, receiver) = mpsc::sync_channel(8192);
        std::thread::spawn(move || {
            let mut files: HashMap<PathBuf, BufWriter<std::fs::File>> = HashMap::new();
            let mut last_flush = std::time::Instant::now();
            loop {
                let mut acknowledge = None;
                match receiver.recv_timeout(std::time::Duration::from_millis(250)) {
                    Ok(DiagnosticPacket::Write(path, event)) => {
                        if !files.contains_key(&path) {
                            if let Some(parent) = path.parent() {
                                let _ = create_dir_all(parent);
                            }
                            match OpenOptions::new().create(true).append(true).open(&path) {
                                Ok(file) => {
                                    files.insert(path.clone(), BufWriter::new(file));
                                }
                                Err(error) => {
                                    eprintln!("diagnostic write failed: {error}");
                                    continue;
                                }
                            }
                        }
                        let file = files.get_mut(&path).expect("opened log");
                        if let Err(error) = serde_json::to_writer(&mut *file, &event)
                            .map_err(std::io::Error::other)
                            .and_then(|()| writeln!(file))
                        {
                            eprintln!("diagnostic write failed: {error}");
                        }
                    }
                    Ok(DiagnosticPacket::Flush(done)) => {
                        acknowledge = Some(done);
                    }
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                    Err(mpsc::RecvTimeoutError::Timeout) => {}
                }
                if acknowledge.is_some()
                    || last_flush.elapsed() >= std::time::Duration::from_millis(250)
                {
                    for file in files.values_mut() {
                        if let Err(error) = file.flush() {
                            eprintln!("diagnostic flush failed: {error}");
                        }
                    }
                    last_flush = std::time::Instant::now();
                }
                if let Some(done) = acknowledge {
                    let _ = done.send(());
                }
            }
        });
        sender
    })
}

pub(crate) fn flush() {
    #[cfg(not(test))]
    {
        let (done, result) = std::sync::mpsc::sync_channel(1);
        if diagnostic_queue()
            .send(DiagnosticPacket::Flush(done))
            .is_ok()
        {
            let _ = result.recv();
        }
    }
}

fn enqueue(path: &Path, event: serde_json::Value) {
    #[cfg(test)]
    if let Err(error) = write_event(path, &event) {
        eprintln!("diagnostic write failed: {error}");
    }
    #[cfg(not(test))]
    if diagnostic_queue()
        .try_send(DiagnosticPacket::Write(path.to_owned(), event))
        .is_err()
    {
        eprintln!("diagnostic queue full or closed");
    }
}

type ContextMap = std::sync::Mutex<std::collections::HashMap<PathBuf, serde_json::Value>>;
fn contexts() -> &'static ContextMap {
    static CONTEXT: std::sync::OnceLock<ContextMap> = std::sync::OnceLock::new();
    CONTEXT.get_or_init(ContextMap::default)
}

pub(crate) fn set_context(config: Option<&Path>, context: serde_json::Value) {
    if let Ok(mut contexts) = contexts().lock() {
        contexts.insert(current_log_file(config), context);
    }
}

pub(crate) fn append_handle(handle: &LogHandle, line: &str) {
    append_line(handle, line);
}

pub(crate) fn log_joint_range_states(config_dir: Option<&Path>, states: &[JointRangeState]) {
    let violations = states
        .iter()
        .filter(|state| state.below_min || state.above_max)
        .map(|state| {
            format!(
                "{} target={} range=[{},{}] side={}",
                state.slot_id,
                state.target_rad,
                state.min_rad,
                state.max_rad,
                if state.below_min { "min" } else { "max" }
            )
        })
        .collect::<Vec<_>>();
    if !violations.is_empty() {
        append_control_event(
            config_dir,
            format!("joint_range_violation {}", violations.join(" | ")),
        );
    }
}

pub(crate) type LogHandle = Arc<PathBuf>;

pub(crate) fn begin_control_loop(
    config_dir: Option<&Path>,
    machine_id: &str,
    machine_name: &str,
    binding_summary: &str,
) -> Option<LogHandle> {
    let path = current_log_file(config_dir);
    append_line(
        &path,
        &format!(
            "loop_open machine_id={machine_id} machine_name={machine_name:?} binding={binding_summary}"
        ),
    );
    Some(Arc::new(path))
}
pub(crate) fn append_policy_trace(config_dir: Option<&Path>, tick: u64, trace: &PolicyTrace) {
    let imu_observation = trace.observation.get(..6).unwrap_or_default();
    let joint_position_observation = trace.observation.get(6..20).unwrap_or_default();
    let joint_velocity_observation = trace.observation.get(20..34).unwrap_or_default();
    let previous_action_observation = trace.observation.get(34..48).unwrap_or_default();
    let body_targets = trace
        .body_targets
        .as_deref()
        .map(|targets| format!("{targets:?}"))
        .unwrap_or_else(|| "none".into());
    let raw_targets = trace
        .raw_targets
        .as_deref()
        .map(|targets| format!("{targets:?}"))
        .unwrap_or_else(|| "none".into());
    let execution_positions = trace
        .execution_positions
        .as_deref()
        .map(|positions| format!("{positions:?}"))
        .unwrap_or_else(|| "none".into());
    append_line(
        &current_log_file(config_dir),
        &format!(
            "policy_trace tick={} phase={:?} stages={:?} imu_source={} imu_gyro={:?} imu_gravity={:?} imu_observation={:?} joint_position_observation={:?} joint_velocity_observation={:?} previous_action_observation={:?} command_twist={:?} command_head={:?} command_body=[{}, {}, {}] observation_len={} action_len={} observation={:?} onnx_action={:?} raw_targets={} body_targets={} backend_positions={:?} backend_velocities={:?} execution_positions={} saturation_reason={:?}",
            tick,
            trace.phase,
            trace.stages,
            trace.imu_source,
            trace.imu.gyro,
            trace.imu.gravity,
            imu_observation,
            joint_position_observation,
            joint_velocity_observation,
            previous_action_observation,
            trace.command.twist,
            trace.command.head,
            trace.command.body.z,
            trace.command.body.roll,
            trace.command.body.pitch,
            trace.observation.len(),
            trace.action.len(),
            trace.observation,
            trace.action,
            raw_targets,
            body_targets,
            trace.backend_state.positions,
            trace.backend_state.velocities,
            execution_positions,
            trace.saturation_reasons
        ),
    );
}

pub(crate) fn append_control_event(config_dir: Option<&Path>, message: impl AsRef<str>) {
    append_line(&current_log_file(config_dir), message.as_ref());
}

/// Keep backend failures distinguishable in the trace without making the loop core depend on
/// transport-specific error types.
#[cfg(test)]
pub(crate) fn saturation_reason_for_error(error: &str) -> &'static str {
    let error = error.to_ascii_lowercase();
    if error.contains("register") || error.contains("encode") {
        "register_encoding"
    } else if error.contains("speed") || error.contains("velocity") {
        "velocity_limit"
    } else if error.contains("range") || error.contains("target") {
        "body_range_limit"
    } else if error.contains("delay") || error.contains("transport") {
        "actuator_delay"
    } else if error.contains("dynamics") || error.contains("inertia") {
        "dynamics_limit"
    } else {
        "backend_error"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::SystemTime;

    #[test]
    fn policy_trace_log_keeps_backend_and_saturation_diagnostics() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = crate::platform::helpers::test_directory()
            .join(format!("microduck-policy-log-{suffix}"));
        let config_dir = root.join("config");
        let trace = crate::runtime::policy::inference::PolicyTrace {
            phase: loop_body::LoopPhase::Driving,
            command: duck_control::obs::Command::default(),
            imu: duck_control::imu::ImuData::default(),
            imu_source: "backend",
            observation: vec![0.0; 61],
            action: vec![0.0; 14],
            body_targets: Some(vec![0.2]),
            raw_targets: Some(vec![0.3]),
            backend_state: loop_body::BackendState {
                positions: vec![0.1],
                velocities: vec![0.4],
                imu: None,
                sequence: 4,
                sampled_at: loop_body::SampleTime(4),
                valid_positions: vec![true],
                valid_velocities: vec![true],
                source: Some("test".into()),
            },
            execution_positions: Some(vec![0.15]),
            saturation_reasons: vec!["target_filter"],
            stages: vec!["read", "write"],
        };
        append_policy_trace(Some(&config_dir), 4, &trace);
        let text = std::fs::read_to_string(current_log_file(Some(&config_dir))).unwrap();
        assert!(text.contains("backend_positions=[0.1]"));
        assert!(text.contains("backend_velocities=[0.4]"));
        assert!(text.contains("execution_positions=[0.15]"));
        assert!(text.contains("target_filter"));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn error_diagnostics_keep_transport_causes_distinguishable() {
        assert_eq!(
            saturation_reason_for_error("register encode failed"),
            "register_encoding"
        );
        assert_eq!(
            saturation_reason_for_error("velocity limit"),
            "velocity_limit"
        );
        assert_eq!(
            saturation_reason_for_error("target out of range"),
            "body_range_limit"
        );
        assert_eq!(
            saturation_reason_for_error("transport delay"),
            "actuator_delay"
        );
        assert_eq!(
            saturation_reason_for_error("dynamics inertia"),
            "dynamics_limit"
        );
        assert_eq!(
            saturation_reason_for_error("serial disconnected"),
            "backend_error"
        );
    }

    #[test]
    fn diagnostic_log_keeps_each_saturation_source_parseable() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = crate::platform::helpers::test_directory()
            .join(format!("microduck-diagnostic-log-{suffix}"));
        let config_dir = root.join("config");
        for reason in [
            "body_range_exceeded",
            "target_filter",
            "actuator_delay",
            "dynamics_limit",
            "body_range_limit",
            "register_encoding",
        ] {
            append_control_event(Some(&config_dir), format!("saturation_reason={reason}"));
        }
        let text = std::fs::read_to_string(current_log_file(Some(&config_dir))).unwrap();
        for reason in [
            "body_range_exceeded",
            "target_filter",
            "actuator_delay",
            "dynamics_limit",
            "body_range_limit",
            "register_encoding",
        ] {
            assert!(text.contains(&format!("saturation_reason={reason}")));
        }
        let _ = std::fs::remove_dir_all(root);
    }
}
