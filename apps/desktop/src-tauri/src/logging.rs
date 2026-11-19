//! Small file logger for hardware control-loop I/O.

use std::fs::{File, OpenOptions, create_dir_all};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::backend::policy_runtime::PolicyTrace;
use loop_body::ControlBackend;
use servo::{ServoError, ServoImuSample, ServoPositionSample, ServoPositionTargets};

use crate::state::types::JointRangeState;

fn log_directory(config_dir: Option<&Path>) -> PathBuf {
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
    log_directory(config_dir).join(format!("{}.log", date_stamp()))
}

fn timestamp_millis() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or_default()
}

fn append_line(path: &Path, line: &str) {
    let result = (|| -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            create_dir_all(parent)?;
        }
        let mut file = OpenOptions::new().create(true).append(true).open(path)?;
        writeln!(file, "[{}] {line}", timestamp_millis())
    })();
    if let Err(error) = result {
        eprintln!("control-loop log write failed: {error}");
    }
}

fn append_handle(handle: &LogHandle, line: &str) {
    let result = (|| -> std::io::Result<()> {
        let mut writer = handle
            .lock()
            .map_err(|_| std::io::Error::other("control-loop log lock poisoned"))?;
        writer.rotate_if_needed()?;
        writeln!(writer.file, "[{}] {line}", timestamp_millis())?;
        writer.file.flush()
    })();
    if let Err(error) = result {
        eprintln!("control-loop log write failed: {error}");
    }
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

pub(crate) struct LogWriter {
    config_dir: Option<PathBuf>,
    day: String,
    file: File,
}

impl LogWriter {
    fn open(config_dir: Option<&Path>) -> std::io::Result<Self> {
        let day = date_stamp();
        let path = current_log_file(config_dir);
        if let Some(parent) = path.parent() {
            create_dir_all(parent)?;
        }
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self {
            config_dir: config_dir.map(Path::to_path_buf),
            day,
            file,
        })
    }

    fn rotate_if_needed(&mut self) -> std::io::Result<()> {
        let day = date_stamp();
        if day == self.day {
            return Ok(());
        }
        let next = Self::open(self.config_dir.as_deref())?;
        self.day = next.day;
        self.file = next.file;
        Ok(())
    }
}

pub(crate) type LogHandle = Arc<Mutex<LogWriter>>;

pub(crate) fn begin_control_loop(
    config_dir: Option<&Path>,
    machine_id: &str,
    machine_name: &str,
    binding_summary: &str,
) -> Option<LogHandle> {
    let mut writer = match LogWriter::open(config_dir) {
        Ok(writer) => writer,
        Err(error) => {
            eprintln!("control-loop log initialization failed: {error}");
            return None;
        }
    };
    let result = writeln!(
        writer.file,
        "[{}] loop_open machine_id={} machine_name={:?} binding={ids}",
        timestamp_millis(),
        machine_id,
        machine_name,
        ids = binding_summary
    );
    if let Err(error) = result {
        eprintln!("control-loop log initialization failed: {error}");
        return None;
    }
    Some(Arc::new(Mutex::new(writer)))
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

pub(crate) struct LoggedServoDriver {
    inner: Box<dyn ControlBackend>,
    handle: LogHandle,
    machine_id: String,
}

impl LoggedServoDriver {
    pub(crate) fn new(
        inner: Box<dyn ControlBackend>,
        handle: LogHandle,
        machine_id: impl Into<String>,
    ) -> Self {
        Self {
            inner,
            handle,
            machine_id: machine_id.into(),
        }
    }

    fn line(&self, message: impl AsRef<str>) {
        append_handle(
            &self.handle,
            &format!("machine_id={} {}", self.machine_id, message.as_ref()),
        );
    }
}

impl servo::ServoLoopDriver for LoggedServoDriver {
    fn set_read_cancel(&mut self, cancel: Arc<std::sync::atomic::AtomicBool>) {
        self.inner.set_read_cancel(cancel);
    }
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        self.inner.feedback_validity(ids)
    }
    fn advance_execution(&mut self, steps: u32, dt: std::time::Duration) -> Result<(), ServoError> {
        self.inner.advance_execution(steps, dt)
    }

    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.inner.reset_execution()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        let result = self.inner.read_positions(ids);
        match &result {
            Ok(sample) => {
                for (index, id) in ids.iter().enumerate() {
                    self.line(format!(
                        "servo_read id={} position_rad={} velocity_rad_s={}",
                        id,
                        sample.positions.get(index).copied().unwrap_or_default(),
                        sample.velocities.get(index).copied().unwrap_or_default()
                    ));
                }
            }
            Err(error) => self.line(format!("servo_read_error ids={ids:?} error={error}")),
        }
        result
    }

    fn read_imu(&mut self) -> Result<Option<ServoImuSample>, ServoError> {
        let result = self.inner.read_imu();
        match &result {
            Ok(Some(sample)) => self.line(format!(
                "imu_read gyro={:?} gravity={:?}",
                sample.gyro, sample.gravity
            )),
            Ok(None) => self.line("imu_read unavailable"),
            Err(error) => self.line(format!("imu_read_error error={error}")),
        }
        result
    }

    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        for (index, id) in ids.iter().enumerate() {
            self.line(format!(
                "servo_write id={} target_rad={}",
                id,
                targets.positions.get(index).copied().unwrap_or_default()
            ));
        }
        let result = self.inner.write_positions(ids, targets);
        if let Err(error) = &result {
            self.line(format!("servo_write_error ids={ids:?} error={error}"));
        }
        result
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.line(format!("servo_torque enabled={enabled}"));
        let result = self.inner.set_torque(enabled);
        if let Err(error) = &result {
            self.line(format!("servo_torque_error error={error}"));
        }
        result
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.line(format!("servo_gain value={gain}"));
        let result = self.inner.set_gain(gain);
        if let Err(error) = &result {
            self.line(format!("servo_gain_error error={error}"));
        }
        result
    }

    fn reboot(&mut self, id: u8) -> Result<(), ServoError> {
        self.line(format!("servo_reboot id={id}"));
        let result = self.inner.reboot(id);
        if let Err(error) = &result {
            self.line(format!("servo_reboot_error id={id} error={error}"));
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use servo::ServoLoopDriver;
    use std::time::SystemTime;

    struct FakeDriver;

    impl ServoLoopDriver for FakeDriver {
        fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
            Ok(ServoPositionSample {
                positions: ids.iter().map(|id| f64::from(*id)).collect(),
                velocities: vec![0.0; ids.len()],
            })
        }

        fn write_positions(
            &mut self,
            _ids: &[u8],
            _targets: &ServoPositionTargets,
        ) -> Result<(), ServoError> {
            Ok(())
        }

        fn set_torque(&mut self, _enabled: bool) -> Result<(), ServoError> {
            Ok(())
        }

        fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
            Ok(())
        }

        fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
            Ok(())
        }
    }

    #[test]
    fn logged_driver_records_each_motor_input_and_output() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("microduck-control-log-{suffix}"));
        let config_dir = root.join("config");
        let handle = begin_control_loop(
            Some(&config_dir),
            "physical-1",
            "duck",
            "left_hip_yaw->1 dir=1 offset=0",
        )
        .unwrap();
        let mut driver = LoggedServoDriver::new(Box::new(FakeDriver), handle, "physical-1");
        driver.read_positions(&[1, 2]).unwrap();
        driver
            .write_positions(
                &[1, 2],
                &ServoPositionTargets {
                    positions: vec![0.1, -0.2],
                },
            )
            .unwrap();
        let text = std::fs::read_to_string(current_log_file(Some(&config_dir))).unwrap();
        assert!(text.contains("servo_read id=1"));
        assert!(text.contains("servo_read id=2"));
        assert!(text.contains("servo_write id=1 target_rad=0.1"));
        assert!(text.contains("servo_write id=2 target_rad=-0.2"));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn policy_trace_log_keeps_backend_and_saturation_diagnostics() {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("microduck-policy-log-{suffix}"));
        let config_dir = root.join("config");
        let trace = crate::backend::policy_runtime::PolicyTrace {
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
        assert!(text.contains("saturation_reason=[\"target_filter\"]"));
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
        let root = std::env::temp_dir().join(format!("microduck-diagnostic-log-{suffix}"));
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
