//! Small file logger for hardware control-loop I/O.

use std::fs::{File, OpenOptions, create_dir_all};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use loop_body::PolicyTrace;
use servo::{ServoError, ServoLoopDriver, ServoPositionSample, ServoPositionTargets};

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
    let result = (|| -> std::io::Result<()> {
        writeln!(
            writer.file,
            "[{}] loop_open machine_id={} machine_name={:?} binding={ids}",
            timestamp_millis(),
            machine_id,
            machine_name,
            ids = binding_summary
        )
    })();
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
    append_line(
        &current_log_file(config_dir),
        &format!(
            "policy_trace tick={} phase={:?} stages={:?} imu_source={} imu_gyro={:?} imu_gravity={:?} imu_observation={:?} joint_position_observation={:?} joint_velocity_observation={:?} previous_action_observation={:?} command_twist={:?} command_head={:?} command_body=[{}, {}, {}] observation_len={} action_len={} observation={:?} onnx_action={:?} raw_targets={} body_targets={}",
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
            body_targets
        ),
    );
}

pub(crate) fn append_control_event(config_dir: Option<&Path>, message: impl AsRef<str>) {
    append_line(&current_log_file(config_dir), message.as_ref());
}

pub(crate) struct LoggedServoDriver {
    inner: Box<dyn ServoLoopDriver>,
    handle: LogHandle,
    machine_id: String,
}

impl LoggedServoDriver {
    pub(crate) fn new(
        inner: Box<dyn ServoLoopDriver>,
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

impl ServoLoopDriver for LoggedServoDriver {
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
}
