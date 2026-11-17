use duck_body::BodyTemplate;
use loop_body::{
    BodyLoop, BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ServoBinding,
};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};
use std::sync::{Arc, Mutex};

#[derive(Default)]
pub struct FakeDriver {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub writes: Vec<Vec<f64>>,
    pub torque: Option<bool>,
    pub gains: Vec<u16>,
    pub fail_reads: usize,
    pub events: Arc<Mutex<Vec<&'static str>>>,
}

impl FakeDriver {
    pub fn new(count: usize) -> Self {
        Self {
            positions: vec![0.0; count],
            velocities: vec![0.0; count],
            events: Arc::new(Mutex::new(Vec::new())),
            ..Self::default()
        }
    }

    #[allow(dead_code)]
    pub fn event_log(&self) -> Arc<Mutex<Vec<&'static str>>> {
        Arc::clone(&self.events)
    }
}

impl ServoLoopDriver for FakeDriver {
    fn read_positions(&mut self, _ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        self.events.lock().unwrap().push("read");
        if self.fail_reads > 0 {
            self.fail_reads -= 1;
            return Err(ServoError::Bus("fake disconnect".into()));
        }
        Ok(ServoPositionSample {
            positions: self.positions.clone(),
            velocities: self.velocities.clone(),
        })
    }

    fn write_positions(
        &mut self,
        _ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        self.events.lock().unwrap().push("write");
        self.positions = targets.positions.clone();
        self.writes.push(targets.positions.clone());
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.events.lock().unwrap().push("torque");
        self.torque = Some(enabled);
        Ok(())
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.events.lock().unwrap().push("gain");
        self.gains.push(gain);
        Ok(())
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        Ok(())
    }
}

pub fn binding(model: ServoModel, count: usize) -> LoopBinding {
    let body = BodyTemplate::official().unwrap();
    let body_id = body.id.clone();
    let body_version = body.version;
    let ids: Vec<_> = (1..=count as u8).collect();
    let slots: Vec<_> = body.actuators.iter().map(|slot| slot.id.clone()).collect();
    LoopBinding {
        body: BodyRuntimeView::new(body).unwrap(),
        servos: ServoBinding {
            model,
            slot_ids: slots.clone(),
        },
        calibration: Calibration {
            body_id,
            body_version,
            joints: slots
                .into_iter()
                .zip(ids)
                .map(|(slot_id, physical_id)| JointCalibration {
                    slot_id,
                    physical_id,
                    direction: 1,
                    zero_offset_rad: 0.0,
                    min_rad: None,
                    max_rad: None,
                })
                .collect(),
        },
    }
}

#[allow(dead_code)]
pub fn loop_with(driver: FakeDriver) -> BodyLoop {
    BodyLoop::open(
        binding(ServoModel::Hd1910, 15),
        Box::new(driver),
        LoopConfig {
            homing_ticks: 2,
            sensor_failure_limit: 2,
            ..LoopConfig::default()
        },
    )
    .unwrap()
}
