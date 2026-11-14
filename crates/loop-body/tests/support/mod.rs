use duck_body::BodyTemplate;
use loop_body::{
    BodyLoop, BodyRuntimeView, Calibration, JointCalibration, LoopBinding, LoopConfig, ServoBinding,
};
use servo::{ServoError, ServoLoopDriver, ServoModel, ServoPositionSample, ServoPositionTargets};

#[derive(Default)]
pub struct FakeDriver {
    pub positions: Vec<f64>,
    pub velocities: Vec<f64>,
    pub writes: Vec<Vec<f64>>,
    pub torque: Option<bool>,
    pub gains: Vec<u16>,
    pub fail_reads: usize,
}

impl FakeDriver {
    pub fn new(count: usize) -> Self {
        Self {
            positions: vec![0.0; count],
            velocities: vec![0.0; count],
            ..Self::default()
        }
    }
}

impl ServoLoopDriver for FakeDriver {
    fn read_positions(&mut self, _ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
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
        self.positions = targets.positions.clone();
        self.writes.push(targets.positions.clone());
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.torque = Some(enabled);
        Ok(())
    }

    fn set_gain(&mut self, gain: u16) -> Result<(), ServoError> {
        self.gains.push(gain);
        Ok(())
    }

    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        Ok(())
    }
}

pub fn binding(model: ServoModel, count: usize) -> LoopBinding {
    let body = BodyTemplate::official().unwrap();
    let ids: Vec<_> = (1..=count as u8).collect();
    let slots: Vec<_> = body.actuators.iter().map(|slot| slot.id.clone()).collect();
    LoopBinding {
        body: BodyRuntimeView::new(body).unwrap(),
        servos: ServoBinding {
            model,
            slot_ids: slots.clone(),
        },
        calibration: Calibration {
            joints: slots
                .into_iter()
                .zip(ids)
                .map(|(slot_id, physical_id)| JointCalibration {
                    slot_id,
                    physical_id,
                    direction: 1,
                    zero_offset_rad: 0.0,
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
