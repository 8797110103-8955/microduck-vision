//! Official native MuJoCo 3.10.0 + BAM M6 backend. No independent plant integration.
//!
//! BAM equations follow Rhoban/bam 62bd8ce12154340be97e06f7f41a0ca8f116d967
//! (Apache-2.0), model.py, actuator.py and mujoco.py. Model setup/initialization follows
//! vendor/microduck_rl/scripts/infer_policy.py. q_offset/identified command_delay are testbench
//! fitting parameters: MujocoController does not apply them. Command history follows mjlab
//! 1.3.0 Actuator.apply_delay/DelayBuffer, updated once per physics step.

use std::collections::{BTreeSet, VecDeque};
use std::ffi::CString;
use std::time::Duration;

use duck_body::{AssetResolver, BodyTemplate, OfficialScene, SceneFrame};
use loop_body::{BackendImuSample, BackendJointSample, BackendPositionTargets, ControlBackend};
use servo::ServoError;

use super::bam::friction_budget;
use super::snapshot::{SimulationSnapshot, SnapshotHandle};
use crate::models::profile::{ActuatorResponse, BamParameters, BodyExecutionProfile};
use engine::{DrivenJoint, JointAddress};

mod engine;

pub use engine::{CompiledScene, runtime_version};

#[derive(Clone, Copy)]
struct Joint {
    driven: DrivenJoint,
    passive: Option<JointAddress>,
}

pub struct NativeSimulation {
    engine: CompiledScene,
    joints: Vec<Option<Joint>>,
    bodies: Vec<usize>,
    trunk: usize,
    home: Vec<f64>,
    profile: BamParameters,
    targets: Vec<f64>,
    requested_targets: Vec<f64>,
    target_history: VecDeque<Vec<f64>>,
    delay_rng: u64,
    response: ActuatorResponse,
    delay_seed: u64,
    voltage_drop_gain: f64,
    minimum_voltage_v: f64,
    step: u64,
    previous_torque: Vec<f64>,
    friction: Vec<f64>,
    effective_voltage: f64,
    torque: bool,
    snapshot: SnapshotHandle,
}

// A model/data pair has exclusive ownership; all its calls are serialized by RuntimeInstance.
// No MuJoCo global callbacks or renderer are installed. Moving ownership between threads is safe.
unsafe impl Send for NativeSimulation {}

fn failure(message: impl Into<String>) -> ServoError {
    ServoError::Bus(message.into())
}
fn cstring(value: &str) -> Result<CString, ServoError> {
    CString::new(value).map_err(|_| failure("MuJoCo name/path contains NUL"))
}

impl NativeSimulation {
    pub fn open(
        template: &BodyTemplate,
        execution: &BodyExecutionProfile,
        source: &dyn AssetResolver,
        snapshot: SnapshotHandle,
    ) -> Result<Self, ServoError> {
        let profile = execution
            .simulation
            .as_ref()
            .ok_or_else(|| failure("missing simulation profile"))?;
        if execution.periods.physics_s != Some(0.005)
            || execution.periods.physics_substeps != Some(4)
        {
            return Err(failure(
                "official simulation requires 5ms x 4 physics steps",
            ));
        }
        if execution.joint_order != template.control.joint_order
            || execution.home_pose_rad != template.control.home_pose
        {
            return Err(failure(
                "simulation profile does not match its body template",
            ));
        }
        let engine = CompiledScene::compile(profile, source).map_err(failure)?;
        let mut simulation = Self {
            engine,
            joints: Vec::new(),
            bodies: Vec::new(),
            trunk: 0,
            home: template.control.home_pose.clone(),
            targets: template.control.home_pose.clone(),
            requested_targets: template.control.home_pose.clone(),
            profile: profile.motor,
            target_history: VecDeque::new(),
            delay_rng: profile.delay_seed,
            response: profile.response,
            delay_seed: profile.delay_seed,
            voltage_drop_gain: profile.voltage_drop_gain,
            minimum_voltage_v: profile.minimum_voltage_v,
            step: 0,
            previous_torque: vec![0.0; template.joints.len()],
            friction: vec![0.0; template.joints.len()],
            effective_voltage: 0.0,
            torque: false,
            snapshot,
        };
        simulation.resolve(template, source)?;
        simulation.reset()?;
        Ok(simulation)
    }

    fn resolve(
        &mut self,
        template: &BodyTemplate,
        source: &dyn AssetResolver,
    ) -> Result<(), ServoError> {
        let mut seen = BTreeSet::new();
        for slot in &template.joints {
            let Some(name) = slot.scene_joint.as_deref() else {
                self.joints.push(None);
                continue;
            };
            let driven = self
                .engine
                .driven_joint(&cstring(name)?)
                .filter(|joint| seen.insert(joint.id))
                .ok_or_else(|| failure(format!("missing/duplicate official joint {name}")))?;
            let pname = cstring(&format!("passive_{name}_backlash"))?;
            let missing_backlash =
                || failure(format!("missing official backlash hinge for {name}"));
            let passive = self
                .engine
                .passive_joint(&pname)
                .map_err(|()| missing_backlash())?;
            if self.response.encoder_through_backlash && passive.is_none() {
                return Err(missing_backlash());
            }
            self.joints.push(Some(Joint { driven, passive }));
        }
        if seen.len() != 14 {
            return Err(failure("official scene requires exactly 14 driven joints"));
        }
        let display = OfficialScene::for_official_body(template, source)
            .map_err(|error| failure(error.to_string()))?;
        for body in &display.model().bodies {
            let name = cstring(&body.name)?;
            let id = self
                .engine
                .body_id(&name)
                .ok_or_else(|| failure(format!("missing official display body {}", body.name)))?;
            self.bodies.push(id);
        }
        self.trunk = self
            .engine
            .body_id(c"trunk_base")
            .ok_or_else(|| failure("missing official display body trunk_base"))?;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), ServoError> {
        if !self.engine.reset() {
            return Err(failure("official scene has no trunk freejoint"));
        }
        for (slot, joint) in self.joints.iter().enumerate() {
            if let Some(j) = joint {
                self.engine
                    .set_position(j.driven.address.qpos, self.home[slot]);
            }
        }
        self.engine.forward();
        self.targets.clone_from(&self.home);
        self.requested_targets.clone_from(&self.home);
        self.previous_torque.fill(0.0);
        self.friction.fill(0.0);
        self.effective_voltage = self.profile.vin;
        self.target_history.clear();
        self.delay_rng = self.delay_seed;
        self.step = 0;
        self.publish()
    }

    fn step_physics(&mut self, response: ActuatorResponse) -> Result<(), ServoError> {
        let [earliest, latest] = response.delay_steps;
        self.target_history
            .push_back(self.requested_targets.clone());
        while self.target_history.len() > latest + 1 {
            self.target_history.pop_front();
        }
        // Training appends the held policy target at every physics step, then reads T-lag.
        // A fresh buffer repeats its first sample instead of holding home for an extra lag.
        // A seeded xorshift stream makes runs reproducible without synchronizing lag to
        // decimation (cycling 3/4/5/6 with four substeps aliases to a fixed control-frame lag).
        self.delay_rng ^= self.delay_rng << 13;
        self.delay_rng ^= self.delay_rng >> 7;
        self.delay_rng ^= self.delay_rng << 17;
        let lag = (earliest + (self.delay_rng % (latest - earliest + 1) as u64) as usize)
            .min(self.target_history.len() - 1);
        self.targets
            .clone_from(&self.target_history[self.target_history.len() - 1 - lag]);
        let p = &self.profile;
        let vin = (p.vin
            - self.voltage_drop_gain * self.previous_torque.iter().map(|v| v.abs()).sum::<f64>())
        .max(self.minimum_voltage_v);
        self.effective_voltage = vin;
        for (slot, joint) in self.joints.iter().enumerate() {
            let Some(j) = joint else {
                continue;
            };
            let [q, dq, motor, external] = self.engine.joint_state(j.driven);
            let output_q = q + j
                .passive
                .filter(|_| self.response.encoder_through_backlash)
                .map_or(0.0, |passive| self.engine.qpos(passive.qpos));
            let duty = ((self.targets[slot] - output_q)
                * p.kp
                * p.error_gain
                * response.position_gain_scale)
                .clamp(-p.max_pwm, p.max_pwm);
            let velocity_drop = p.kt * dq * response.back_emf_scale;
            let torque = (vin * duty - velocity_drop) * p.kt / p.resistance;
            self.previous_torque[slot] = torque;
            let friction = friction_budget(p, motor, external, dq) * response.friction_scale;
            self.friction[slot] = friction;
            self.engine.control(
                j.driven,
                if self.torque { torque } else { 0.0 },
                friction,
                p.friction_viscous,
            );
        }
        if !self.engine.step() {
            return Err(failure("MuJoCo solver warning: execution stopped"));
        }
        self.step += 1;
        Ok(())
    }

    fn sample(&self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        let mut positions = Vec::with_capacity(ids.len());
        let mut velocities = Vec::with_capacity(ids.len());
        for id in ids {
            let slot = usize::from(*id)
                .checked_sub(1)
                .filter(|i| *i < self.joints.len())
                .ok_or_else(|| failure("invalid canonical simulation ID"))?;
            let (q, dq) = match self.joints[slot] {
                Some(j) => {
                    let passive = j.passive.filter(|_| self.response.encoder_through_backlash);
                    (
                        self.engine.qpos(j.driven.address.qpos)
                            + passive.map_or(0.0, |p| self.engine.qpos(p.qpos)),
                        self.engine.qvel(j.driven.address.dof)
                            + passive.map_or(0.0, |p| self.engine.qvel(p.dof)),
                    )
                }
                None => (0.0, 0.0), // mouth is an explicitly unactuated display slot.
            };
            positions.push(q);
            velocities.push(dq);
        }
        Ok(BackendJointSample {
            positions,
            velocities,
        })
    }

    fn publish(&mut self) -> Result<(), ServoError> {
        self.engine.render();
        let trunk = self.engine.body_pose(self.trunk);
        let bodies = self
            .bodies
            .iter()
            .map(|body| self.engine.body_pose(*body))
            .collect();
        let motor_positions = self
            .joints
            .iter()
            .map(|j| j.map_or(0.0, |j| self.engine.qpos(j.driven.address.qpos)))
            .collect();
        let passive_positions = self
            .joints
            .iter()
            .map(|j| {
                j.and_then(|j| j.passive)
                    .map_or(0.0, |passive| self.engine.qpos(passive.qpos))
            })
            .collect();
        let state = SimulationSnapshot {
            physics_step: self.step,
            time: self.engine.time(),
            trunk_position: [trunk[0], trunk[1], trunk[2]],
            trunk_quaternion: [trunk[3], trunk[4], trunk[5], trunk[6]],
            frame: SceneFrame {
                bodies,
                markers: Vec::new(),
            },
            motor_positions,
            passive_positions,
            base_velocity: self.engine.base_velocity(),
            contacts: self.engine.contacts(),
            motor_torques: self.previous_torque.clone(),
            friction_budgets: self.friction.clone(),
            effective_voltage: self.effective_voltage,
        };
        self.snapshot
            .lock()
            .map_err(|_| failure("simulation snapshot poisoned"))?
            .state = Some(state);
        Ok(())
    }
}

impl ControlBackend for NativeSimulation {
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        ids.iter()
            .map(|id| {
                id.checked_sub(1)
                    .and_then(|i| self.joints.get(usize::from(i)))
                    .is_some_and(Option::is_some)
            })
            .collect()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<BackendJointSample, ServoError> {
        self.sample(ids)
    }
    fn read_imu(&mut self) -> Result<Option<BackendImuSample>, ServoError> {
        let (gyro, gravity) = self
            .engine
            .imu()
            .ok_or_else(|| failure("official IMU mapping missing"))?;
        Ok(Some(BackendImuSample { gyro, gravity }))
    }
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &BackendPositionTargets,
    ) -> Result<(), ServoError> {
        if ids.len() != targets.positions.len() || targets.positions.iter().any(|v| !v.is_finite())
        {
            return Err(failure("invalid simulation targets"));
        }
        let mut next = self.requested_targets.clone();
        for (id, target) in ids.iter().zip(&targets.positions) {
            let slot = usize::from(*id)
                .checked_sub(1)
                .filter(|i| *i < next.len())
                .ok_or_else(|| failure("invalid canonical simulation target ID"))?;
            next[slot] = *target;
        }
        self.requested_targets = next;
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.torque = enabled;
        if !enabled {
            self.target_history.clear();
            self.previous_torque.fill(0.0);
        }
        Ok(())
    }
    fn gain_capability(&self) -> loop_body::GainCapability {
        loop_body::GainCapability::FixedModel
    }
    fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
        Err(ServoError::UnsupportedGain)
    }
    fn reboot(&mut self, _id: u8) -> Result<(), ServoError> {
        Err(failure("simulation does not support servo reboot"))
    }
    fn advance_execution(&mut self, steps: u32, dt: Duration) -> Result<(), ServoError> {
        if steps != 4 || dt != Duration::from_millis(5) {
            return Err(failure(
                "official simulation requires 5ms x 4 physics steps",
            ));
        }
        for _ in 0..steps {
            self.step_physics(self.response)?;
        }
        self.publish()
    }

    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.reset()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::models::assets::{OfficialAssets, load_official_body};
    use crate::models::profile::{ExecutionKind, ModelKind, Selection};
    use crate::models::resolve::{defaults, resolve_simulation};

    fn profile(kind: ModelKind, body: &duck_body::BodyProfile) -> BodyExecutionProfile {
        resolve_simulation(
            Selection {
                execution_kind: ExecutionKind::Simulation,
                model_kind: kind,
                servo_model: kind.servo_model(),
            },
            body,
            &OfficialAssets::Embedded,
            defaults(kind),
        )
        .unwrap()
    }

    #[test]
    fn servo_models_use_separate_simulation_actuator_profiles() {
        let body = load_official_body(&OfficialAssets::Embedded).unwrap();
        let xl = profile(ModelKind::Xl330, &body).simulation.unwrap();
        let hd = profile(ModelKind::Hd1910Compatible, &body)
            .simulation
            .unwrap();
        assert_eq!(xl.id, "official-xl330-m6");
        assert_eq!(xl.response.delay_steps, [3, 6]);
        assert!(xl.response.encoder_through_backlash);
        assert_eq!(
            xl.response,
            ActuatorResponse {
                delay_steps: [3, 6],
                encoder_through_backlash: true,
                position_gain_scale: 1.0,
                back_emf_scale: 1.0,
                friction_scale: 1.0,
            }
        );
        assert_eq!(hd.id, "body-drive-hd1910");
        assert_eq!(hd.response.delay_steps, [3, 6]);
        assert!(hd.response.encoder_through_backlash);
        assert!(
            (hd.response.position_gain_scale
                - xl.nominal_motor.position_stiffness() / hd.nominal_motor.position_stiffness()
                    * 0.989)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn command_delay_backfills_the_first_target_and_keeps_physics_step_history() {
        let body = load_official_body(&OfficialAssets::Embedded).unwrap();
        let execution = profile(ModelKind::Xl330, &body);
        let response = ActuatorResponse {
            delay_steps: [3, 3],
            ..execution.simulation.as_ref().unwrap().response
        };
        let mut simulation = NativeSimulation::open(
            body.template(),
            &execution,
            &OfficialAssets::Embedded,
            SnapshotHandle::default(),
        )
        .unwrap();
        simulation.set_torque(true).unwrap();
        simulation
            .write_positions(
                &[8],
                &BackendPositionTargets {
                    positions: vec![0.1],
                },
            )
            .unwrap();
        simulation.step_physics(response).unwrap();
        assert_eq!(
            simulation.targets[7], 0.1,
            "first target fills missing history"
        );

        simulation
            .write_positions(
                &[8],
                &BackendPositionTargets {
                    positions: vec![0.2],
                },
            )
            .unwrap();
        simulation
            .write_positions(
                &[9],
                &BackendPositionTargets {
                    positions: vec![0.3],
                },
            )
            .unwrap();
        assert_eq!(
            simulation.requested_targets[7], 0.2,
            "partial writes retain the latest requested frame"
        );
        for _ in 0..3 {
            simulation.step_physics(response).unwrap();
            assert_eq!(simulation.targets[7], 0.1);
            assert_eq!(simulation.targets[8], 0.0);
        }
        simulation.step_physics(response).unwrap();
        assert_eq!(simulation.targets[7], 0.2);
        assert_eq!(simulation.targets[8], 0.3);
        assert_eq!(simulation.target_history.len(), 4);
        simulation.reset().unwrap();
        assert!(simulation.target_history.is_empty());
        assert_eq!(
            simulation.requested_targets,
            body.template().control.home_pose
        );
    }

    #[test]
    fn native_physics_advances_while_unpowered_and_status_reads_do_not_advance() {
        let body = load_official_body(&OfficialAssets::Embedded).unwrap();
        let execution = profile(ModelKind::Xl330, &body);
        let mut simulation = NativeSimulation::open(
            body.template(),
            &execution,
            &OfficialAssets::Embedded,
            SnapshotHandle::default(),
        )
        .unwrap();
        let ids = (1..=15).collect::<Vec<_>>();
        for _ in 0..10 {
            simulation.read_positions(&ids).unwrap();
        }
        assert_eq!(simulation.step, 0);
        for _ in 0..10 {
            simulation
                .advance_execution(4, Duration::from_millis(5))
                .unwrap();
        }
        assert_eq!(simulation.step, 40);
        let state = simulation.snapshot.lock().unwrap().state.clone().unwrap();
        assert_ne!(state.trunk_position[2], 0.125);
    }
}
