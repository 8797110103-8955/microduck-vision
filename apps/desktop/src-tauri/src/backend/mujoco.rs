//! Official native MuJoCo 3.10.0 + BAM M6 backend. No independent plant integration.
//!
//! BAM equations follow Rhoban/bam 62bd8ce12154340be97e06f7f41a0ca8f116d967
//! (Apache-2.0), model.py, actuator.py and mujoco.py. Model setup/initialization follows
//! vendor/microduck_rl/scripts/infer_policy.py. q_offset/identified command_delay are testbench
//! fitting parameters: MujocoController does not apply them; actuator delay is queued here once.

use std::collections::{BTreeSet, VecDeque};
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::path::PathBuf;
use std::ptr::NonNull;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_body::{BodyTemplate, OfficialScene, ServoProfile};
use servo::{
    ServoError, ServoImuSample, ServoLoopDriver, ServoPositionSample, ServoPositionTargets,
};

use crate::types::{SceneFrame, SimulationState};

/// Actuator execution settings selected by the simulated servo model.
///
/// The policy contract stays shared. These settings describe how a canonical target is turned
/// into actuator dynamics after the controller has produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SimulationActuatorProfile {
    pub id: &'static str,
    pub delay_steps: [usize; 2],
    pub encoder_through_backlash: bool,
}

impl SimulationActuatorProfile {
    pub(crate) fn for_model(model: servo::ServoModel) -> Result<Self, ServoError> {
        match model {
            servo::ServoModel::Xl330M288 => Ok(Self {
                id: "official-xl330-m6",
                delay_steps: [3, 6],
                encoder_through_backlash: true,
            }),
            servo::ServoModel::Hd1910 => Ok(Self {
                id: "hardware-matched-hd1910",
                // The identified HD1910 command delay is 0.98ms, below one 5ms physics step.
                // The official XL330 training delay is a separate [3, 6]-step contract.
                delay_steps: [0, 0],
                encoder_through_backlash: false,
            }),
            _ => Err(failure("unsupported simulation actuator profile")),
        }
    }
}

unsafe extern "C" {
    fn duck_mj_version() -> c_int;
    fn duck_mj_open(
        path: *const c_char,
        force: f64,
        armature: f64,
        error: *mut c_char,
        size: c_int,
    ) -> *mut c_void;
    fn duck_mj_close(engine: *mut c_void);
    fn duck_mj_joint(
        engine: *mut c_void,
        name: *const c_char,
        actuator: *mut c_int,
        qpos: *mut c_int,
        dof: *mut c_int,
    ) -> c_int;
    fn duck_mj_passive(
        engine: *mut c_void,
        name: *const c_char,
        qpos: *mut c_int,
        dof: *mut c_int,
    ) -> c_int;
    fn duck_mj_reset(engine: *mut c_void) -> c_int;
    fn duck_mj_set_position(engine: *mut c_void, q: c_int, value: f64);
    fn duck_mj_forward(engine: *mut c_void);
    fn duck_mj_render(engine: *mut c_void);
    fn duck_mj_state(engine: *mut c_void, joint: c_int, q: c_int, dof: c_int, values: *mut f64);
    fn duck_mj_qpos(engine: *mut c_void, q: c_int) -> f64;
    fn duck_mj_qvel(engine: *mut c_void, dof: c_int) -> f64;
    fn duck_mj_control(
        engine: *mut c_void,
        a: c_int,
        dof: c_int,
        torque: f64,
        friction: f64,
        damping: f64,
    );
    fn duck_mj_step(engine: *mut c_void) -> c_int;
    fn duck_mj_body(engine: *mut c_void, name: *const c_char) -> c_int;
    fn duck_mj_pose(engine: *mut c_void, body: c_int, pose: *mut f64);
    fn duck_mj_imu(engine: *mut c_void, gyro: *mut f64, gravity: *mut f64) -> c_int;
    fn duck_mj_time(engine: *mut c_void) -> f64;
    fn duck_mj_contact_count(engine: *mut c_void) -> c_int;
    fn duck_mj_contact(engine: *mut c_void, index: c_int, pair: *mut c_int);
    fn duck_mj_base_velocity(engine: *mut c_void, velocity: *mut f64);
}

#[derive(Default)]
pub(crate) struct SharedSnapshot {
    pub state: Option<SimulationState>,
}
pub(crate) type SnapshotHandle = Arc<Mutex<SharedSnapshot>>;

#[derive(Clone, Copy)]
struct Joint {
    joint: i32,
    actuator: i32,
    qpos: i32,
    dof: i32,
    passive: Option<(i32, i32)>,
}

pub(super) struct NativeSimulation {
    engine: NonNull<c_void>,
    joints: Vec<Option<Joint>>,
    bodies: Vec<i32>,
    trunk: i32,
    home: Vec<f64>,
    profile: ServoProfile,
    targets: Vec<f64>,
    pending: VecDeque<(u64, Vec<f64>)>,
    delay: [usize; 2],
    sequence: u64,
    step: u64,
    previous_torque: Vec<f64>,
    friction: Vec<f64>,
    effective_voltage: f64,
    torque: bool,
    encoder_through_backlash: bool,
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

pub(crate) fn resource_directory() -> Result<PathBuf, ServoError> {
    let executable = std::env::current_exe().map_err(|e| failure(e.to_string()))?;
    let parent = executable
        .parent()
        .ok_or_else(|| failure("missing executable directory"))?;
    let deployed = parent.join("resources/microduck");
    if deployed.is_dir() {
        return Ok(deployed);
    }
    // cargo test runs in deps, while the app runs from the profile root.
    let deployed = parent
        .parent()
        .unwrap_or(parent)
        .join("resources/microduck");
    if deployed.is_dir() {
        return Ok(deployed);
    }
    Err(failure(
        "official MuJoCo model assets are missing beside the application",
    ))
}

impl NativeSimulation {
    pub(super) fn open(
        template: &BodyTemplate,
        profile: ServoProfile,
        actuator_profile: SimulationActuatorProfile,
        snapshot: SnapshotHandle,
    ) -> Result<Self, ServoError> {
        let scene_name = match profile.model {
            servo::ServoModel::Xl330M288 => "scene_backlash.xml",
            servo::ServoModel::Hd1910 => "scene.xml",
            _ => return Err(failure("unsupported official native simulation profile")),
        };
        let scene = resource_directory()?.join(scene_name);
        let path = cstring(&scene.to_string_lossy())?;
        let mut error = [0 as c_char; 2048];
        // SAFETY: buffers live throughout the call, the binding allocates an owned model/data.
        let engine = unsafe {
            duck_mj_open(
                path.as_ptr(),
                profile.bam_vin * profile.bam_kt / profile.bam_resistance,
                profile.bam_armature,
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        let engine = NonNull::new(engine).ok_or_else(|| {
            failure(
                unsafe { CStr::from_ptr(error.as_ptr()) }
                    .to_string_lossy()
                    .into_owned(),
            )
        })?;
        let mut simulation = Self {
            engine,
            joints: Vec::new(),
            bodies: Vec::new(),
            trunk: -1,
            home: template.control.home_pose.clone(),
            targets: template.control.home_pose.clone(),
            profile,
            pending: VecDeque::new(),
            delay: actuator_profile.delay_steps,
            sequence: 0,
            step: 0,
            previous_torque: vec![0.0; template.joints.len()],
            friction: vec![0.0; template.joints.len()],
            effective_voltage: 0.0,
            torque: false,
            encoder_through_backlash: actuator_profile.encoder_through_backlash,
            snapshot,
        };
        simulation.resolve(template)?;
        simulation.reset()?;
        Ok(simulation)
    }

    fn ptr(&self) -> *mut c_void {
        self.engine.as_ptr()
    }

    fn resolve(&mut self, template: &BodyTemplate) -> Result<(), ServoError> {
        let mut seen = BTreeSet::new();
        for slot in &template.joints {
            let Some(name) = slot.scene_joint.as_deref() else {
                self.joints.push(None);
                continue;
            };
            let cname = cstring(name)?;
            let (mut actuator, mut qpos, mut dof) = (-1, -1, -1);
            let joint = unsafe {
                duck_mj_joint(
                    self.ptr(),
                    cname.as_ptr(),
                    &mut actuator,
                    &mut qpos,
                    &mut dof,
                )
            };
            if joint < 0 || !seen.insert(joint) {
                return Err(failure(format!("missing/duplicate official joint {name}")));
            }
            let pname = cstring(&format!("passive_{name}_backlash"))?;
            let (mut pq, mut pd) = (-1, -1);
            let passive_found =
                unsafe { duck_mj_passive(self.ptr(), pname.as_ptr(), &mut pq, &mut pd) };
            if passive_found < 0
                || (self.profile.model == servo::ServoModel::Xl330M288 && passive_found != 1)
            {
                return Err(failure(format!(
                    "missing official backlash hinge for {name}"
                )));
            }
            self.joints.push(Some(Joint {
                joint,
                actuator,
                qpos,
                dof,
                passive: (passive_found == 1).then_some((pq, pd)),
            }));
        }
        if seen.len() != 14 {
            return Err(failure("official scene requires exactly 14 driven joints"));
        }
        let display = OfficialScene::from_path(&resource_directory()?.join("robot_walk.xml"))
            .map_err(|e| failure(e.to_string()))?;
        for body in &display.model().bodies {
            let name = cstring(&body.name)?;
            let id = unsafe { duck_mj_body(self.ptr(), name.as_ptr()) };
            if id < 0 {
                return Err(failure(format!(
                    "missing official display body {}",
                    body.name
                )));
            }
            self.bodies.push(id);
        }
        self.trunk = unsafe { duck_mj_body(self.ptr(), cstring("trunk_base")?.as_ptr()) };
        Ok(())
    }

    fn reset(&mut self) -> Result<(), ServoError> {
        if unsafe { duck_mj_reset(self.ptr()) } != 1 {
            return Err(failure("official scene has no trunk freejoint"));
        }
        for (slot, joint) in self.joints.iter().enumerate() {
            if let Some(j) = joint {
                unsafe {
                    duck_mj_set_position(self.ptr(), j.qpos, self.home[slot]);
                }
            }
        }
        unsafe {
            duck_mj_forward(self.ptr());
        }
        self.targets.clone_from(&self.home);
        self.previous_torque.fill(0.0);
        self.friction.fill(0.0);
        self.effective_voltage = self.profile.bam_vin;
        self.pending.clear();
        self.sequence = 0;
        self.step = 0;
        self.publish()
    }

    fn step_physics(&mut self) -> Result<(), ServoError> {
        let mut due = None;
        while self.pending.front().is_some_and(|(at, _)| *at <= self.step) {
            due = self.pending.pop_front().map(|(_, t)| t);
        }
        if let Some(targets) = due {
            self.targets = targets;
        }
        let p = &self.profile;
        let vin = (p.bam_vin - 0.1 * self.previous_torque.iter().map(|v| v.abs()).sum::<f64>())
            .max(p.vin_min);
        self.effective_voltage = vin;
        for (slot, joint) in self.joints.iter().enumerate() {
            let Some(j) = joint else {
                continue;
            };
            let mut v = [0.0; 4];
            unsafe {
                duck_mj_state(self.ptr(), j.joint, j.qpos, j.dof, v.as_mut_ptr());
            }
            let [q, dq, motor, external] = v;
            let output_q = q + j
                .passive
                .filter(|_| self.encoder_through_backlash)
                .map_or(0.0, |(pq, _)| unsafe { duck_mj_qpos(self.ptr(), pq) });
            let duty = ((self.targets[slot] - output_q) * p.bam_kp * p.bam_error_gain)
                .clamp(-p.bam_max_pwm, p.bam_max_pwm);
            let torque = (vin * duty - p.bam_kt * dq) * p.bam_kt / p.bam_resistance;
            self.previous_torque[slot] = torque;
            let friction = friction_budget(p, motor, external, dq);
            self.friction[slot] = friction;
            unsafe {
                duck_mj_control(
                    self.ptr(),
                    j.actuator,
                    j.dof,
                    if self.torque { torque } else { 0.0 },
                    friction,
                    p.bam_friction_viscous,
                );
            }
        }
        if unsafe { duck_mj_step(self.ptr()) } == 0 {
            return Err(failure("MuJoCo solver warning: execution stopped"));
        }
        self.step += 1;
        Ok(())
    }

    fn sample(&self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        let mut positions = Vec::with_capacity(ids.len());
        let mut velocities = Vec::with_capacity(ids.len());
        for id in ids {
            let slot = usize::from(*id)
                .checked_sub(1)
                .filter(|i| *i < self.joints.len())
                .ok_or_else(|| failure("invalid canonical simulation ID"))?;
            let (q, dq) = match self.joints[slot] {
                Some(j) => unsafe {
                    (
                        duck_mj_qpos(self.ptr(), j.qpos)
                            + j.passive
                                .filter(|_| self.encoder_through_backlash)
                                .map_or(0.0, |(q, _)| duck_mj_qpos(self.ptr(), q)),
                        duck_mj_qvel(self.ptr(), j.dof)
                            + j.passive
                                .filter(|_| self.encoder_through_backlash)
                                .map_or(0.0, |(_, d)| duck_mj_qvel(self.ptr(), d)),
                    )
                },
                None => (0.0, 0.0), // mouth is an explicitly unactuated display slot.
            };
            positions.push(q);
            velocities.push(dq);
        }
        Ok(ServoPositionSample {
            positions,
            velocities,
        })
    }

    fn publish(&self) -> Result<(), ServoError> {
        // Recompute post-step render transforms without perturbing controller feedback or the
        // official mj_step sensor evaluation phase.
        unsafe {
            duck_mj_render(self.ptr());
        }
        let pose = |body| {
            let mut value = [0.0; 7];
            unsafe {
                duck_mj_pose(self.ptr(), body, value.as_mut_ptr());
            }
            value
        };
        let trunk = pose(self.trunk);
        let bodies = self.bodies.iter().map(|body| pose(*body)).collect();
        let motor_positions = self
            .joints
            .iter()
            .map(|j| j.map_or(0.0, |j| unsafe { duck_mj_qpos(self.ptr(), j.qpos) }))
            .collect();
        let passive_positions = self
            .joints
            .iter()
            .map(|j| {
                j.and_then(|j| j.passive)
                    .map_or(0.0, |(q, _)| unsafe { duck_mj_qpos(self.ptr(), q) })
            })
            .collect();
        let contacts = (0..unsafe { duck_mj_contact_count(self.ptr()) })
            .map(|i| {
                let mut pair = [0; 2];
                unsafe {
                    duck_mj_contact(self.ptr(), i, pair.as_mut_ptr());
                }
                pair
            })
            .collect();
        let mut base_velocity = [0.0; 6];
        unsafe {
            duck_mj_base_velocity(self.ptr(), base_velocity.as_mut_ptr());
        }
        let state = SimulationState {
            physics_step: self.step,
            time: unsafe { duck_mj_time(self.ptr()) },
            trunk_position: [trunk[0], trunk[1], trunk[2]],
            trunk_quaternion: [trunk[3], trunk[4], trunk[5], trunk[6]],
            frame: SceneFrame {
                bodies,
                markers: Vec::new(),
            },
            motor_positions,
            passive_positions,
            base_velocity,
            contacts,
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

fn friction_budget(p: &ServoProfile, motor: f64, external: f64, dq: f64) -> f64 {
    let stribeck = (-(dq.abs() / p.bam_dtheta_stribeck).powf(p.bam_alpha)).exp();
    let base = (external * p.bam_load_friction_external - motor * p.bam_load_friction_motor).abs();
    let low_speed = (external * p.bam_load_friction_external_stribeck
        - motor * p.bam_load_friction_motor_stribeck)
        .abs();
    // BAM's equality case intentionally selects neither quadratic term.
    let quadratic = if external.signum() != motor.signum() {
        if motor.abs() > external.abs() {
            p.bam_load_friction_external_quad * external.powi(2)
        } else if external.abs() > motor.abs() {
            p.bam_load_friction_motor_quad * motor.powi(2)
        } else {
            0.0
        }
    } else {
        0.0
    };
    p.bam_friction_base + base + stribeck * (p.bam_friction_stribeck + low_speed + quadratic)
}

impl ServoLoopDriver for NativeSimulation {
    fn feedback_validity(&self, ids: &[u8]) -> Vec<bool> {
        ids.iter()
            .map(|id| {
                id.checked_sub(1)
                    .and_then(|i| self.joints.get(usize::from(i)))
                    .is_some_and(Option::is_some)
            })
            .collect()
    }
    fn read_positions(&mut self, ids: &[u8]) -> Result<ServoPositionSample, ServoError> {
        self.sample(ids)
    }
    fn read_imu(&mut self) -> Result<Option<ServoImuSample>, ServoError> {
        let mut sample = ServoImuSample {
            gyro: [0.0; 3],
            gravity: [0.0; 3],
        };
        if unsafe {
            duck_mj_imu(
                self.ptr(),
                sample.gyro.as_mut_ptr(),
                sample.gravity.as_mut_ptr(),
            )
        } != 1
        {
            return Err(failure("official IMU mapping missing"));
        }
        Ok(Some(sample))
    }
    fn write_positions(
        &mut self,
        ids: &[u8],
        targets: &ServoPositionTargets,
    ) -> Result<(), ServoError> {
        if ids.len() != targets.positions.len() || targets.positions.iter().any(|v| !v.is_finite())
        {
            return Err(failure("invalid simulation targets"));
        }
        let mut next = self.targets.clone();
        for (id, target) in ids.iter().zip(&targets.positions) {
            let slot = usize::from(*id)
                .checked_sub(1)
                .filter(|i| *i < next.len())
                .ok_or_else(|| failure("invalid canonical simulation target ID"))?;
            next[slot] = *target;
        }
        let span = self.delay[1] - self.delay[0] + 1;
        let delay = self.delay[0] + self.sequence as usize % span;
        self.sequence += 1;
        // Absolute deadlines; a later command may become due before an older one. In that case
        // the older command is superseded, so it can never overwrite a newer applied target.
        let at = self.step + delay as u64;
        while self
            .pending
            .back()
            .is_some_and(|(deadline, _)| *deadline >= at)
        {
            self.pending.pop_back();
        }
        self.pending.push_back((at, next));
        Ok(())
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ServoError> {
        self.torque = enabled;
        if !enabled {
            self.pending.clear();
            self.previous_torque.fill(0.0);
        }
        Ok(())
    }
    fn set_gain(&mut self, _gain: u16) -> Result<(), ServoError> {
        Ok(())
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
            self.step_physics()?;
        }
        self.publish()
    }

    fn reset_execution(&mut self) -> Result<(), ServoError> {
        self.reset()
    }
}

impl Drop for NativeSimulation {
    fn drop(&mut self) {
        unsafe {
            duck_mj_close(self.ptr());
        }
    }
}

pub(super) fn runtime_version() -> i32 {
    unsafe { duck_mj_version() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::collections::BTreeMap;

    #[derive(Deserialize)]
    struct Fixture {
        asset_sha256: BTreeMap<String, String>,
        tolerances: BTreeMap<String, f64>,
        cases: Vec<Case>,
    }
    #[derive(Deserialize)]
    struct Case {
        delay: [usize; 2],
        encoder_through_backlash: bool,
        frames: Vec<serde_json::Value>,
    }

    #[test]
    fn servo_models_use_separate_simulation_actuator_profiles() {
        let xl = SimulationActuatorProfile::for_model(servo::ServoModel::Xl330M288).unwrap();
        let hd = SimulationActuatorProfile::for_model(servo::ServoModel::Hd1910).unwrap();
        assert_eq!(xl.id, "official-xl330-m6");
        assert_eq!(xl.delay_steps, [3, 6]);
        assert!(xl.encoder_through_backlash);
        assert_eq!(hd.id, "hardware-matched-hd1910");
        assert_eq!(hd.delay_steps, [0, 0]);
        assert!(!hd.encoder_through_backlash);
    }

    #[test]
    fn native_mujoco_matches_pinned_vendor_cpu_for_300_ticks_and_all_delay_schedules() {
        let fixture: Fixture = serde_json::from_str(include_str!(
            "../../tests/fixtures/native-mujoco-xl330.json"
        ))
        .unwrap();
        use sha2::{Digest, Sha256};
        let root = resource_directory().unwrap();
        for (path, expected) in &fixture.asset_sha256 {
            assert_eq!(
                format!(
                    "{:x}",
                    Sha256::digest(std::fs::read(root.join(path)).unwrap())
                ),
                *expected,
                "reference asset changed: {path}"
            );
        }
        let body = BodyTemplate::official().unwrap();
        let profile = duck_body::servo_profiles()
            .into_iter()
            .find(|p| p.model == servo::ServoModel::Xl330M288)
            .unwrap();
        let ids = (1..=15).collect::<Vec<_>>();
        let mut reports = Vec::new();
        let mut first_failure = None;
        for case in fixture.cases {
            let snapshot = SnapshotHandle::default();
            let mut simulation = NativeSimulation::open(
                &body,
                profile.clone(),
                SimulationActuatorProfile {
                    id: "test-fixture",
                    delay_steps: case.delay,
                    encoder_through_backlash: case.encoder_through_backlash,
                },
                snapshot.clone(),
            )
            .unwrap();
            simulation.set_torque(true).unwrap();
            let mut maxima = BTreeMap::<String, f64>::new();
            for (tick, expected) in case.frames.iter().enumerate() {
                let target: Vec<f64> = serde_json::from_value(expected["target"].clone()).unwrap();
                simulation
                    .write_positions(&ids, &ServoPositionTargets { positions: target })
                    .unwrap();
                simulation
                    .advance_execution(4, Duration::from_millis(5))
                    .unwrap();
                assert!(
                    simulation.pending.len() <= 2,
                    "delay queue must remain bounded"
                );
                let sample = simulation.read_positions(&ids).unwrap();
                let imu = simulation.read_imu().unwrap().unwrap();
                let state = snapshot.lock().unwrap().state.clone().unwrap();
                assert_eq!(state.physics_step, (tick as u64 + 1) * 4);
                assert_eq!(
                    serde_json::to_value(&state.contacts).unwrap(),
                    expected["contacts"],
                    "contact pairs at tick {tick}"
                );
                let motor_torque = simulation
                    .previous_torque
                    .iter()
                    .enumerate()
                    .filter_map(|(i, v)| simulation.joints[i].is_some().then_some(*v))
                    .collect::<Vec<_>>();
                let actual = serde_json::json!({"positions":sample.positions,"velocities":sample.velocities,
                    "motor_positions":state.motor_positions,"passive_positions":state.passive_positions,
                    "trunk_position":state.trunk_position,"trunk_quaternion":state.trunk_quaternion,
                    "gyro":imu.gyro,"gravity":imu.gravity,"torque":motor_torque,"base_velocity":state.base_velocity,"time":state.time});
                for (field, tolerance) in &fixture.tolerances {
                    let error = if field == "time" {
                        (actual[field].as_f64().unwrap() - expected[field].as_f64().unwrap()).abs()
                    } else {
                        let a = actual[field].as_array().unwrap();
                        let e = expected[field].as_array().unwrap();
                        assert_eq!(a.len(), e.len());
                        let direct = a
                            .iter()
                            .zip(e)
                            .map(|(a, e)| (a.as_f64().unwrap() - e.as_f64().unwrap()).abs())
                            .fold(0.0, f64::max);
                        if field == "trunk_quaternion" {
                            direct.min(
                                a.iter()
                                    .zip(e)
                                    .map(|(a, e)| (a.as_f64().unwrap() + e.as_f64().unwrap()).abs())
                                    .fold(0.0, f64::max),
                            )
                        } else {
                            direct
                        }
                    };
                    let max = maxima.entry(field.clone()).or_default();
                    *max = max.max(error);
                    if error > *tolerance && first_failure.is_none() {
                        first_failure = Some(format!(
                            "delay={:?} tick={tick} field={field} error={error} tolerance={tolerance}",
                            case.delay
                        ));
                    }
                }
            }
            reports.push(serde_json::json!({"delay":case.delay,"encoder_through_backlash":case.encoder_through_backlash,"ticks":case.frames.len(),"max_abs_error":maxima}));
            // Reset must clear solver/controller/delay state and reproduce the initial state.
            simulation.reset().unwrap();
            assert_eq!(simulation.step, 0);
            assert!(simulation.pending.is_empty());
            assert_eq!(
                simulation.read_positions(&ids).unwrap().positions,
                body.control.home_pose
            );
        }
        let report = serde_json::json!({"source":"fixed vendor CPU reference; not an upstream published golden","cases":reports,"first_failure":first_failure});
        let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../.agentic");
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(
            directory.join("native-mujoco-comparison.json"),
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
        assert!(first_failure.is_none(), "{report}");
    }

    #[test]
    fn native_physics_advances_while_unpowered_and_status_reads_do_not_advance() {
        let body = BodyTemplate::official().unwrap();
        let profile = duck_body::servo_profiles()
            .into_iter()
            .find(|p| p.model == servo::ServoModel::Xl330M288)
            .unwrap();
        let mut simulation =
            NativeSimulation::open(
                &body,
                profile,
                SimulationActuatorProfile {
                    id: "test-fixture",
                    delay_steps: [3, 6],
                    encoder_through_backlash: true,
                },
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
