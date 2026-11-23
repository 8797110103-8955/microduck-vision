//! Compile and drive an official model through MuJoCo's native API.

use std::ffi::{CStr, CString, c_int};
use std::mem::MaybeUninit;
use std::ptr::NonNull;

use duck_body::AssetResolver;

use crate::models::profile::ResolvedSimulationProfile;

#[allow(
    dead_code,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_imports
)]
#[path = "ffi.rs"]
mod ffi;

struct Vfs(ffi::mjVFS);

impl Vfs {
    fn new() -> Self {
        let mut vfs = MaybeUninit::uninit();
        // MuJoCo initializes the VFS here and owns its internal allocation.
        unsafe {
            ffi::mj_defaultVFS(vfs.as_mut_ptr());
            Self(vfs.assume_init())
        }
    }

    fn add(&mut self, name: &CStr, bytes: &[u8]) -> Result<(), String> {
        let size = c_int::try_from(bytes.len()).map_err(|error| error.to_string())?;
        if unsafe { ffi::mj_addBufferVFS(&mut self.0, name.as_ptr(), bytes.as_ptr().cast(), size) }
            == 0
        {
            Ok(())
        } else {
            Err(format!(
                "Cannot add MuJoCo resource: {}",
                name.to_string_lossy()
            ))
        }
    }
}

impl Drop for Vfs {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteVFS(&mut self.0) };
    }
}

struct Spec(NonNull<ffi::mjSpec>);

impl Drop for Spec {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteSpec(self.0.as_ptr()) };
    }
}

struct Model(NonNull<ffi::mjModel>);

impl Drop for Model {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteModel(self.0.as_ptr()) };
    }
}

struct Data(NonNull<ffi::mjData>);

impl Data {
    fn new(model: &Model) -> Result<Self, String> {
        NonNull::new(unsafe { ffi::mj_makeData(model.0.as_ptr()) })
            .map(Self)
            .ok_or_else(|| "mj_makeData failed".into())
    }
}

impl Drop for Data {
    fn drop(&mut self) {
        unsafe { ffi::mj_deleteData(self.0.as_ptr()) };
    }
}

#[derive(Clone, Copy)]
pub(crate) struct JointAddress {
    pub(crate) qpos: usize,
    pub(crate) dof: usize,
}

#[derive(Clone, Copy)]
pub(crate) struct DrivenJoint {
    pub(crate) id: c_int,
    actuator: usize,
    pub(crate) address: JointAddress,
}

pub struct CompiledScene {
    // Both state buffers are dropped before their model.
    render: Data,
    data: Data,
    model: Model,
}

impl CompiledScene {
    pub fn compile(
        profile: &ResolvedSimulationProfile,
        source: &dyn AssetResolver,
    ) -> Result<Self, String> {
        let scene_xml = profile
            .scenario
            .scene_xml
            .ok_or("MuJoCo desktop preview requires a complete native scene")?;
        let bundle = profile
            .scenario
            .resources(source)
            .map_err(|error| error.to_string())?;
        if bundle.manifest() != profile.source_assets {
            return Err("the resolved scene assets changed before MuJoCo compilation".into());
        }
        let scene = CString::new(scene_xml).map_err(|error| error.to_string())?;
        let names = bundle
            .files()
            .map(|(name, _)| CString::new(name))
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| error.to_string())?;
        let with_scene = |detail: String| format!("cannot compile {scene_xml}: {detail}");
        let mut vfs = Vfs::new();
        // Keep the asset bytes and C strings alive until the VFS has been released.
        for (name, (_, bytes)) in names.iter().zip(bundle.files()) {
            vfs.add(name, bytes).map_err(with_scene)?;
        }
        let motor = profile.motor;
        let model = compile_model(
            &scene,
            &vfs,
            motor.vin * motor.kt / motor.resistance,
            motor.armature,
        )
        .map_err(with_scene)?;
        let data = Data::new(&model).map_err(with_scene)?;
        let render = Data::new(&model).map_err(with_scene)?;
        Ok(Self {
            render,
            data,
            model,
        })
    }

    fn model(&self) -> &ffi::mjModel {
        unsafe { self.model.0.as_ref() }
    }

    fn data(&self) -> &ffi::mjData {
        unsafe { self.data.0.as_ref() }
    }

    fn name_id(&self, kind: ffi::mjtObj, name: &CStr) -> Option<c_int> {
        let id = unsafe { ffi::mj_name2id(self.model.0.as_ptr(), kind, name.as_ptr()) };
        (id >= 0).then_some(id)
    }

    fn joint_address(&self, id: c_int, kind: ffi::mjtJoint) -> Option<JointAddress> {
        // IDs came from mj_name2id for this model's joint table.
        let model = self.model();
        let id = id as usize;
        unsafe {
            if *model.jnt_type.add(id) != kind as c_int {
                return None;
            }
            Some(JointAddress {
                qpos: *model.jnt_qposadr.add(id) as usize,
                dof: *model.jnt_dofadr.add(id) as usize,
            })
        }
    }

    pub(crate) fn driven_joint(&self, name: &CStr) -> Option<DrivenJoint> {
        let id = self.name_id(ffi::mjOBJ_JOINT, name)?;
        let actuator = self.name_id(ffi::mjOBJ_ACTUATOR, name)? as usize;
        let address = self.joint_address(id, ffi::mjJNT_HINGE)?;
        // Each actuator has two transmission IDs.
        if unsafe { *self.model().actuator_trnid.add(2 * actuator) } != id {
            return None;
        }
        Some(DrivenJoint {
            id,
            actuator,
            address,
        })
    }

    pub(crate) fn passive_joint(&self, name: &CStr) -> Result<Option<JointAddress>, ()> {
        let Some(id) = self.name_id(ffi::mjOBJ_JOINT, name) else {
            return Ok(None);
        };
        self.joint_address(id, ffi::mjJNT_HINGE).map(Some).ok_or(())
    }

    fn trunk_freejoint(&self) -> Option<JointAddress> {
        let id = self.name_id(ffi::mjOBJ_JOINT, c"trunk_base_freejoint")?;
        self.joint_address(id, ffi::mjJNT_FREE)
    }

    pub(crate) fn reset(&mut self) -> bool {
        unsafe { ffi::mj_resetData(self.model.0.as_ptr(), self.data.0.as_ptr()) };
        let Some(root) = self.trunk_freejoint() else {
            return false;
        };
        self.set_position(root.qpos + 2, 0.125);
        self.set_position(root.qpos + 3, 1.0);
        true
    }

    pub(crate) fn set_position(&mut self, index: usize, value: f64) {
        assert!(index < self.model().nq as usize);
        unsafe { *self.data().qpos.add(index) = value };
    }

    pub(crate) fn forward(&mut self) {
        unsafe { ffi::mj_forward(self.model.0.as_ptr(), self.data.0.as_ptr()) };
    }

    pub(crate) fn qpos(&self, index: usize) -> f64 {
        assert!(index < self.model().nq as usize);
        unsafe { *self.data().qpos.add(index) }
    }

    pub(crate) fn qvel(&self, index: usize) -> f64 {
        assert!(index < self.model().nv as usize);
        unsafe { *self.data().qvel.add(index) }
    }

    pub(crate) fn joint_state(&self, joint: DrivenJoint) -> [f64; 4] {
        let JointAddress { qpos, dof } = joint.address;
        let q = self.qpos(qpos);
        let dq = self.qvel(dof);
        let data = self.data();
        // The DOF index is checked by qvel. MuJoCo supplies nefc entries in each
        // constraint array and the existing native lookup matches the joint ID.
        unsafe {
            let motor = *data.qfrc_actuator.add(dof);
            let mut external = -*data.qfrc_bias.add(dof) + *data.qfrc_constraint.add(dof);
            for i in 0..data.nefc as usize {
                if *data.efc_type.add(i) == ffi::mjCNSTR_FRICTION_DOF as c_int
                    && *data.efc_id.add(i) == joint.id
                {
                    external -= *data.efc_force.add(i);
                }
            }
            [q, dq, motor, external]
        }
    }

    pub(crate) fn control(&mut self, joint: DrivenJoint, torque: f64, friction: f64, damping: f64) {
        let model = self.model();
        let dof = joint.address.dof;
        assert!(joint.actuator < model.nu as usize && dof < model.nv as usize);
        unsafe {
            *self.data().ctrl.add(joint.actuator) = torque;
            *model.dof_frictionloss.add(dof) = friction;
            *model.dof_damping.add(dof) = damping;
        }
    }

    pub(crate) fn step(&mut self) -> bool {
        unsafe { ffi::mj_step(self.model.0.as_ptr(), self.data.0.as_ptr()) };
        self.data()
            .warning
            .iter()
            .all(|warning| warning.number == 0)
    }

    pub(crate) fn render(&mut self) {
        // Post-step displays use scratch data so the solver's constraint forces,
        // sensor sampling phase and warm-start state survive rendering.
        unsafe {
            ffi::mj_copyData(
                self.render.0.as_ptr(),
                self.model.0.as_ptr(),
                self.data.0.as_ptr(),
            );
            ffi::mj_kinematics(self.model.0.as_ptr(), self.render.0.as_ptr());
        }
    }

    pub(crate) fn body_id(&self, name: &CStr) -> Option<usize> {
        self.name_id(ffi::mjOBJ_BODY, name).map(|id| id as usize)
    }

    pub(crate) fn body_pose(&self, body: usize) -> [f64; 7] {
        assert!(body < self.model().nbody as usize);
        let render = unsafe { self.render.0.as_ref() };
        unsafe {
            let [x, y, z] = render.xpos.add(3 * body).cast::<[f64; 3]>().read();
            let [w, qx, qy, qz] = render.xquat.add(4 * body).cast::<[f64; 4]>().read();
            [x, y, z, w, qx, qy, qz]
        }
    }

    pub(crate) fn imu(&self) -> Option<([f64; 3], [f64; 3])> {
        let sensor = self.name_id(ffi::mjOBJ_SENSOR, c"imu_ang_vel")? as usize;
        let body = self.body_id(c"trunk_base")?;
        let model = self.model();
        let data = self.data();
        unsafe {
            if *model.sensor_dim.add(sensor) != 3 {
                return None;
            }
            let address = *model.sensor_adr.add(sensor) as usize;
            let gyro = data.sensordata.add(address).cast::<[f64; 3]>().read();
            // R^T * [0, 0, -1] in the same trunk frame as infer_policy.py.
            let [x, y, z] = data.xmat.add(9 * body + 6).cast::<[f64; 3]>().read();
            Some((gyro, [-x, -y, -z]))
        }
    }

    pub(crate) fn time(&self) -> f64 {
        self.data().time
    }

    pub(crate) fn contacts(&self) -> Vec<[c_int; 2]> {
        let data = self.data();
        // MuJoCo allocates contact data for the first ncon entries.
        (0..data.ncon as usize)
            .map(|i| unsafe { (*data.contact.add(i)).geom })
            .collect()
    }

    pub(crate) fn base_velocity(&self) -> [f64; 6] {
        let root = self.trunk_freejoint().expect("validated trunk freejoint");
        assert!(root.dof + 6 <= self.model().nv as usize);
        unsafe { self.data().qvel.add(root.dof).cast::<[f64; 6]>().read() }
    }
}

fn compile_model(scene: &CStr, vfs: &Vfs, force: f64, armature: f64) -> Result<Model, String> {
    if runtime_version() != ffi::mjVERSION_HEADER as c_int {
        return Err("MuJoCo runtime/header version mismatch".into());
    }

    let mut error = [0_u8; 2048];
    let mut spec = NonNull::new(unsafe {
        ffi::mj_parseXML(
            scene.as_ptr(),
            &vfs.0,
            error.as_mut_ptr().cast(),
            error.len() as c_int,
        )
    })
    .map(Spec)
    .ok_or_else(|| {
        CStr::from_bytes_until_nul(&error)
            .map(|message| message.to_string_lossy().into_owned())
            .unwrap_or_else(|_| "mj_parseXML failed".into())
    })?;

    configure_actuators(&mut spec, force, armature)?;
    let mut model =
        NonNull::new(unsafe { ffi::mj_compile(spec.0.as_ptr(), &vfs.0) }).ok_or_else(|| {
            let message = unsafe { ffi::mjs_getError(spec.0.as_ptr()) };
            if message.is_null() {
                "mj_compile failed".into()
            } else {
                unsafe { CStr::from_ptr(message) }
                    .to_string_lossy()
                    .into_owned()
            }
        })?;
    unsafe { model.as_mut().opt.timestep = 0.005 };
    Ok(Model(model))
}

fn configure_actuators(spec: &mut Spec, force: f64, armature: f64) -> Result<(), String> {
    // Edit the live spec before compilation, as in infer_policy.py::load_mujoco_with_bam.
    // The element kind passed to mjs_firstElement determines the kind of every item returned.
    unsafe {
        let spec = spec.0.as_ptr();
        let mut element = ffi::mjs_firstElement(spec, ffi::mjOBJ_ACTUATOR);
        while !element.is_null() {
            let actuator = &mut *ffi::mjs_asActuator(element);
            let target = ffi::mjs_getString(actuator.target);
            if target.is_null() {
                return Err("missing actuator target".into());
            }
            let target = CStr::from_ptr(target);
            if !target.to_bytes().starts_with(b"passive_") {
                let failure = ffi::mjs_setToMotor(actuator);
                if !failure.is_null() {
                    let failure = CStr::from_ptr(failure);
                    if !failure.to_bytes().is_empty() {
                        return Err(failure.to_string_lossy().into_owned());
                    }
                }
                actuator.forcelimited = 1;
                actuator.forcerange = [-force, force];
                actuator.ctrllimited = 0;
                actuator.gear.fill(0.0);
                actuator.gear[0] = 1.0;

                let joint = ffi::mjs_findElement(spec, ffi::mjOBJ_JOINT, target.as_ptr());
                if joint.is_null() {
                    return Err(format!(
                        "missing actuator joint {}",
                        target.to_string_lossy()
                    ));
                }
                let joint = &mut *ffi::mjs_asJoint(joint);
                joint.damping.fill(0.0);
                joint.frictionloss = 0.0;
                joint.armature = armature;
                joint.solref_friction = [-5e4, -2e2];
                joint.solimp_friction = [0.99, 0.9999, 0.001, 0.5, 2.0];
            }
            element = ffi::mjs_nextElement(spec, element);
        }
    }
    Ok(())
}

pub fn runtime_version() -> i32 {
    unsafe { ffi::mj_version() }
}
