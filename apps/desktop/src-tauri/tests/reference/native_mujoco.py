"""Pinned vendor CPU reference, with an explicit whole-frame physics-step delay adapter.

The reference imports the official BAM implementation, not the Rust translation. The model
setup function is extracted verbatim from the pinned vendor script without importing its GUI.
Run from the repository root; see README.md beside this file for the pinned environment.
"""
import ast
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
from bam.model import load_model

ROOT = Path(__file__).resolve().parents[5]
VENDOR = ROOT / 'vendor/microduck_rl/scripts/infer_policy.py'
BAM_REV = '62bd8ce12154340be97e06f7f41a0ca8f116d967'
script = ast.parse(VENDOR.read_text(encoding='utf-8'))
functions = [n for n in script.body if isinstance(n, ast.FunctionDef) and n.name == 'load_mujoco_with_bam']
namespace = dict(mujoco=mujoco, np=np, BAM_STIFF_SOLREF_FRICTION=(-5e4, -2e2),
                 BAM_STIFF_SOLIMP_FRICTION=(.99,.9999,.001,.5,2), BAM_MODEL='m6')
exec(compile(ast.Module(body=functions, type_ignores=[]), str(VENDOR), 'exec'), namespace)

# Declared before comparison: f64 absolute tolerances, no relative tolerance. Quaternion
# comparisons use the smaller q/-q distance. Units: rad, rad/s, m, s, gyro rad/s, gravity unit.
TOLERANCES = dict(positions=1e-7, velocities=1e-6, motor_positions=1e-7,
                  passive_positions=1e-7, trunk_position=1e-7, trunk_quaternion=1e-7,
                  gyro=1e-6, gravity=1e-7, torque=1e-6, base_velocity=1e-6, time=1e-12)

def run(delay_range, through_backlash=True, policy_path=None):
    bam = load_model(motor_name='xl330', model='m6')
    bam.actuator.kp = 200.; bam.actuator.vin = 7.4; bam.actuator.max_current = None
    scene = ROOT/'vendor/microduck_rl/src/mjlab_microduck/robot/microduck/scene_backlash.xml'
    model, data, controller, names = namespace['load_mujoco_with_bam'](str(scene), bam, .005, .1, 6.)
    # Names in canonical 15-slot order, with an explicit non-driven mouth slot at 9.
    names = names[:9] + [None] + names[9:]
    joints = [None if n is None else model.joint(n).id for n in names]
    qids = [None if j is None else model.jnt_qposadr[j] for j in joints]
    dids = [None if j is None else model.jnt_dofadr[j] for j in joints]
    pqids = [None if n is None else model.jnt_qposadr[model.joint('passive_'+n+'_backlash').id] for n in names]
    pdids = [None if n is None else model.jnt_dofadr[model.joint('passive_'+n+'_backlash').id] for n in names]
    home14 = np.array([0.,-.08726646259971647,-.457924,-.004940,.452984,
                       .3490658503988659,.3490658503988659,0.,0.,
                       0.,.08726646259971647,.457924,.004940,-.452984])
    home = np.insert(home14,9,0.)
    mujoco.mj_resetData(model,data)
    root = model.jnt_qposadr[model.joint('trunk_base_freejoint').id]
    data.qpos[root:root+7] = [0,0,.125,1,0,0,0]
    for q,value in zip(qids,home):
        if q is not None: data.qpos[q]=value
    controller.reset(data.qpos)
    mujoco.mj_forward(model,data)
    render = mujoco.MjData(model)
    active=home.copy(); queue=[]; frames=[]; step=0
    session = None
    previous_action=np.zeros(14,dtype=np.float32)
    if policy_path:
        import onnxruntime as ort
        options=ort.SessionOptions(); options.intra_op_num_threads=1; options.inter_op_num_threads=1
        session=ort.InferenceSession(str(policy_path),options,providers=['CPUExecutionProvider'])
    def sensors():
        motor=np.array([0 if q is None else data.qpos[q] for q in qids])
        passive=np.array([0 if q is None else data.qpos[q] for q in pqids])
        velocities=np.array([0 if d is None else data.qvel[d]+(data.qvel[p] if through_backlash else 0.) for d,p in zip(dids,pdids)])
        return motor+(passive if through_backlash else 0.),velocities
    for tick in range(300):
        target=home.copy(); observation=None; action=None
        # Hold, reversal and movement. Modest joint excitation includes foot/ground contacts.
        if tick >= 30:
            target[2] += .08*np.sin((tick-30)*.07)
            target[12] -= .08*np.sin((tick-30)*.07)
            target[7] += .12*np.sin((tick-30)*.11)
        if session:
            pos,vel=sensors()
            command=np.zeros(13,dtype=np.float32); command[0]=.2 if 75<=tick<225 else 0.
            b=model.body('trunk_base').id
            gravity=-data.xmat[b].reshape(3,3)[2,:]
            observation=np.concatenate([data.sensor('imu_ang_vel').data,gravity,
                np.delete(pos-home,9),np.delete(vel,9),previous_action,command]).astype(np.float32)
            action=session.run(None,{session.get_inputs()[0].name:observation.reshape(1,-1)})[0].reshape(-1)
            previous_action=action.copy()
            target=home+np.insert(action.astype(np.float64),9,0.)
        delay=delay_range[0] + tick % (delay_range[1]-delay_range[0]+1)
        at=step+delay
        queue=[item for item in queue if item[0]<at]
        queue.append((at,target.copy()))
        for substep in range(4):
            while queue and queue[0][0] <= step: _,active=queue.pop(0)
            for name,value in zip(names,active):
                if name is not None: controller.set_q_target(name,float(value))
            # Explicit training-encoder adapter: vendor BacklashEncoderBamActuator::get_command
            # reads motor+passive position for PWM error, while back-EMF/friction stay motor-side.
            # Subtracting passive from q_target is algebraically the same P error, without
            # changing the imported official controller or solver.
            if through_backlash:
                for name,pq in zip(names,pqids):
                    if name is not None: controller.set_q_target(name,controller.get_q_target(name)-data.qpos[pq])
            controller.update()
            mujoco.mj_step(model,data)
            step+=1
        mujoco.mj_copyData(render,model,data); mujoco.mj_kinematics(model,render)
        motor=np.array([0 if q is None else data.qpos[q] for q in qids])
        passive=np.array([0 if q is None else data.qpos[q] for q in pqids])
        velocity=np.array([0 if d is None else data.qvel[d]+data.qvel[p] for d,p in zip(dids,pdids)])
        b=model.body('trunk_base').id; r=data.xmat[b].reshape(3,3)
        gyro=data.sensor('imu_ang_vel').data.copy()
        if not through_backlash:
            velocity=np.array([0 if d is None else data.qvel[d] for d in dids])
        frames.append(dict(target=target.tolist(), observation=None if observation is None else observation.tolist(), action=None if action is None else action.tolist(), positions=(motor+passive if through_backlash else motor).tolist(),velocities=velocity.tolist(),
            motor_positions=motor.tolist(),passive_positions=passive.tolist(),trunk_position=render.xpos[b].tolist(),
            trunk_quaternion=render.xquat[b].tolist(),gyro=gyro.tolist(),gravity=(-r[2,:]).tolist(),
            torque=controller._prev_motor_torque.tolist(),base_velocity=data.qvel[:6].tolist(),
            contacts=[list(map(int,c.geom)) for c in data.contact],time=float(data.time),physics_step=step))
    return dict(delay=delay_range,encoder_through_backlash=through_backlash,frames=frames)

if __name__=='__main__':
    output=ROOT/'apps/desktop/src-tauri/tests/fixtures/native-mujoco-xl330.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    fixture=dict(source='fixed vendor CPU reference; not an upstream published golden', bam_revision=BAM_REV,
        asset_sha256={str(p.relative_to(ROOT/'vendor/microduck_rl/src/mjlab_microduck/robot/microduck')).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((ROOT/'vendor/microduck_rl/src/mjlab_microduck/robot/microduck').rglob('*')) if p.suffix.lower() in ('.xml','.stl')},
        mujoco_version=mujoco.__version__, vendor_script_sha256=hashlib.sha256(VENDOR.read_bytes()).hexdigest(),
        tolerances=TOLERANCES, observation_encoder='motor + passive (training encoder-through-backlash)',
        controller_encoder='separate CPU motor-only and training encoder-through-backlash cases', sampling='qpos/qvel post-step; IMU official mj_step evaluation phase; render post-step scratch kinematics',
        cases=[run([0,0],False),run([0,0]),run([3,3]),run([6,6]),run([3,6])])
    output.write_text(json.dumps(fixture,separators=(',',':')),encoding='utf-8')
    print(output, hashlib.sha256(output.read_bytes()).hexdigest())
    policy=ROOT/'.agentic/velstand.onnx'
    if policy.is_file():
        policy_output=ROOT/'.agentic/native-policy-reference.json'
        policy_tolerances=dict(observation=2e-4,action=2e-5,target=2e-5,positions=2e-5,velocities=2e-4,
            motor_positions=2e-5,passive_positions=2e-5,trunk_position=2e-5,trunk_quaternion=2e-5,base_velocity=2e-4,gyro=2e-4,gravity=2e-5,time=1e-12)
        policy_fixture=dict(source=fixture['source'], policy_sha256=hashlib.sha256(policy.read_bytes()).hexdigest(),
            asset_sha256=fixture['asset_sha256'], bam_revision=BAM_REV, vendor_script_sha256=fixture['vendor_script_sha256'], mujoco_version=mujoco.__version__,
            tolerances=policy_tolerances, case=run([3,6],True,policy))
        policy_output.write_text(json.dumps(policy_fixture,separators=(',',':')),encoding='utf-8')
        print(policy_output,hashlib.sha256(policy_output.read_bytes()).hexdigest())
