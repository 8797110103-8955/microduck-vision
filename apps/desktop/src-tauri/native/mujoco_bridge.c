// Thin field-access binding compiled against the pinned official MuJoCo 3.10.0 headers.
// Physics and constraints remain entirely in MuJoCo; BAM control is implemented in Rust.
#include <mujoco/mujoco.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if mjVERSION_HEADER != 3010000
#error MuJoCo binding requires 3.10.0
#endif

typedef struct { mjModel* model; mjData* data; mjData* render; } DuckMj;

int duck_mj_version(void) { return mj_version(); }

DuckMj* duck_mj_open(const char* path, double force, double armature, char* error, int size) {
  if (mj_version() != mjVERSION_HEADER) {
    snprintf(error, size, "MuJoCo runtime/header version mismatch");
    return NULL;
  }
  mjSpec* spec = mj_parseXML(path, NULL, error, size);
  if (!spec) return NULL;
  // infer_policy.py::load_mujoco_with_bam, before compilation (including dampratio handling).
  for (mjsElement* e = mjs_firstElement(spec, mjOBJ_ACTUATOR); e; e = mjs_nextElement(spec, e)) {
    mjsActuator* a = mjs_asActuator(e);
    const char* target = mjs_getString(a->target);
    if (strncmp(target, "passive_", 8) == 0) continue;
    const char* failure = mjs_setToMotor(a);
    if (failure && failure[0]) { snprintf(error, size, "%s", failure); mj_deleteSpec(spec); return NULL; }
    a->forcelimited = 1;
    a->forcerange[0] = -force; a->forcerange[1] = force;
    a->ctrllimited = 0;
    memset(a->gear, 0, sizeof(a->gear)); a->gear[0] = 1;
    mjsElement* je = mjs_findElement(spec, mjOBJ_JOINT, target);
    if (!je) { snprintf(error, size, "missing actuator joint %s", target); mj_deleteSpec(spec); return NULL; }
    mjsJoint* j = mjs_asJoint(je);
    memset(j->damping, 0, sizeof(j->damping));
    j->frictionloss = 0;
    j->armature = armature;
    j->solref_friction[0] = -5e4; j->solref_friction[1] = -2e2;
    const double solimp[5] = {0.99, 0.9999, 0.001, 0.5, 2.0};
    memcpy(j->solimp_friction, solimp, sizeof(solimp));
  }
  mjModel* model = mj_compile(spec, NULL);
  if (!model) snprintf(error, size, "%s", mjs_getError(spec));
  mj_deleteSpec(spec);
  if (!model) return NULL;
  model->opt.timestep = 0.005;
  mjData* data = mj_makeData(model);
  if (!data) { mj_deleteModel(model); snprintf(error, size, "mj_makeData failed"); return NULL; }
  DuckMj* engine = (DuckMj*)calloc(1, sizeof(DuckMj));
  if (!engine) { mj_deleteData(data); mj_deleteModel(model); return NULL; }
  engine->model = model; engine->data = data; engine->render = mj_makeData(model);
  if (!engine->render) { mj_deleteData(data); mj_deleteModel(model); free(engine); return NULL; }
  return engine;
}

void duck_mj_close(DuckMj* e) {
  if (e) { mj_deleteData(e->render); mj_deleteData(e->data); mj_deleteModel(e->model); free(e); }
}

// Object IDs are resolved by name, never inferred from canonical array offsets.
int duck_mj_joint(DuckMj* e, const char* name, int* actuator, int* qpos, int* dof) {
  int j = mj_name2id(e->model, mjOBJ_JOINT, name);
  int a = mj_name2id(e->model, mjOBJ_ACTUATOR, name);
  if (j < 0 || a < 0 || e->model->jnt_type[j] != mjJNT_HINGE ||
      e->model->actuator_trnid[2*a] != j) return -1;
  *actuator = a; *qpos = e->model->jnt_qposadr[j]; *dof = e->model->jnt_dofadr[j];
  return j;
}

int duck_mj_passive(DuckMj* e, const char* name, int* qpos, int* dof) {
  int j = mj_name2id(e->model, mjOBJ_JOINT, name);
  if (j < 0) return 0;
  if (e->model->jnt_type[j] != mjJNT_HINGE) return -1;
  *qpos = e->model->jnt_qposadr[j]; *dof = e->model->jnt_dofadr[j];
  return 1;
}

int duck_mj_reset(DuckMj* e) {
  mj_resetData(e->model, e->data);
  int j = mj_name2id(e->model, mjOBJ_JOINT, "trunk_base_freejoint");
  if (j < 0 || e->model->jnt_type[j] != mjJNT_FREE) return 0;
  int q = e->model->jnt_qposadr[j];
  e->data->qpos[q+2] = 0.125;
  e->data->qpos[q+3] = 1;
  return 1;
}
void duck_mj_set_position(DuckMj* e, int q, double value) { e->data->qpos[q] = value; }
void duck_mj_forward(DuckMj* e) { mj_forward(e->model, e->data); }
// Recompute post-integration transforms in separate scratch data. Never change the solver's
// warm-start, constraint forces or sensor sampling phase just to render a frame.
void duck_mj_render(DuckMj* e) {
  mj_copyData(e->render, e->model, e->data);
  mj_kinematics(e->model, e->render);
}
void duck_mj_state(DuckMj* e, int joint, int q, int dof, double* values) {
  values[0] = e->data->qpos[q]; values[1] = e->data->qvel[dof];
  values[2] = e->data->qfrc_actuator[dof];
  values[3] = -e->data->qfrc_bias[dof] + e->data->qfrc_constraint[dof];
  for (int i=0; i<e->data->nefc; ++i)
    if (e->data->efc_type[i] == mjCNSTR_FRICTION_DOF && e->data->efc_id[i] == joint)
      values[3] -= e->data->efc_force[i];
}
double duck_mj_qpos(DuckMj* e, int q) { return e->data->qpos[q]; }
double duck_mj_qvel(DuckMj* e, int dof) { return e->data->qvel[dof]; }
void duck_mj_control(DuckMj* e, int a, int dof, double torque, double friction, double damping) {
  e->data->ctrl[a] = torque;
  e->model->dof_frictionloss[dof] = friction;
  e->model->dof_damping[dof] = damping;
}
int duck_mj_step(DuckMj* e) {
  mj_step(e->model, e->data);
  for (int i=0; i<mjNWARNING; ++i) if (e->data->warning[i].number) return 0;
  return 1;
}
int duck_mj_body(DuckMj* e, const char* name) { return mj_name2id(e->model, mjOBJ_BODY, name); }
void duck_mj_pose(DuckMj* e, int body, double* pose) {
  memcpy(pose, e->render->xpos+3*body, 3*sizeof(double));
  memcpy(pose+3, e->render->xquat+4*body, 4*sizeof(double));
}
int duck_mj_imu(DuckMj* e, double* gyro, double* gravity) {
  int s = mj_name2id(e->model, mjOBJ_SENSOR, "imu_ang_vel");
  int b = mj_name2id(e->model, mjOBJ_BODY, "trunk_base");
  if (s<0 || b<0 || e->model->sensor_dim[s] != 3) return 0;
  memcpy(gyro, e->data->sensordata+e->model->sensor_adr[s], 3*sizeof(double));
  // R^T * [0,0,-1], in the same trunk frame as infer_policy.py::get_projected_gravity.
  const double* r = e->data->xmat+9*b;
  gravity[0] = -r[6]; gravity[1] = -r[7]; gravity[2] = -r[8];
  return 1;
}
double duck_mj_time(DuckMj* e) { return e->data->time; }
int duck_mj_contact_count(DuckMj* e) { return e->data->ncon; }
void duck_mj_contact(DuckMj* e, int index, int* pair) {
  pair[0] = e->data->contact[index].geom[0];
  pair[1] = e->data->contact[index].geom[1];
}
void duck_mj_base_velocity(DuckMj* e, double* velocity) {
  int j = mj_name2id(e->model, mjOBJ_JOINT, "trunk_base_freejoint");
  memcpy(velocity, e->data->qvel+e->model->jnt_dofadr[j], 6*sizeof(double));
}
