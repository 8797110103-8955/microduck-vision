import * as commands from "../../../generated/types";
import { computed, defineComponent, onMounted, ref } from "vue";
import { useDesktopStore } from "../../store";
import { Select } from "../../components/select";
import { SceneStage } from "../scene-stage";
import style from "./style.module.scss";

const defaultTask = "Mjlab-Velocity-Flat-MicroDuck";

function inputValue(event: Event) {
  return event.currentTarget instanceof HTMLInputElement ? event.currentTarget.value : "";
}

export const TrainingPage = defineComponent(() => {
  const store = useDesktopStore();
  const task = ref(defaultTask);
  const workingDir = ref("vendor/microduck_rl");
  const virtualMachineId = ref("");
  const policyId = ref("");
  const experimentId = ref("");
  const recipe = ref("student-distillation");
  const teacherMotionProvenance = ref("teacher:official-motion");
  const terrain = ref("");
  const parallelEnvironments = ref("");
  const domainRandomization = ref(true);
  const teacherPolicyId = ref("");
  const evaluationSteps = ref("100");
  const evaluationSeed = ref("0");
  const preparing = ref(false);
  const error = ref<string | null>(null);
  const progress = computed(() => store.state.training);
  const machines = computed(() => store.state.virtualMachines);
  const policies = computed(() => store.state.policies);
  const teacherPolicies = computed(() =>
    policies.value.filter((policy) => policy.kind === "teacher-training" && policy.cached),
  );
  const running = computed(
    () => progress.value?.phase === "starting" || progress.value?.phase === "running",
  );
  const busy = computed(() => preparing.value || running.value);
  const useTeacherDataset = computed(() => Boolean(store.state.teacherDatasetPath));

  const start = async () => {
    preparing.value = true;
    try {
      error.value = null;
      const args = ["run", "train", task.value.trim()];
      if (!useTeacherDataset.value) {
        if (terrain.value) args.push("--terrain", terrain.value);
        if (parallelEnvironments.value) {
          const count = Number(parallelEnvironments.value);
          if (!Number.isSafeInteger(count) || count < 1) throw new Error("并行环境数须为正整数");
          args.push("--num-envs", String(count));
        }
        if (!domainRandomization.value) args.push("--domain-randomization", "off");
      }
      const started = await commands.trainingStart({
        program: "uv",
        args,
        workingDir: workingDir.value.trim(),
        virtualMachineId: virtualMachineId.value,
        policyId: policyId.value || null,
        experimentId: experimentId.value.trim() || null,
        recipe: store.state.teacherDatasetPath ? "recording-behavior-cloning" : recipe.value.trim(),
        teacherMotionProvenance: store.state.teacherDatasetPath
          ? `recording-dataset:${store.state.teacherDatasetPath}`
          : teacherMotionProvenance.value.trim(),
        datasetPath: store.state.teacherDatasetPath || null,
      });
      if (store.state.training?.runId !== started.runId) store.applyTrainingProgress(started);
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      preparing.value = false;
    }
  };

  const stop = async () => {
    try {
      error.value = null;
      await commands.trainingStop();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const evaluateTeacher = async () => {
    preparing.value = true;
    error.value = null;
    try {
      const steps = Number(evaluationSteps.value);
      const seed = Number(evaluationSeed.value);
      if (
        !Number.isSafeInteger(steps) ||
        steps < 1 ||
        steps > 5000 ||
        !Number.isSafeInteger(seed) ||
        seed < 0 ||
        seed > 2147483647
      ) {
        throw new Error("评估步数须为 1 到 5000，种子须为非负整数");
      }
      const started = await commands.trainingEvaluateTeacher({
        workingDir: workingDir.value.trim(),
        virtualMachineId: virtualMachineId.value,
        teacherPolicyId: teacherPolicyId.value,
        taskId: task.value.trim(),
        steps,
        seed,
      });
      if (store.state.training?.runId !== started.runId) store.applyTrainingProgress(started);
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      preparing.value = false;
    }
  };

  onMounted(() => {
    virtualMachineId.value =
      store.state.selectedVirtualMachine ?? store.state.virtualMachines[0]?.id ?? "";
    policyId.value = store.state.selectedPolicyId ?? "";
    void commands
      .trainingStatus()
      .then((value) => {
        if (!store.state.training) store.applyTrainingProgress(value);
      })
      .catch(() => undefined);
  });
  return () => (
    <>
      <SceneStage training={store.state.trainingFrame} />
      <section class={style.trainingPage} aria-label="本地训练">
        <header class={style.header}>
          <div>
            <h2>本地训练</h2>
            <p>训练进程在本机运行，输出会实时同步到这里。</p>
          </div>
          <span class={[style.phase, busy.value ? style.active : undefined]}>
            {preparing.value
              ? "准备任务"
              : progress.value?.phase === "finished"
                ? "已完成"
                : progress.value?.phase === "failed"
                  ? "失败"
                  : progress.value?.phase === "stopped"
                    ? "已停止"
                    : running.value
                      ? "运行中"
                      : "未启动"}
          </span>
        </header>

        <div class={style.form}>
          <label>
            <span>教师数据集目录（可选）</span>
            <input
              value={store.state.teacherDatasetPath}
              disabled={busy.value}
              onInput={(event) => (store.state.teacherDatasetPath = inputValue(event))}
            />
          </label>
          {store.state.teacherDatasetPath && (
            <p>将使用导出的观测和教师 action 训练学生策略，并输出 student.pt 与 student.onnx。</p>
          )}
          <label>
            <span>模拟机</span>
            <Select
              value={virtualMachineId.value}
              disabled={busy.value}
              onChange={(event: Event) => {
                if (event.currentTarget instanceof HTMLSelectElement)
                  virtualMachineId.value = event.currentTarget.value;
              }}
            >
              <option value="">请选择模拟机</option>
              {machines.value.map((machine) => (
                <option value={machine.id} key={machine.id}>
                  {machine.name} · {machine.bodyId} · {machine.modelKind}
                </option>
              ))}
            </Select>
          </label>
          <label>
            <span>参考策略（可选）</span>
            <Select
              value={policyId.value}
              disabled={busy.value}
              onChange={(event: Event) => {
                if (event.currentTarget instanceof HTMLSelectElement)
                  policyId.value = event.currentTarget.value;
              }}
            >
              <option value="">使用固定 61/14/50 布局</option>
              {policies.value.map((policy) => (
                <option value={policy.id} key={policy.id}>
                  {policy.name} · {policy.cached ? "已缓存" : "未缓存"}
                </option>
              ))}
            </Select>
          </label>
          <label>
            <span>实验 ID（留空自动生成）</span>
            <input
              value={experimentId.value}
              disabled={busy.value}
              onInput={(event) => (experimentId.value = inputValue(event))}
            />
          </label>
          <label>
            <span>Recipe</span>
            <input
              value={recipe.value}
              disabled={busy.value}
              onInput={(event) => (recipe.value = inputValue(event))}
            />
          </label>
          <label>
            <span>Teacher / motion provenance</span>
            <input
              value={teacherMotionProvenance.value}
              disabled={busy.value}
              onInput={(event) => (teacherMotionProvenance.value = inputValue(event))}
            />
          </label>
          <label>
            <span>任务</span>
            <input
              value={task.value}
              disabled={busy.value}
              onInput={(event) => (task.value = inputValue(event))}
            />
          </label>
          <label>
            <span>训练项目目录</span>
            <input
              value={workingDir.value}
              disabled={busy.value}
              onInput={(event) => (workingDir.value = inputValue(event))}
            />
          </label>
          {!useTeacherDataset.value && (
            <>
              <label>
                <span>任务场地</span>
                <Select
                  value={terrain.value}
                  disabled={busy.value}
                  onChange={(event: Event) => {
                    if (event.currentTarget instanceof HTMLSelectElement)
                      terrain.value = event.currentTarget.value;
                  }}
                >
                  <option value="">任务默认</option>
                  <option value="plane">平地</option>
                  <option value="generator">程序地形</option>
                </Select>
              </label>
              <label>
                <span>并行环境数</span>
                <input
                  type="number"
                  min="1"
                  step="1"
                  placeholder="任务默认"
                  value={parallelEnvironments.value}
                  disabled={busy.value}
                  onInput={(event) => (parallelEnvironments.value = inputValue(event))}
                />
              </label>
              <label>
                <span>域随机化</span>
                <input
                  type="checkbox"
                  checked={domainRandomization.value}
                  disabled={busy.value}
                  onChange={(event) => {
                    if (event.currentTarget instanceof HTMLInputElement)
                      domainRandomization.value = event.currentTarget.checked;
                  }}
                />
              </label>
            </>
          )}
          <div class={style.actions}>
            <button
              type="button"
              disabled={busy.value || !virtualMachineId.value || !task.value.trim()}
              onClick={() => void start()}
            >
              开始训练
            </button>
            <button type="button" disabled={!running.value} onClick={() => void stop()}>
              停止
            </button>
          </div>
        </div>

        {teacherPolicies.value.length > 0 && (
          <section class={style.form} aria-label="教师仿真评估">
            <label>
              <span>教师产物</span>
              <Select
                value={teacherPolicyId.value}
                disabled={busy.value}
                onChange={(event: Event) => {
                  if (event.currentTarget instanceof HTMLSelectElement)
                    teacherPolicyId.value = event.currentTarget.value;
                }}
              >
                <option value="">请选择学生模型</option>
                {teacherPolicies.value.map((policy) => (
                  <option value={policy.id} key={policy.id}>
                    {policy.name}
                  </option>
                ))}
              </Select>
            </label>
            <label>
              <span>评估步数</span>
              <input
                type="number"
                min="1"
                max="5000"
                step="1"
                value={evaluationSteps.value}
                disabled={busy.value}
                onInput={(event) => (evaluationSteps.value = inputValue(event))}
              />
            </label>
            <label>
              <span>评估种子</span>
              <input
                type="number"
                min="0"
                max="2147483647"
                step="1"
                value={evaluationSeed.value}
                disabled={busy.value}
                onInput={(event) => (evaluationSeed.value = inputValue(event))}
              />
            </label>
            <div class={style.actions}>
              <button
                type="button"
                disabled={
                  busy.value ||
                  !teacherPolicyId.value ||
                  !virtualMachineId.value ||
                  !task.value.trim()
                }
                onClick={() => void evaluateTeacher()}
              >
                评估所选教师产物
              </button>
            </div>
          </section>
        )}

        {error.value && <p class={style.error}>{error.value}</p>}
        {progress.value && (
          <div class={style.progressCard}>
            <div class={style.progressHeader}>
              <strong>{progress.value.message || "等待训练输出…"}</strong>
              {progress.value.percent != null && <span>{progress.value.percent.toFixed(1)}%</span>}
            </div>
            <div class={style.track}>
              <div style={{ width: `${progress.value.percent ?? 0}%` }} />
            </div>
            <div class={style.metrics}>
              <span>
                step {progress.value.step ?? "—"}
                {progress.value.totalSteps != null && ` / ${progress.value.totalSteps}`}
              </span>
              <span>reward {progress.value.reward?.toFixed(4) ?? "—"}</span>
              <span>loss {progress.value.loss?.toFixed(4) ?? "—"}</span>
            </div>
            <div class={style.metadata}>
              <span>run {progress.value.runId}</span>
              <span>
                {progress.value.metadata.bodyTemplateId} v{progress.value.metadata.bodyVersion} ·{" "}
                {progress.value.metadata.servoProfile}
              </span>
              <span>
                {progress.value.metadata.policyLayout.obsLen}/
                {progress.value.metadata.policyLayout.actionLen} ·{" "}
                {progress.value.metadata.policyLayout.controlHz} Hz
              </span>
              <span>{progress.value.metadata.recipe}</span>
            </div>
            <small class={style.resultDir}>结果目录：{progress.value.resultDir}</small>
            {progress.value.line && <pre>{progress.value.line}</pre>}
          </div>
        )}
      </section>
    </>
  );
});
