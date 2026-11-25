import * as commands from "../../../generated/types";
import type { TrainingProvenance } from "../../../generated/types";
import { useIntervalFn, useRafFn } from "@vueuse/core";
import {
  computed,
  defineComponent,
  onMounted,
  onScopeDispose,
  reactive,
  ref,
  shallowRef,
  watch,
} from "vue";
import { useDesktopStore } from "../../store";
import { Checkbox } from "../../components/checkbox";
import { Select } from "../../components/select";
import { ScrollArea } from "../../components/scroll-area";
import { showMessage } from "../../components/message";
import { SceneStage } from "../scene-stage";
import { TaskConfiguration } from "./task-configuration";
import style from "./style.module.scss";

const deepmimicTask = "Microduck-DeepMimic-Velstand-Flat-v1";
const baselineMethod = "ppo-baseline";

function isBaseline(experimentId: string, method?: string) {
  return method === baselineMethod || experimentId.toLowerCase().startsWith(`${baselineMethod}-`);
}

function inputValue(event: Event) {
  return event.currentTarget instanceof HTMLInputElement ? event.currentTarget.value : "";
}

function selectValue(select: HTMLSelectElement, previous: string) {
  if (!select.value) {
    select.value = previous;
    return previous;
  }
  return select.value;
}

function teacherSlug(recordingId: string, bundleName?: string) {
  const prefix = recordingId.replace(/(?:-\d+){4}$/, "");
  const name = prefix === recordingId ? (bundleName?.split(" · ")[0] ?? prefix) : prefix;
  return (
    name
      .toLowerCase()
      .normalize("NFKD")
      .replace(/\p{Mark}/gu, "")
      .replace(/[^a-z0-9]+/g, "-")
      .replace(/^-+|-+$/g, "")
      .replace(/-deepmimic$/, "") || "teacher"
  );
}

function uniqueExperimentId(prefix: string, existing: ReadonlySet<string>) {
  let id: string;
  do {
    id = `${prefix}-${crypto.randomUUID().slice(0, 7)}`;
  } while (existing.has(id));
  return id;
}

export const TrainingPage = defineComponent(() => {
  const store = useDesktopStore();
  const progress = computed(() => store.state.training);
  const task = ref("");
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const rounds = ref<Awaited<ReturnType<typeof commands.trainingRounds>>>([]);
  const loadingRounds = ref(true);
  const roundsError = ref<string | null>(null);
  const enteredExperimentId = ref<string | null>(null);
  const replaceExperiment = ref(true);
  const baselineOpen = ref(false);
  const publicationOpen = ref(false);
  const baselineHistory = computed(() => baselineOpen.value && !publicationOpen.value);
  const histories = reactive({
    training: { experimentId: "", runId: "" },
    baseline: { experimentId: "", runId: "" },
    publication: { experimentId: "", runId: "" },
  });
  const currentHistory = computed(() =>
    publicationOpen.value
      ? histories.publication
      : baselineOpen.value
        ? histories.baseline
        : histories.training,
  );
  const initializationChoice = ref<
    | "fresh"
    | NonNullable<Parameters<typeof commands.trainingStart>[0]["initialization"]>["mode"]
    | null
  >(null);
  const publicationName = ref<string | null>(null);
  const publicationTargets = ref<Awaited<ReturnType<typeof commands.trainingPublicationTargets>>>(
    [],
  );
  const publicationTargetId = ref("");
  const loadingPublicationTargets = ref(false);
  let publicationTargetsEpoch = 0;
  const resetNormalization = ref(false);
  const publishing = ref(false);
  const previewing = ref(false);
  const candidatePreview = shallowRef<Awaited<ReturnType<typeof commands.trainingPreview>>>();
  const candidateIndex = ref(0);
  let previewEpoch = 0;
  let playbackStarted: number | undefined;
  const playback = useRafFn(
    ({ timestamp }) => {
      const preview = candidatePreview.value;
      if (!preview) return;
      playbackStarted ??= timestamp;
      candidateIndex.value = Math.min(
        Math.floor((timestamp - playbackStarted) / preview.sampleIntervalMs),
        preview.frames.length - 1,
      );
      if (candidateIndex.value === preview.frames.length - 1) playback.pause();
    },
    { immediate: false },
  );
  const candidateFrame = computed(() => {
    const preview = publicationOpen.value ? candidatePreview.value : undefined;
    const frame = preview?.frames[candidateIndex.value];
    return frame && preview ? { ...frame, previewScene: preview.scene } : undefined;
  });
  const previewSummary = computed(() => {
    const report = candidatePreview.value?.report;
    if (!report) return "";
    const seconds = report.seconds?.toFixed(1) ?? "—";
    const forward = report.displacementM[0]?.toFixed(2) ?? "—";
    const upright =
      report.uprightFraction == null ? "—" : `${(report.uprightFraction * 100).toFixed(0)}%`;
    return `预览 ${seconds} 秒，前向位移 ${forward} 米，直立占比 ${upright}`;
  });
  const publicationMessage = ref("");
  const matchingRounds = computed(() =>
    rounds.value.filter(
      (round) =>
        round.record.taskId === task.value &&
        !!selectedMachine.value &&
        round.progress?.metadata.servoProfile === selectedMachine.value.servoModel,
    ),
  );
  const visibleRounds = computed(() =>
    publicationOpen.value || baselineOpen.value
      ? matchingRounds.value
      : matchingRounds.value.filter(
          (round) =>
            !isBaseline(
              round.record.experimentId,
              round.progress?.metadata.deepmimic?.trainingMethod,
            ),
        ),
  );
  const experimentIds = computed(() => [
    ...new Set(visibleRounds.value.map((round) => round.record.experimentId)),
  ]);
  const selectedExperimentId = computed(() =>
    experimentIds.value.includes(currentHistory.value.experimentId)
      ? currentHistory.value.experimentId
      : baselineHistory.value
        ? ""
        : (experimentIds.value[0] ?? ""),
  );
  const historyRounds = computed(() =>
    visibleRounds.value
      .filter((round) => round.record.experimentId === selectedExperimentId.value)
      .toSorted((a, b) => b.record.round - a.record.round),
  );
  const selectedRound = computed(
    () =>
      historyRounds.value.find((round) => round.record.runId === currentHistory.value.runId) ??
      historyRounds.value[0],
  );
  const selectedCheckpoint = computed(() => selectedRound.value?.checkpoints.at(-1));
  const canOpenPublication = computed(
    () => selectedRound.value?.progress?.phase === "finished" && !!selectedCheckpoint.value,
  );
  const compatiblePublicationTargets = computed(() =>
    publicationTargets.value.filter(
      (target) => target.servoModel === selectedRound.value?.progress?.metadata.servoProfile,
    ),
  );
  const initializationMode = computed(() => {
    const round = selectedRound.value;
    const active = progress.value;
    if (
      round &&
      active?.runId === round.record.runId &&
      (active.phase === "starting" || active.phase === "running")
    )
      return round.record.initialization?.mode ?? "fresh";
    return initializationChoice.value ?? (selectedCheckpoint.value ? "resume" : "fresh");
  });
  const usedExperimentIds = computed(
    () => new Set(rounds.value.map((round) => round.record.experimentId.toLowerCase())),
  );
  const newExperimentId = computed(
    () => enteredExperimentId.value?.trim() || defaultExperimentId.value,
  );
  const sameNameRounds = computed(() =>
    rounds.value.filter(
      (round) => round.record.experimentId.toLowerCase() === newExperimentId.value.toLowerCase(),
    ),
  );
  const canReplaceExperiment = computed(
    () =>
      sameNameRounds.value.length > 0 &&
      sameNameRounds.value.every((round) => matchingRounds.value.includes(round)),
  );
  const experimentConflict = computed(() =>
    sameNameRounds.value.length === 0
      ? null
      : !canReplaceExperiment.value
        ? "训练 ID 已被其他任务或电机使用，请更换 ID"
        : !replaceExperiment.value
          ? "训练 ID 已存在，请更换 ID 或勾选删除同名训练"
          : null,
  );
  watch([task, () => selectedMachine.value?.servoModel], () => {
    for (const history of Object.values(histories)) {
      history.experimentId = "";
      history.runId = "";
    }
    enteredExperimentId.value = null;
    publicationTargetId.value = "";
    replaceExperiment.value = true;
    initializationChoice.value = null;
  });
  const tasks = computed(() => store.state.trainingTasks ?? []);
  const taskHint = computed(() =>
    store.state.trainingTasksError
      ? `无法读取训练任务，请排查 Python 环境（uv、依赖和训练项目）。\n${store.state.trainingTasksError}`
      : store.state.trainingTasks === null
        ? "正在读取 Python 注册的任务…"
        : undefined,
  );
  watch(
    tasks,
    (values) => {
      task.value = values.includes(deepmimicTask) ? deepmimicTask : "";
    },
    { immediate: true },
  );
  const policyId = ref("");
  const recipe = ref("student-distillation");
  const teacherMotionProvenance = ref("teacher:official-motion");
  const terrain = ref("");
  const parameters = reactive({ numEnvs: "64", iterations: "50", seed: "0" });
  const domainRandomization = ref(false);
  const teacherPolicyId = ref("");
  const evaluationSteps = ref("100");
  const evaluationSeed = ref("0");
  const preparing = ref(false);
  const stopping = ref(false);
  const error = ref<string | null>(null);
  const deepmimic = computed(() => task.value === deepmimicTask);
  const supportsDeepmimic = computed(
    () =>
      selectedMachine.value?.bodyId === "official" && selectedMachine.value.modelKind === "xl330",
  );
  const baselineDefaults = reactive({ bundleId: "", numEnvs: "64", iterations: "50", seed: "0" });
  const baselineSettings = reactive({ ...baselineDefaults });
  const bundles = ref<Awaited<ReturnType<typeof commands.deepmimicBundles>>>([]);
  const bundleId = ref("");
  const bundleError = ref<string | null>(null);
  const loadingBundles = ref(false);
  let alive = true;
  let bundleEpoch = 0;
  let roundsEpoch = 0;
  let formEpoch = 0;
  onScopeDispose(() => {
    alive = false;
    bundleEpoch++;
  });
  const policies = computed(() => store.state.policies);
  const teacherPolicies = computed(() =>
    policies.value.filter((policy) => policy.kind === "teacher-training" && policy.cached),
  );
  const running = computed(
    () => progress.value?.phase === "starting" || progress.value?.phase === "running",
  );
  const busy = computed(() => preparing.value || running.value);
  const compatibleBundles = computed(() =>
    bundles.value.filter(
      (bundle) => bundle.body_id === selectedMachine.value?.bodyId && bundle.control_s === 0.02,
    ),
  );
  const selectedBundle = computed(() =>
    compatibleBundles.value.find((bundle) => bundle.id === bundleId.value),
  );
  const selectedBaselineBundle = computed(() =>
    compatibleBundles.value.find((bundle) => bundle.id === baselineSettings.bundleId),
  );
  const defaultExperimentId = computed(() => {
    const existing = usedExperimentIds.value;
    if (deepmimic.value) {
      const bundle = selectedBundle.value;
      const prefix = `deepmimic-${teacherSlug(bundle?.recording_id ?? "", bundle?.name)}`;
      return uniqueExperimentId(prefix, existing);
    }
    let next = 1;
    while (existing.has(`experiment-${next}`)) next++;
    return `experiment-${next}`;
  });
  const rewardSourceName = (id: string) =>
    bundles.value.find((bundle) => bundle.id === id)?.name ?? id;
  const defaultPublicationName = computed(() => {
    const checkpoint = selectedCheckpoint.value;
    if (!checkpoint) return "";
    const hash = checkpoint.sha256.slice(0, 7);
    const round = selectedRound.value;
    const source = round?.progress?.metadata.deepmimic;
    if (!source) return `policy-${hash}`;
    const experiment = round.record.experimentId;
    if (/^(deepmimic|ppo-baseline)-[a-z0-9-]+-[a-f0-9]{7}$/i.test(experiment)) return experiment;
    const bundle = bundles.value.find((candidate) => candidate.id === source.bundleId);
    const prefix = source.trainingMethod === baselineMethod ? baselineMethod : "deepmimic";
    return `${prefix}-${teacherSlug(source.sourceRecordingId, bundle?.name)}-${hash}`;
  });
  const chosenPublicationName = computed(
    () => publicationName.value?.trim() || defaultPublicationName.value,
  );
  const hasPublishedPolicy = (name: string) => {
    const normalized = name.toLowerCase();
    return policies.value.some(
      (policy) => policy.kind === "training" && policy.name.trim().toLowerCase() === normalized,
    );
  };
  const publicationNameExists = computed(() => hasPublishedPolicy(chosenPublicationName.value));
  const previousRewardSource = computed(() => {
    const source = selectedRound.value?.progress?.metadata.deepmimic;
    if (!source) return "";
    if (source.trainingMethod === baselineMethod) return "PPO 基线";
    return rewardSourceName(source.bundleId);
  });
  const rewardSourceLocked = computed(() => initializationMode.value === "resume");
  const restoreDeepmimicSettings = (value: Awaited<ReturnType<typeof commands.trainingStatus>>) => {
    const source = value?.metadata.deepmimic;
    if (!source || source.trainingMethod === baselineMethod) return;
    bundleId.value = source.bundleId;
    parameters.numEnvs = String(source.numEnvs);
    parameters.iterations = String(source.iterations);
    parameters.seed = String(source.seed);
    domainRandomization.value = false;
  };
  const restore = (value: Awaited<ReturnType<typeof commands.trainingStatus>>) => {
    if (!value?.metadata.deepmimic) return;
    if (initializationMode.value !== "resume" || !selectedRound.value?.progress?.metadata.deepmimic)
      restoreDeepmimicSettings(value);
  };
  watch(
    [() => selectedRound.value?.record.runId, initializationMode, baselineOpen, publicationOpen],
    ([, mode]) => {
      if (!baselineOpen.value && !publicationOpen.value && mode === "resume")
        restoreDeepmimicSettings(selectedRound.value?.progress ?? null);
    },
  );
  const openBaseline = () => {
    error.value = null;
    const sourceRound = selectedRound.value;
    Object.assign(baselineDefaults, {
      bundleId: bundleId.value,
      numEnvs: parameters.numEnvs,
      iterations: parameters.iterations,
      seed: parameters.seed,
    });
    Object.assign(baselineSettings, baselineDefaults);
    histories.baseline.experimentId = sourceRound?.record.experimentId ?? "";
    histories.baseline.runId = sourceRound?.record.runId ?? "";
    baselineOpen.value = true;
  };
  watch(
    () => (baselineHistory.value ? selectedRound.value?.progress?.metadata.deepmimic : undefined),
    (source) => {
      if (!baselineHistory.value) return;
      Object.assign(
        baselineSettings,
        source
          ? {
              bundleId: source.bundleId,
              numEnvs: String(source.numEnvs),
              iterations: String(source.iterations),
              seed: String(source.seed),
            }
          : baselineDefaults,
      );
    },
  );
  const refreshBundles = async () => {
    const epoch = ++bundleEpoch;
    loadingBundles.value = true;
    bundleError.value = null;
    try {
      const result = await commands.deepmimicBundles(null);
      if (alive && epoch === bundleEpoch) {
        bundles.value = result;
        if (!bundleId.value && !rewardSourceLocked.value)
          bundleId.value = compatibleBundles.value[0]?.id ?? "";
      }
    } catch (reason) {
      if (alive && epoch === bundleEpoch)
        bundleError.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      if (alive && epoch === bundleEpoch) loadingBundles.value = false;
    }
  };
  const changeTask = (select: HTMLSelectElement) => {
    const next = selectValue(select, task.value);
    if (next !== deepmimicTask) {
      select.value = task.value;
    } else if (next !== task.value) {
      task.value = next;
      formEpoch++;
    }
  };

  const start = async () => {
    const machine = selectedMachine.value;
    if (busy.value || loadingRounds.value || (roundsError.value && !initializationChoice.value))
      return;
    if (!machine || !deepmimic.value) return;
    const asBaseline = baselineHistory.value;
    if (!asBaseline && initializationMode.value === "fresh" && isBaseline(newExperimentId.value)) {
      showMessage("ppo-baseline- 开头的训练 ID 用于基线对照。", { type: "error" });
      return;
    }
    if (!asBaseline && initializationMode.value === "fresh" && experimentConflict.value) {
      showMessage(experimentConflict.value, { type: "error" });
      return;
    }
    preparing.value = true;
    try {
      error.value = null;
      const parent = selectedRound.value;
      const checkpoint = selectedCheckpoint.value;
      const mode = asBaseline ? "fresh" : initializationMode.value;
      if (mode !== "fresh" && (!parent || !checkpoint))
        throw new Error("所选任务没有可用的历史保存点");
      const args = ["run", "train", task.value.trim()];
      if (!deepmimic.value) {
        if (terrain.value) args.push("--terrain", terrain.value);
        if (parameters.numEnvs) {
          const count = Number(parameters.numEnvs);
          if (!Number.isSafeInteger(count) || count < 1) throw new Error("并行环境数须为正整数");
          args.push("--num-envs", String(count));
        }
        if (!domainRandomization.value) args.push("--domain-randomization", "off");
      }
      let reference: Parameters<typeof commands.trainingStart>[0]["deepmimic"] = null;
      const bundle = asBaseline ? selectedBaselineBundle.value : selectedBundle.value;
      if (deepmimic.value) {
        const count = Number(asBaseline ? baselineSettings.numEnvs : parameters.numEnvs);
        const iterationCount = Number(
          asBaseline ? baselineSettings.iterations : parameters.iterations,
        );
        const randomSeed = Number(asBaseline ? baselineSettings.seed : parameters.seed);
        if (!supportsDeepmimic.value || !bundle)
          throw new Error("DeepMimic 需要当前模拟机为 official XL330，并选择匹配的动作包");
        if (
          !Number.isSafeInteger(count) ||
          count < 1 ||
          !Number.isSafeInteger(iterationCount) ||
          iterationCount < 1 ||
          !Number.isSafeInteger(randomSeed) ||
          randomSeed < 0 ||
          randomSeed > 2147483647
        )
          throw new Error("环境数和迭代数须为正整数，种子须为 0 到 2147483647");
        reference = {
          bundleId: bundle.id,
          numEnvs: count,
          iterations: iterationCount,
          seed: randomSeed,
          baseline: asBaseline,
        };
      }
      const started = await commands.trainingStart({
        initialization:
          mode !== "fresh" && parent && checkpoint
            ? {
                mode,
                parentRunId: parent.record.runId,
                checkpoint: checkpoint.id,
                resetNormalization: mode === "adapt" && resetNormalization.value,
              }
            : null,
        program: "uv",
        args: reference ? [] : args,
        virtualMachineId: machine.id,
        policyId: reference ? null : policyId.value || null,
        experimentId:
          asBaseline && bundle
            ? uniqueExperimentId(
                `${baselineMethod}-${teacherSlug(bundle.recording_id, bundle.name)}`,
                usedExperimentIds.value,
              )
            : mode === "fresh"
              ? newExperimentId.value
              : selectedExperimentId.value,
        replaceExistingExperiment:
          !asBaseline && mode === "fresh" && canReplaceExperiment.value && replaceExperiment.value,
        recipe: reference ? (asBaseline ? baselineMethod : "deepmimic") : recipe.value.trim(),
        teacherMotionProvenance: reference
          ? `deepmimic-bundle:${reference.bundleId}`
          : teacherMotionProvenance.value.trim(),
        datasetPath: null,
        deepmimic: reference,
      });
      if (asBaseline) {
        histories.baseline.experimentId = started.metadata.experimentId;
        histories.baseline.runId = started.runId;
      }
      if (store.state.training?.runId !== started.runId) store.applyTrainingProgress(started);
      await refreshRounds();
    } catch (reason) {
      const message = reason instanceof Error ? reason.message : String(reason);
      if (message.startsWith("训练 ID 已存在")) showMessage(message, { type: "error" });
      else error.value = message;
    } finally {
      preparing.value = false;
    }
  };

  const stop = async () => {
    if (stopping.value) return;
    stopping.value = true;
    try {
      error.value = null;
      await commands.trainingStop();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      stopping.value = false;
    }
  };

  const openResult = async (runId: string) => {
    try {
      await commands.trainingOpenResult(runId);
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const evaluateTeacher = async () => {
    const machine = selectedMachine.value;
    if (busy.value || !machine) return;
    preparing.value = true;
    error.value = null;
    try {
      const steps = Number(evaluationSteps.value);
      const evaluationRandomSeed = Number(evaluationSeed.value);
      if (
        !Number.isSafeInteger(steps) ||
        steps < 1 ||
        steps > 5000 ||
        !Number.isSafeInteger(evaluationRandomSeed) ||
        evaluationRandomSeed < 0 ||
        evaluationRandomSeed > 2147483647
      ) {
        throw new Error("评估步数须为 1 到 5000，种子须为非负整数");
      }
      const started = await commands.trainingEvaluateTeacher({
        virtualMachineId: machine.id,
        teacherPolicyId: teacherPolicyId.value,
        taskId: task.value.trim(),
        steps,
        seed: evaluationRandomSeed,
      });
      if (store.state.training?.runId !== started.runId) store.applyTrainingProgress(started);
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      preparing.value = false;
    }
  };

  const refreshRounds = async (background = false) => {
    const epoch = ++roundsEpoch;
    if (!background) loadingRounds.value = true;
    roundsError.value = null;
    try {
      const result = await commands.trainingRounds();
      if (alive && epoch === roundsEpoch) rounds.value = result;
    } catch (reason) {
      if (alive && epoch === roundsEpoch)
        roundsError.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      if (alive && epoch === roundsEpoch) loadingRounds.value = false;
    }
  };
  const refreshPublicationTargets = async () => {
    const epoch = ++publicationTargetsEpoch;
    loadingPublicationTargets.value = true;
    try {
      const result = await commands.trainingPublicationTargets();
      if (alive && epoch === publicationTargetsEpoch) publicationTargets.value = result;
    } catch (reason) {
      if (alive && epoch === publicationTargetsEpoch && publicationOpen.value)
        error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      if (alive && epoch === publicationTargetsEpoch) loadingPublicationTargets.value = false;
    }
  };
  const openPublication = () => {
    if (!canOpenPublication.value || !selectedRound.value) return;
    error.value = null;
    publicationName.value = null;
    publicationTargetId.value = "";
    publicationMessage.value = "";
    histories.publication.experimentId = selectedRound.value.record.experimentId;
    histories.publication.runId = selectedRound.value.record.runId;
    publicationOpen.value = true;
    void refreshPublicationTargets();
  };
  const roundPolling = useIntervalFn(() => void refreshRounds(true), 5000, { immediate: false });
  watch(
    () => publicationOpen.value && running.value,
    (active) => {
      if (active) {
        void refreshRounds(true);
        roundPolling.resume();
      } else {
        roundPolling.pause();
      }
    },
  );
  const clearCandidatePreview = () => {
    previewEpoch++;
    playback.pause();
    playbackStarted = undefined;
    candidateIndex.value = 0;
    candidatePreview.value = undefined;
    previewing.value = false;
  };
  const previewCandidate = async () => {
    const runId = selectedRound.value?.record.runId;
    const checkpointId = selectedCheckpoint.value?.id;
    const virtualMachineId = selectedMachine.value?.id;
    if (!runId || !checkpointId || !virtualMachineId || previewing.value) return;
    clearCandidatePreview();
    const epoch = previewEpoch;
    previewing.value = true;
    error.value = null;
    try {
      const preview = await commands.trainingPreview({ runId, checkpointId, virtualMachineId });
      if (alive && epoch === previewEpoch) {
        candidatePreview.value = preview;
        if (preview.frames.length > 1) playback.resume();
      }
    } catch (reason) {
      if (alive && epoch === previewEpoch)
        error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      if (alive && epoch === previewEpoch) previewing.value = false;
    }
  };
  watch(
    [
      publicationOpen,
      () => selectedRound.value?.record.runId,
      () => selectedCheckpoint.value?.id,
      () => selectedMachine.value?.id,
    ],
    clearCandidatePreview,
  );
  const publish = async () => {
    const parent = selectedRound.value;
    const checkpoint = selectedCheckpoint.value;
    const name = chosenPublicationName.value;
    const appendTo = publicationTargetId.value || null;
    if (publishing.value || parent?.progress?.phase !== "finished" || !checkpoint || !name) return;
    publishing.value = true;
    error.value = null;
    publicationMessage.value = "";
    try {
      store.state.policies = await commands.policyList();
      if (hasPublishedPolicy(name)) return;
      const policy = await commands.trainingPublish(
        parent.record.runId,
        checkpoint.id,
        name,
        appendTo,
      );
      store.state.policies = await commands.policyList();
      publicationMessage.value = `已发布：${policy.name}`;
      void refreshPublicationTargets();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      publishing.value = false;
    }
  };
  watch(
    () => selectedRound.value?.record.runId,
    () => {
      if (!publicationOpen.value) return;
      publicationName.value = null;
      publicationMessage.value = "";
      error.value = null;
    },
  );
  watch(
    () => progress.value?.phase,
    (phase) => {
      if (phase && phase !== "starting" && phase !== "running") void refreshRounds();
    },
  );

  onMounted(() => {
    void refreshRounds();
    policyId.value = store.state.selectedPolicyId ?? "";
    restore(progress.value);
    void refreshBundles();
    const epoch = formEpoch;
    void commands
      .trainingStatus()
      .then((value) => {
        if (!alive) return;
        if (!store.state.training) {
          store.applyTrainingProgress(value);
          if (epoch === formEpoch) restore(value);
        }
      })
      .catch(() => undefined);
  });
  const renderHistoryFields = (disabled: boolean) => (
    <>
      {(experimentIds.value.length > 0 || baselineHistory.value) && (
        <label>
          <span>训练历史</span>
          <Select
            value={selectedExperimentId.value}
            disabled={disabled || loadingRounds.value}
            onSelectionChange={(select) => {
              const next = baselineHistory.value
                ? select.value
                : selectValue(select, selectedExperimentId.value);
              if (next === selectedExperimentId.value) return;
              currentHistory.value.experimentId = next;
              currentHistory.value.runId = "";
              formEpoch++;
            }}
          >
            {baselineHistory.value && <option value="">新建基线</option>}
            {experimentIds.value.map((id) => (
              <option key={id} value={id}>
                {id}
              </option>
            ))}
          </Select>
        </label>
      )}
      <label>
        <span>历史轮次（继承或发布）</span>
        <Select
          value={selectedRound.value?.record.runId ?? ""}
          disabled={disabled || !historyRounds.value.length}
          onSelectionChange={(select) => {
            const previous = selectedRound.value?.record.runId ?? "";
            const next = selectValue(select, previous);
            if (next === previous) return;
            currentHistory.value.runId = next;
            formEpoch++;
          }}
        >
          {!historyRounds.value.length && (
            <option value="">
              {loadingRounds.value
                ? "正在读取训练历史…"
                : roundsError.value
                  ? "读取训练历史失败"
                  : baselineHistory.value
                    ? "未选择历史轮次"
                    : "当前模拟机没有匹配的历史轮次"}
            </option>
          )}
          {historyRounds.value.map((round) => (
            <option key={round.record.runId} value={round.record.runId}>
              第 {round.record.round} 轮 ·{" "}
              {baselineHistory.value &&
                (isBaseline(
                  round.record.experimentId,
                  round.progress?.metadata.deepmimic?.trainingMethod,
                )
                  ? "PPO 基线 · "
                  : "普通训练 · ")}
              {(progress.value?.runId === round.record.runId
                ? progress.value.phase
                : round.progress?.phase) ?? "未启动"}
            </option>
          ))}
        </Select>
      </label>
    </>
  );

  const renderRunData = (value: NonNullable<typeof progress.value>) => (
    <>
      <div class={style.metrics}>
        <span>
          step {value.step ?? "—"}
          {value.totalSteps != null && ` / ${value.totalSteps}`}
        </span>
        <span>reward {value.reward?.toFixed(4) ?? "—"}</span>
        <span>loss {value.loss?.toFixed(4) ?? "—"}</span>
      </div>
      <div class={style.metadata}>
        <span>run {value.runId}</span>
        <span>
          {value.metadata.bodyTemplateId} v{value.metadata.bodyVersion} ·{" "}
          {value.metadata.servoProfile}
        </span>
        <span>
          {value.metadata.policyLayout.obsLen}/{value.metadata.policyLayout.actionLen} ·{" "}
          {value.metadata.policyLayout.controlHz} Hz
        </span>
        <span>{value.metadata.recipe}</span>
      </div>
    </>
  );

  const renderIteration = (source: TrainingProvenance, round?: number) => (
    <div class={style.metadata} key={round}>
      {round !== undefined && <span>第 {round} 轮</span>}
      <span>动作包：{rewardSourceName(source.bundleId)}</span>
      <span>seed {source.seed}</span>
      <span>
        {source.numEnvs} 环境 × {source.iterations} 次迭代
      </span>
    </div>
  );

  const renderProgress = (value: NonNullable<typeof progress.value>) => (
    <div class={style.progressCard}>
      <div class={style.progressHeader}>
        <strong>{value.message || "等待训练输出…"}</strong>
        {value.percent != null && <span>{value.percent.toFixed(1)}%</span>}
      </div>
      {renderRunData(value)}
      {value.metadata.deepmimic && renderIteration(value.metadata.deepmimic)}
      <small class={style.resultDir}>结果目录：{value.resultDir}</small>
      <button type="button" onClick={() => void openResult(value.runId)}>
        打开结果目录
      </button>
      {value.line && <pre class={style.runOutput}>{value.line}</pre>}
    </div>
  );

  const renderPublicationPage = () => {
    const round = selectedRound.value;
    const value = progress.value?.runId === round?.record.runId ? progress.value : round?.progress;
    return (
      <div class={style.content}>
        <header class={style.header}>
          <div class={style.publicationActions}>
            <button
              type="button"
              disabled={previewing.value || !selectedCheckpoint.value || !selectedMachine.value}
              onClick={() => void previewCandidate()}
            >
              {previewing.value ? "预览中…" : "预览"}
            </button>
            <button
              type="button"
              disabled={
                publishing.value || !canOpenPublication.value || publicationNameExists.value
              }
              onClick={() => void publish()}
            >
              {publishing.value ? "发布中…" : "发布"}
            </button>
          </div>
          <div class={style.publicationActions}>
            <button
              type="button"
              disabled={!round}
              onClick={() => round && void openResult(round.record.runId)}
            >
              打开记录
            </button>
            <button
              type="button"
              disabled={publishing.value}
              onClick={() => {
                publicationOpen.value = false;
                error.value = null;
              }}
            >
              返回训练
            </button>
          </div>
        </header>
        <div class={style.form}>
          {renderHistoryFields(publishing.value)}
          <label>
            <span>追加到</span>
            <Select
              value={publicationTargetId.value}
              disabled={publishing.value || loadingPublicationTargets.value}
              onSelectionChange={(select) => (publicationTargetId.value = select.value)}
            >
              <option value="">新建发布</option>
              {compatiblePublicationTargets.value.map((target) => (
                <option key={target.id} value={target.id}>
                  {target.name}
                </option>
              ))}
            </Select>
          </label>
          <label>
            <span>策略名称</span>
            <input
              value={publicationName.value ?? defaultPublicationName.value}
              placeholder={defaultPublicationName.value}
              disabled={publishing.value}
              onInput={(event) => (publicationName.value = inputValue(event))}
            />
          </label>
        </div>
        {publicationNameExists.value && <p class={style.error}>已有同名策略，请修改策略名称。</p>}
        {previewSummary.value && (
          <p role="status" class={style.metadata}>
            {previewSummary.value}
          </p>
        )}
        {value && renderRunData(value)}
        {round &&
          historyRounds.value.toReversed().map(({ record, progress: roundProgress }) => {
            const source = roundProgress?.metadata.deepmimic;
            return record.round <= round.record.round && source
              ? renderIteration(source, record.round)
              : null;
          })}
        {value?.line && <pre class={style.runOutput}>{value.line}</pre>}
        {publicationMessage.value && <p role="status">{publicationMessage.value}</p>}
        {error.value && <p class={style.error}>{error.value}</p>}
      </div>
    );
  };

  const renderParameters = (fields: typeof parameters) => (
    <>
      <label>
        <span>并行环境数</span>
        <input
          type="number"
          min="1"
          step="1"
          value={fields.numEnvs}
          disabled={busy.value}
          onInput={(event) => (fields.numEnvs = inputValue(event))}
        />
      </label>
      <label>
        <span>迭代数</span>
        <input
          type="number"
          min="1"
          step="1"
          value={fields.iterations}
          disabled={busy.value}
          onInput={(event) => (fields.iterations = inputValue(event))}
        />
      </label>
      <label>
        <span>随机种子</span>
        <input
          type="number"
          min="0"
          max="2147483647"
          step="1"
          value={fields.seed}
          disabled={busy.value}
          onInput={(event) => (fields.seed = inputValue(event))}
        />
      </label>
    </>
  );

  const renderTrainButton = (disabled: boolean) => (
    <div class={style.actions}>
      <button
        type="button"
        disabled={stopping.value || (!running.value && (busy.value || disabled))}
        onClick={() => void (running.value ? stop() : start())}
      >
        {stopping.value ? "停止中…" : running.value ? "停止" : "开始训练"}
      </button>
    </div>
  );

  const renderCurrentProgress = (asBaseline: boolean) => {
    const current = progress.value;
    if (!current || current.phase === "finished") return null;
    const baseline = isBaseline(
      current.metadata.experimentId,
      current.metadata.deepmimic?.trainingMethod,
    );
    return baseline === asBaseline ? renderProgress(current) : null;
  };

  const renderBaselinePage = () => (
    <div class={style.content}>
      <header class={style.header}>
        <h2>基线对照</h2>
        <div class={style.publicationActions}>
          <button type="button" disabled={!canOpenPublication.value} onClick={openPublication}>
            策略发布
          </button>
          <button
            type="button"
            onClick={() => {
              baselineOpen.value = false;
              error.value = null;
            }}
          >
            返回训练
          </button>
        </div>
      </header>
      <div class={style.form}>
        {renderHistoryFields(preparing.value)}
        <small class={style.metadata}>
          当前动作：
          {baselineSettings.bundleId ? rewardSourceName(baselineSettings.bundleId) : "未选择"}
        </small>
        <Checkbox label="PPO 基线" stacked checked disabled />
        {renderParameters(baselineSettings)}
        {renderTrainButton(
          loadingRounds.value || !supportsDeepmimic.value || !selectedBaselineBundle.value,
        )}
      </div>
      {error.value && <p class={style.error}>{error.value}</p>}
      {renderCurrentProgress(true)}
    </div>
  );

  const renderTrainingPage = () => (
    <div class={style.content}>
      <header class={style.header}>
        <h2>{selectedMachine.value?.name ?? "未选择模拟机"}</h2>
        <div class={style.publicationActions}>
          <button
            type="button"
            disabled={!deepmimic.value || loadingBundles.value || loadingRounds.value}
            onClick={openBaseline}
          >
            基线对照
          </button>
          <button type="button" disabled={!canOpenPublication.value} onClick={openPublication}>
            策略发布
          </button>
        </div>
      </header>

      <div class={style.form}>
        <label title={taskHint.value}>
          <span>任务</span>
          <Select
            value={task.value}
            disabled={busy.value || !!taskHint.value}
            onSelectionChange={changeTask}
          >
            <option value="">
              {store.state.trainingTasksError
                ? "任务读取失败"
                : store.state.trainingTasks === null
                  ? "正在读取任务…"
                  : "DeepMimic 任务未注册"}
            </option>
            {tasks.value.map((id) => (
              <option key={id} value={id} disabled={id !== deepmimicTask}>
                {id}
              </option>
            ))}
          </Select>
        </label>
        {renderHistoryFields(preparing.value)}
        <label>
          <span>训练规则</span>
          <Select
            value={initializationMode.value}
            disabled={busy.value}
            onSelectionChange={(select) => {
              const value = selectValue(select, initializationMode.value);
              if (value === "fresh" || value === "resume" || value === "adapt") {
                initializationChoice.value = value;
                formEpoch++;
              }
            }}
          >
            <option value="fresh">从头训练</option>
            <option value="resume" disabled={!selectedCheckpoint.value}>
              {previousRewardSource.value
                ? `继续训练（${previousRewardSource.value}）`
                : "继续训练"}
            </option>
            <option value="adapt" disabled={!selectedCheckpoint.value}>
              {deepmimic.value
                ? "继续训练（切换奖励来源）"
                : "适配（继承 actor，重置 critic 和优化器）"}
            </option>
          </Select>
        </label>
        {deepmimic.value && (
          <>
            <label>
              <span>奖励来源</span>
              <Select
                value={bundleId.value}
                disabled={busy.value || loadingBundles.value || rewardSourceLocked.value}
                onSelectionChange={(select) => {
                  if (!rewardSourceLocked.value)
                    bundleId.value = selectValue(select, bundleId.value);
                }}
              >
                <option value="">请选择已发布的动作包</option>
                {rewardSourceLocked.value && bundleId.value && !selectedBundle.value && (
                  <option value={bundleId.value}>{bundleId.value}</option>
                )}
                {compatibleBundles.value.map((bundle) => (
                  <option key={bundle.id} value={bundle.id}>
                    {bundle.name} · {bundle.frames} 帧 · {bundle.duration_s?.toFixed(2) ?? "—"} s ·{" "}
                    {bundle.playback}
                  </option>
                ))}
              </Select>
            </label>
            <button
              type="button"
              disabled={loadingBundles.value || busy.value}
              onClick={() => void refreshBundles()}
            >
              刷新动作包
            </button>
            {bundleError.value && <p class={style.error}>{bundleError.value}</p>}
            {!loadingBundles.value && !compatibleBundles.value.length && (
              <p>暂无匹配的动作包，请先在日志中转换 XL330 录制。</p>
            )}
          </>
        )}
        {initializationMode.value === "adapt" && (
          <Checkbox
            label="重置观测归一化统计"
            stacked
            checked={resetNormalization.value}
            disabled={busy.value}
            onChange={(checked) => (resetNormalization.value = checked)}
          />
        )}
        {roundsError.value && <p role="alert">读取训练历史失败：{roundsError.value}</p>}
        {!deepmimic.value && (
          <>
            <label>
              <span>参考策略（可选）</span>
              <Select
                value={policyId.value}
                disabled={busy.value}
                onSelectionChange={(select) => (policyId.value = select.value)}
              >
                <option value="">使用固定 61/14/50 布局</option>
                {policies.value.map((policy) => (
                  <option value={policy.id} key={policy.id}>
                    {policy.name} · {policy.cached ? "已缓存" : "未缓存"}
                  </option>
                ))}
              </Select>
            </label>
          </>
        )}
        {!deepmimic.value && (
          <>
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
          </>
        )}
        {!deepmimic.value && (
          <>
            <label>
              <span>任务场地</span>
              <Select
                value={terrain.value}
                disabled={busy.value}
                onSelectionChange={(select) => (terrain.value = select.value)}
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
                value={parameters.numEnvs}
                disabled={busy.value}
                onInput={(event) => (parameters.numEnvs = inputValue(event))}
              />
            </label>
            <Checkbox
              label="域随机化"
              stacked
              checked={domainRandomization.value}
              disabled={busy.value}
              onChange={(checked) => (domainRandomization.value = checked)}
            />
          </>
        )}
        {deepmimic.value && (
          <>
            {renderParameters(parameters)}
            {selectedMachine.value && !supportsDeepmimic.value && (
              <p>DeepMimic 需要 official XL330 模拟机。</p>
            )}
          </>
        )}
        {initializationMode.value === "fresh" && (
          <div class={style.experimentName}>
            <label>
              <span>训练 ID</span>
              <input
                value={enteredExperimentId.value ?? defaultExperimentId.value}
                placeholder={defaultExperimentId.value}
                disabled={busy.value || loadingRounds.value}
                onInput={(event) => {
                  enteredExperimentId.value = inputValue(event);
                  formEpoch++;
                }}
              />
            </label>
            <Checkbox
              class={style.replaceExperiment}
              label="删除同名训练"
              stacked
              checked={replaceExperiment.value}
              disabled={busy.value || loadingRounds.value}
              onChange={(checked) => (replaceExperiment.value = checked)}
            />
          </div>
        )}
        {renderTrainButton(
          loadingRounds.value ||
            (!!roundsError.value && !initializationChoice.value) ||
            (initializationMode.value !== "fresh" && !selectedCheckpoint.value) ||
            !deepmimic.value ||
            !supportsDeepmimic.value ||
            !selectedBundle.value,
        )}
        <TaskConfiguration taskId={task.value} />
      </div>

      {task.value && !deepmimic.value && teacherPolicies.value.length > 0 && (
        <section class={style.form} aria-label="教师仿真评估">
          <label>
            <span>教师产物</span>
            <Select
              value={teacherPolicyId.value}
              disabled={busy.value}
              onSelectionChange={(select) =>
                (teacherPolicyId.value = selectValue(select, teacherPolicyId.value))
              }
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
                busy.value || !teacherPolicyId.value || !selectedMachine.value || !task.value.trim()
              }
              onClick={() => void evaluateTeacher()}
            >
              评估所选教师产物
            </button>
          </div>
        </section>
      )}

      {error.value && <p class={style.error}>{error.value}</p>}
      {renderCurrentProgress(false)}
    </div>
  );

  return () => (
    <>
      <SceneStage training={candidateFrame.value ?? store.state.trainingFrame} />
      <section
        class={style.trainingPage}
        aria-label={
          publicationOpen.value ? "策略发布" : baselineOpen.value ? "基线对照" : "本地训练"
        }
      >
        <ScrollArea
          fill
          key={publicationOpen.value ? "publication" : baselineOpen.value ? "baseline" : "training"}
        >
          {publicationOpen.value
            ? renderPublicationPage()
            : baselineOpen.value
              ? renderBaselinePage()
              : renderTrainingPage()}
        </ScrollArea>
      </section>
    </>
  );
});
