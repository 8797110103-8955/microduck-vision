import { sendRuntimeCommand } from "../../runtime-command";
import * as commands from "../../../generated/types";
import {
  computed,
  defineComponent,
  onMounted,
  onScopeDispose,
  ref,
  shallowRef,
  toRef,
  watch,
} from "vue";
import { useEventListener } from "@vueuse/core";
import type { LoopStatus, PolicyEntry, PreviewInput } from "../../../generated/types";
import { ScrollArea } from "../../components/scroll-area";
import { Select } from "../../components/select";
import { Slider } from "../../components/slider";
import {
  focusDuckEvent,
  defaultPoseEvent,
  requestDuckFocus,
  restoreDuckViewEvent,
  requestServoSelection,
  servoSelectionEvent,
} from "../../components/viewer-events";
import { persistWorldCache, pruneWorldCache, useDesktopStore } from "../../store";
import {
  DebugPage,
  debugBindingFor,
  debugBindingMachineFor,
  debugPhysicalMachineFor,
  load,
} from "../debug";
import { TrainingPage } from "../training";
import { DuckViewer } from "./viewer";
import style from "./style.module.scss";

const JOINT_COUNT = 15;
const officialJointNames = [
  "left_hip_yaw",
  "left_hip_roll",
  "left_hip_pitch",
  "left_knee",
  "left_ankle",
  "neck_pitch",
  "head_pitch",
  "head_yaw",
  "head_roll",
  "mouth",
  "right_hip_yaw",
  "right_hip_roll",
  "right_hip_pitch",
  "right_knee",
  "right_ankle",
];

/**
 * Right edge of the official jaw.stl mesh at the mouth seam for the default
 * camera after its MJCF geom transform, expressed in the jaw_soft body frame.
 * The view's horizontal axis combines model x/y, so this is intentionally the
 * right-side x/y point with the lower seam z value.
 */
const mouthMarkerRightEdge: [number, number, number] = [0.0119914336, 0.04442375898, -0.0751049415];
const markerLabel = {
  online: "在线",
  warn: "告警",
  missing: "缺失",
  unknown: "未知",
};

export function physicalMarkerState(
  physicalId: number | null,
  onlineServoIds: number[],
  machinePresent: boolean,
  sensorFailures = 0,
) {
  if (!machinePresent) return "unknown" as const;
  if (physicalId == null || !onlineServoIds.includes(physicalId)) return "missing" as const;
  return sensorFailures ? ("warn" as const) : ("online" as const);
}

const isWalkingPolicy = (policy: PolicyEntry | undefined) =>
  policy?.slot === "walk" || /walking|velstand/i.test(policy?.file ?? "");

const inputForPolicy = (policy: PolicyEntry | undefined): PreviewInput => ({
  twist: isWalkingPolicy(policy) ? [0.15, 0, 0] : [0, 0, 0],
  head: [0, 0, 0, 0],
  body: [0, 0, 0],
  targetDistanceM: null,
});

export const OfficialDuckStage = defineComponent(() => {
  const store = useDesktopStore();
  const host = shallowRef<HTMLDivElement>();
  const viewer = shallowRef<DuckViewer>();
  const scene = ref<Awaited<ReturnType<typeof commands.officialSceneModel>> | null>(null);
  const frame = ref<Awaited<ReturnType<typeof commands.officialSceneFrame>> | null>(null);
  const debug = computed(() => !store.state.recordingPlayback && store.state.tab === "debug");
  const activeLoop = computed(() => store.state.recordingPlayback?.status ?? store.state.loop);
  const previewPhysicalId = computed(() =>
    !store.state.recordingPlayback && store.state.tab === "preview"
      ? store.state.requestedPhysicalMachine
      : null,
  );
  const displayingSimulation = computed(
    () =>
      !debug.value &&
      !previewPhysicalId.value &&
      !!activeLoop.value?.activeBody &&
      !activeLoop.value.activeBody.physicalMachineId,
  );
  const cachedWorld = computed(() => {
    const machineId = store.state.selectedVirtualMachine;
    return machineId ? store.state.worldCache[machineId] : undefined;
  });
  const displayWorld = computed(() => {
    const body = activeLoop.value?.activeBody;
    return body?.virtualMachineId &&
      (!debug.value || body.virtualMachineId === store.state.selectedVirtualMachine)
      ? activeLoop.value?.world
      : cachedWorld.value;
  });
  const displayFrame = computed(() =>
    displayingSimulation.value ? activeLoop.value?.simulation?.frame : undefined,
  );
  const debugSource = computed(() =>
    activeLoop.value?.readSources.find((source) => source.id === mappedMachine.value?.id),
  );
  const activePositions = computed(() => {
    const loop = activeLoop.value;
    if (!debug.value) {
      if (previewPhysicalId.value)
        return loop?.activeBody?.physicalMachineId === previewPhysicalId.value
          ? loop.positions
          : loop?.readSources.find((source) => source.id === previewPhysicalId.value)?.positions;
      return loop?.activeBody ? loop.positions : undefined;
    }
    if (store.state.debugMappingPending) return undefined;
    const physicalId = mappedMachine.value?.id;
    const physicalPositions =
      physicalId && loop?.activeBody?.physicalMachineId === physicalId
        ? loop.positions
        : debugSource.value?.positions;
    return store.state.rangePreview ?? physicalPositions ?? store.state.debugPose;
  });
  const frameContext = computed(() =>
    JSON.stringify([
      debug.value,
      previewPhysicalId.value,
      store.state.debugMappingPending,
      selectedMachine.value?.jointBindings,
      store.state.selectedVirtualMachine,
      activeLoop.value?.runtimeGeneration,
      activeLoop.value?.executionGeneration,
      activeLoop.value?.activeBody?.physicalMachineId,
      debugSource.value?.generation,
      mappedMachine.value?.jointBindings,
      mappedMachine.value?.jointDirections,
    ]),
  );
  const markerPos = ref<({ x: number; y: number } | null)[]>([]);
  const labelPos = ref<({ x: number; y: number } | null)[]>([]);
  const selectedServoId = ref<number | null>(null);
  let frameRequestInFlight = false;
  let diagnosedPoseContext: string | null = null;
  let pendingFramePositions: number[] | null = null;
  let stageActive = true;

  const mappedMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const rangeStateBySlot = computed(() => {
    const loop = activeLoop.value;
    const states =
      debug.value && loop?.activeBody?.physicalMachineId !== mappedMachine.value?.id
        ? []
        : (loop?.jointRangeStates ?? []);
    return new Map(states.map((state) => [state.slotId, state]));
  });
  watch(
    [() => store.state.virtualMachines.map((machine) => machine.id), () => store.state.loading],
    ([machineIds, loading]) => {
      if (loading || (!machineIds.length && !store.state.catalog.length)) return;
      pruneWorldCache(store.state.worldCache, machineIds);
    },
    { immediate: true },
  );
  /** Every virtual machine in the scene; the selected one is the live duck. */
  const sceneMachines = computed(() => store.state.virtualMachines);
  const selectedSceneIndex = computed(() =>
    Math.max(
      0,
      sceneMachines.value.findIndex(
        (machine) =>
          machine.id ===
          ((!debug.value && activeLoop.value?.activeBody?.virtualMachineId) ||
            store.state.selectedVirtualMachine),
      ),
    ),
  );
  const ghostCount = () => Math.max(sceneMachines.value.length - 1, 0);
  const worldPositions = computed(() =>
    sceneMachines.value.map((machine) => {
      const position = machine.simulation.worldPosition ?? [0, 0, 0];
      return [position[0] ?? 0, position[1] ?? 0, position[2] ?? 0] satisfies [
        number,
        number,
        number,
      ];
    }),
  );
  const worldPoses = computed(() =>
    sceneMachines.value.map((machine) => store.state.worldCache[machine.id]),
  );
  /** Every duck gets a label so selecting a machine remains visible in both views. */
  const labels = computed(() => {
    const body = activeLoop.value?.activeBody;
    const physical =
      !debug.value &&
      store.state.physicalMachines.find(
        (machine) => machine.id === (previewPhysicalId.value ?? body?.physicalMachineId),
      );
    return sceneMachines.value.map((machine) => ({
      id: machine.id,
      name: physical && machine.id === body?.virtualMachineId ? physical.name : machine.name,
    }));
  });
  const markers = computed(() =>
    officialJointNames.map((name, index) => ({
      name,
      body:
        scene.value?.model.joints.find((joint) => joint.name === name)?.body ??
        scene.value?.model.joints.find((joint) => joint.name === "head_roll")?.body ??
        0,
      index,
      id: index + 1,
      physicalId:
        debugBindingFor(
          debugBindingMachineFor(
            selectedMachine.value,
            debug.value
              ? mappedMachine.value
              : store.state.physicalMachines.find(
                  (machine) =>
                    machine.id ===
                    (previewPhysicalId.value ?? activeLoop.value?.activeBody?.physicalMachineId),
                ),
          ),
          name,
        )?.physicalId ?? null,
    })),
  );
  const markerState = (physicalId: number | null) => {
    const online = mappedMachine.value?.onlineServoIds ?? [];
    return physicalMarkerState(
      physicalId,
      online,
      mappedMachine.value != null,
      debugSource.value
        ? Number(debugSource.value.health === "offline")
        : (activeLoop.value?.sensorFailures ?? 0),
    );
  };

  async function refreshFrame(positions: number[]) {
    if (!scene.value || (debug.value && store.state.debugMappingPending)) return;
    const simulation = displayFrame.value;
    if (simulation) {
      pendingFramePositions = null;
      frame.value = simulation;
      const world = displayWorld.value;
      viewer.value?.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
      if (activeLoop.value?.simulation)
        viewer.value?.setSimulationFrame(activeLoop.value.simulation);
      return;
    }
    // Reopening a read source temporarily has no sample. Keep the last real pose;
    // synthesizing zero angles here makes the whole body jump during a direction edit.
    if (positions.length !== JOINT_COUNT) {
      pendingFramePositions = null;
      return;
    }
    if (frameRequestInFlight) {
      pendingFramePositions = [...positions];
      return;
    }
    frameRequestInFlight = true;
    try {
      const context = frameContext.value;
      const source = debugSource.value;
      const rawFrame = await commands.officialSceneFrame(positions);
      if (!stageActive || context !== frameContext.value || displayFrame.value) return;
      const markerFrame = officialJointNames.map(
        (_, index) => rawFrame.markers[index] ?? [0, 0, 0],
      );
      frame.value = { ...rawFrame, markers: markerFrame };
      viewer.value?.setFrame(frame.value);
      if (
        debug.value &&
        viewer.value &&
        source?.positions.length === JOINT_COUNT &&
        source.sequence > 0 &&
        diagnosedPoseContext !== context &&
        !store.state.rangePreview
      ) {
        diagnosedPoseContext = context;
        void commands
          .frontendDiagnostic(
            JSON.stringify({
              event: "debug_pose_applied",
              physicalMachineId: mappedMachine.value?.id,
              virtualMachineId: store.state.selectedVirtualMachine,
              runtimeGeneration: activeLoop.value?.runtimeGeneration,
              executionGeneration: activeLoop.value?.executionGeneration,
              source: "readOnlySource",
              sourceGeneration: source.generation,
              sequence: source.sequence,
              jointOrder: officialJointNames,
              positions,
              jointBindings: selectedMachine.value?.jointBindings,
              jointCalibration: mappedMachine.value?.jointBindings,
              jointDirections: mappedMachine.value?.jointDirections,
            }),
          )
          .catch((error) => console.warn("记录调试姿态失败", error));
      }
    } finally {
      frameRequestInFlight = false;
      if (stageActive && pendingFramePositions) {
        const nextPositions = pendingFramePositions;
        pendingFramePositions = null;
        void refreshFrame(nextPositions);
      }
    }
  }

  async function buildViewer() {
    if (!host.value || !scene.value) return;
    viewer.value?.dispose();
    viewer.value = new DuckViewer(host.value, scene.value);
    viewer.value.onRender = () => {
      const v = viewer.value;
      if (!v) return;
      if (debug.value) {
        markerPos.value = markers.value.map((marker, index) =>
          index === 9 ? v.markerScreenAt(marker.body, mouthMarkerRightEdge) : v.markerScreen(index),
        );
      }
      const rangeBodies = markers.value
        .filter((marker) => {
          const state = rangeStateBySlot.value.get(marker.name);
          return state?.belowMin || state?.aboveMax;
        })
        .map((marker) => marker.body);
      v.highlightBodies(rangeBodies);
      labelPos.value = labels.value.map((_, index) => v.duckScreen(index));
    };
    viewer.value.setGhosts(ghostCount(), 0.25, worldPositions.value);
    viewer.value.setMachineWorldPoses(worldPoses.value);
    viewer.value.setSelectedDuck(selectedSceneIndex.value);
    const world = displayWorld.value;
    viewer.value.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
    await refreshFrame(activePositions.value ?? []);
  }

  useEventListener(window, focusDuckEvent, focusCurrentDuck);
  useEventListener(window, defaultPoseEvent, restoreDefaultPose);
  useEventListener(window, restoreDuckViewEvent, restorePreviousDuckView);
  useEventListener(window, servoSelectionEvent, onServoSelection);

  onMounted(async () => {
    stageActive = true;
    scene.value = await commands.officialSceneModel();
    if (!stageActive) return;
    await buildViewer();
  });

  onScopeDispose(() => {
    stageActive = false;
    pendingFramePositions = null;
    viewer.value?.dispose();
  });

  function onServoSelection(event: Event) {
    if (!(event instanceof CustomEvent)) return;
    selectedServoId.value = typeof event.detail === "number" ? event.detail : null;
  }

  function focusCurrentDuck() {
    viewer.value?.focusCurrentDuck();
  }

  function restorePreviousDuckView() {
    viewer.value?.restorePreviousView();
  }

  function restoreDefaultPose() {
    if (!scene.value) return;
    pendingFramePositions = null;
    frame.value = scene.value.defaultFrame;
    viewer.value?.setFrame(scene.value.defaultFrame);
  }

  function selectSceneDuck(event: MouseEvent) {
    const index = viewer.value?.pickDuck(event.clientX, event.clientY);
    if (index == null) return;
    const machine = sceneMachines.value[index];
    if (!machine) return;
    store.state.selectedVirtualMachine = machine.id;
    store.state.selectedVirtualMachineManual = true;
  }

  watch([frameContext, activePositions, displayFrame], ([, positions]) => {
    void refreshFrame(positions ?? []);
  });
  watch(
    () => displayWorld.value,
    (world) => {
      viewer.value?.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
    },
    { deep: true },
  );
  watch(
    [
      () => debug.value,
      () => sceneMachines.value,
      () => worldPositions.value,
      () => worldPoses.value,
      () => selectedSceneIndex.value,
    ],
    () => {
      viewer.value?.setGhosts(ghostCount(), 0.25, worldPositions.value);
      viewer.value?.setMachineWorldPoses(worldPoses.value);
      viewer.value?.setSelectedDuck(selectedSceneIndex.value);
    },
    { deep: true },
  );

  return () => (
    <section class={style.officialDuckStage}>
      <div class={style.stageBar}>
        <div class={style.seg}>
          <button
            class={store.state.tab === "preview" ? style.on : undefined}
            onClick={() => (store.state.tab = store.state.tab === "preview" ? "home" : "preview")}
          >
            VLA
          </button>
          <button
            class={store.state.tab === "training" ? style.on : undefined}
            onClick={() => (store.state.tab = store.state.tab === "training" ? "home" : "training")}
          >
            训练
          </button>
          <button
            class={debug.value ? style.on : undefined}
            onClick={() => (store.state.tab = debug.value ? "home" : "debug")}
          >
            调试
          </button>
        </div>
      </div>
      <div class={style.stageBody}>
        <div
          data-testid="duck-viewport"
          class={style.viewport}
          ref={host}
          onClick={selectSceneDuck}
        >
          {labels.value.map((label, index) => {
            const position = labelPos.value[index];
            if (!position) return null;
            return (
              <span
                key={label.id}
                class={style.duckLabel}
                style={{ left: `${position.x}px`, top: `${position.y}px` }}
              >
                {label.name}
              </span>
            );
          })}
          {debug.value &&
            markers.value.map((marker, index) => {
              const position = markerPos.value[index];
              if (!position) return null;
              const state = markerState(marker.physicalId);
              return (
                <button
                  key={marker.name}
                  class={[
                    style.marker,
                    style[state],
                    selectedServoId.value === marker.id ? style.active : undefined,
                  ]}
                  style={{ left: `${position.x}px`, top: `${position.y}px` }}
                  title={`${marker.name} · ${marker.physicalId == null ? "未匹配" : `ID ${marker.physicalId}`} · ${markerLabel[state]}`}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    requestServoSelection(selectedServoId.value === marker.id ? null : marker.id);
                  }}
                >
                  {marker.physicalId ?? "—"}
                </button>
              );
            })}
        </div>
      </div>
    </section>
  );
});

export const PreviewPage = defineComponent(() => {
  const store = useDesktopStore();
  const busy = ref<string | null>(null);
  const error = ref<string | null>(null);
  const loopBusy = ref(false);
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const physicalMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const physicalSyncAvailable = computed(() => physicalMachine.value?.connected === true);
  const worldAtConfiguredPosition = computed(() => {
    const machineId = selectedMachine.value?.id;
    const activeWorld = runtime.value?.world;
    const world = activeWorld ?? (machineId ? store.state.worldCache[machineId] : undefined);
    if (!world) return true;
    const epsilon = 1e-6;
    return (
      Math.abs(world.position[0] ?? 0) <= epsilon &&
      Math.abs(world.position[1] ?? 0) <= epsilon &&
      Math.abs(world.yaw ?? 0) <= epsilon &&
      Math.abs(world.distanceM ?? 0) <= epsilon
    );
  });
  const cachedPolicies = computed(() => store.state.policies.filter((policy) => policy.cached));
  const selectedPolicy = computed(() =>
    cachedPolicies.value.find((policy) => policy.id === store.state.selectedPolicyId),
  );
  const previewInput = toRef(store.state, "previewInput");
  const runtime = computed(() =>
    store.state.loop?.activeBody?.virtualMachineId ||
    (store.state.loop?.runtimeState === "faulted" && store.state.loop.hasExecution)
      ? store.state.loop
      : null,
  );
  const detached = computed(
    () => runtime.value?.runtimeState === "faulted" && !runtime.value.activeBody,
  );
  const physicalSyncEnabled = computed(
    () => !!store.state.requestedPhysicalMachine || !!runtime.value?.activeBody?.physicalMachineId,
  );
  const torqueEnabled = computed(
    () => !!runtime.value?.activeBody?.physicalMachineId && runtime.value.torque,
  );
  const controlRunning = computed(() => runtime.value?.phase === "driving");

  const updateWorldStatus = (status: LoopStatus) => {
    store.applyLoopStatus(status);
  };

  const reportLoopError = async (reason: unknown) => {
    error.value = reason instanceof Error ? reason.message : String(reason);
    const status = await commands.loopStatus().catch(() => undefined);
    if (status) store.applyLoopStatus(status);
    else if (status === null) store.state.loop = null;
  };

  const switchBody = async (loop: LoopStatus, virtualId: string, physicalId: string | null) => {
    updateWorldStatus(
      await commands.runtimeSelectBody(
        loop.runtimeGeneration,
        loop.executionGeneration,
        virtualId,
        physicalId,
      ),
    );
  };

  const updatePreviewInput = async () => {
    const loop = runtime.value;
    if (!loop || loopBusy.value || !controlRunning.value) return;
    try {
      updateWorldStatus(
        await sendRuntimeCommand(loop.runtimeGeneration, loop.executionGeneration, {
          kind: "input",
          input: previewInput.value,
        }),
      );
      error.value = null;
    } catch (reason) {
      if (runtime.value?.executionGeneration === loop.executionGeneration)
        await reportLoopError(reason);
    }
  };

  const openLogs = async () => {
    try {
      await commands.openLogs();
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const resetWorld = async () => {
    const machineId = runtime.value?.activeBody?.virtualMachineId ?? selectedMachine.value?.id;
    if (!machineId || physicalSyncEnabled.value) return;
    try {
      const loop = runtime.value;
      if (loop)
        store.applyLoopStatus(
          await sendRuntimeCommand(loop.runtimeGeneration, loop.executionGeneration, {
            kind: "resetWorld",
          }),
        );
      delete store.state.worldCache[machineId];
      persistWorldCache(store.state.worldCache);
      error.value = null;
    } catch (reason) {
      await reportLoopError(reason);
    }
  };

  const updateHead = (index: number, value: number) => {
    const current = previewInput.value.head;
    const head: [number, number, number, number] = [
      current[0] ?? 0,
      current[1] ?? 0,
      current[2] ?? 0,
      current[3] ?? 0,
    ];
    head[index] = value;
    previewInput.value = { ...previewInput.value, head };
  };

  const updateBody = (index: number, value: number) => {
    const current = previewInput.value.body;
    const body: [number, number, number] = [current[0] ?? 0, current[1] ?? 0, current[2] ?? 0];
    body[index] = value;
    previewInput.value = { ...previewInput.value, body };
  };

  const refresh = async () => {
    try {
      store.state.policies = await commands.policyList();
      store.state.selectedPolicyId =
        store.state.policies.find((policy) => policy.selected)?.id ?? null;
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const applyPolicy = async (loop: LoopStatus, id: string) => {
    const status = await sendRuntimeCommand(loop.runtimeGeneration, loop.executionGeneration, {
      kind: "policy",
      policy_id: id,
      input: previewInput.value,
    });
    updateWorldStatus(status);
  };

  const selectPolicy = async (id: string) => {
    if (loopBusy.value || busy.value) return;
    busy.value = id;
    try {
      const selected = await commands.policySelect(id);
      store.state.selectedPolicyId = selected.id;
      previewInput.value = inputForPolicy(selected);
      if (runtime.value && controlRunning.value) await applyPolicy(runtime.value, id);
      await refresh();
    } catch (reason) {
      await reportLoopError(reason);
    } finally {
      busy.value = null;
    }
  };

  const ensureSampling = async () => {
    const virtualId = selectedMachine.value?.id;
    if (!virtualId) return;
    if (!runtime.value) {
      updateWorldStatus(
        await commands.runtimeOpen({
          virtualMachineId: virtualId,
          physicalMachineId: null,
          access: "read_only",
          policyId: null,
        }),
      );
    }
    const physicalId = store.state.requestedPhysicalMachine;
    if (
      physicalId &&
      !runtime.value?.activeBody?.physicalMachineId &&
      !store.state.loop?.readSources.some((source) => source.id === physicalId)
    ) {
      updateWorldStatus(
        await commands.runtimeOpen({
          virtualMachineId: virtualId,
          physicalMachineId: physicalId,
          access: "read_only",
          policyId: null,
        }),
      );
    }
  };

  const pauseControl = async () => {
    const loop = runtime.value;
    if (!loop) return;
    updateWorldStatus(
      await sendRuntimeCommand(loop.runtimeGeneration, loop.executionGeneration, {
        kind: "action",
        action: { kind: "drive", on: false },
      }),
    );
  };

  const startLoop = async () => {
    if (
      loopBusy.value ||
      busy.value ||
      !selectedPolicy.value ||
      (physicalSyncEnabled.value && !torqueEnabled.value)
    )
      return;
    loopBusy.value = true;
    try {
      await ensureSampling();
      if (runtime.value) await applyPolicy(runtime.value, selectedPolicy.value.id);
      error.value = null;
    } catch (reason) {
      await reportLoopError(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const stopLoop = async () => {
    if (loopBusy.value || busy.value) return;
    loopBusy.value = true;
    try {
      await pauseControl();
      error.value = null;
    } catch (reason) {
      await reportLoopError(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const toggleLoop = () => {
    if (controlRunning.value) void stopLoop();
    else void startLoop();
  };

  const resetFaultedLoop = async () => {
    const loop = runtime.value;
    if (!loop || loopBusy.value) return;
    loopBusy.value = true;
    try {
      await commands.runtimeClose(loop.runtimeGeneration, "runtime");
      store.state.loop = null;
      await ensureSampling();
      error.value = null;
    } catch (reason) {
      await reportLoopError(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const toggleTorque = async () => {
    const virtualId = selectedMachine.value?.id;
    const physicalId = physicalMachine.value?.id;
    if (!virtualId || !physicalId || !physicalSyncEnabled.value || loopBusy.value || busy.value)
      return;
    loopBusy.value = true;
    try {
      await ensureSampling();
      if (runtime.value) {
        if (runtime.value.activeBody?.physicalMachineId === physicalId && !torqueEnabled.value) {
          updateWorldStatus(
            await sendRuntimeCommand(
              runtime.value.runtimeGeneration,
              runtime.value.executionGeneration,
              {
                kind: "action",
                action: { kind: "power", on: true },
              },
            ),
          );
        } else await switchBody(runtime.value, virtualId, torqueEnabled.value ? null : physicalId);
      }
      await ensureSampling();
      error.value = null;
    } catch (reason) {
      await reportLoopError(reason);
      await ensureSampling().catch(() => undefined);
    } finally {
      loopBusy.value = false;
    }
  };

  const togglePhysicalSync = async () => {
    const virtualId = selectedMachine.value?.id;
    if (
      !virtualId ||
      loopBusy.value ||
      busy.value ||
      (!detached.value && !physicalSyncEnabled.value && !physicalSyncAvailable.value)
    )
      return;
    const previousPhysicalId = store.state.requestedPhysicalMachine;
    loopBusy.value = true;
    try {
      if (physicalSyncEnabled.value || detached.value) {
        if (runtime.value?.activeBody?.physicalMachineId || detached.value) {
          await switchBody(runtime.value!, virtualId, null);
        }
        if (
          store.state.loop?.readSources.some(
            (source) => source.id === store.state.requestedPhysicalMachine,
          )
        ) {
          await commands.runtimeClose(store.state.loop.runtimeGeneration, "readOnlySource");
          const status = await commands.loopStatus();
          if (status) updateWorldStatus(status);
        }
        store.state.requestedPhysicalMachine = null;
      } else {
        await pauseControl();
        store.state.requestedPhysicalMachine = physicalMachine.value?.id ?? null;
        await ensureSampling();
      }
      error.value = null;
    } catch (reason) {
      store.state.requestedPhysicalMachine = previousPhysicalId;
      await reportLoopError(reason);
    } finally {
      loopBusy.value = false;
    }
  };
  watch(
    [() => selectedMachine.value?.id, () => store.state.loading],
    async ([id, loading]) => {
      if (!id || loading || loopBusy.value) return;
      loopBusy.value = true;
      try {
        const loop = runtime.value;
        if (loop && loop.activeBody?.virtualMachineId !== id) {
          if (loop.readSources.length) {
            await commands.runtimeClose(loop.runtimeGeneration, "readOnlySource");
          }
          await switchBody(loop, id, null);
          store.state.requestedPhysicalMachine = null;
        }
        await ensureSampling();
      } catch (reason) {
        await reportLoopError(reason);
      } finally {
        loopBusy.value = false;
      }
    },
    { immediate: true },
  );
  onMounted(() => {
    if (!store.state.policies.length && !store.state.loading) void refresh();
  });

  return () => (
    <div class={[style.page, style.previewPage]}>
      <div class={style.previewMachineRow}>
        <strong>{selectedMachine.value?.name ?? "机器"}</strong>
        <span>{physicalMachine.value?.name ?? "未关联实机"}</span>
        <i
          class={[
            style.previewStatusDot,
            physicalMachine.value?.connected ? style.online : style.offline,
          ]}
          title={physicalMachine.value?.connected ? "实机在线" : "实机离线"}
        />
        <button
          type="button"
          class={[style.previewSource, physicalSyncEnabled.value ? style.syncOn : undefined]}
          aria-pressed={physicalSyncEnabled.value}
          disabled={
            (!detached.value && !physicalSyncEnabled.value && !physicalSyncAvailable.value) ||
            loopBusy.value ||
            !!busy.value
          }
          onClick={() => void togglePhysicalSync()}
          title={
            physicalSyncAvailable.value ? "只读同步真机体态，开启力矩后可运行策略" : "需要真机在线"
          }
        >
          {detached.value ? "切回模拟" : "真机同步"}
        </button>
      </div>
      <div class={style.previewPolicyList} aria-label="已缓存 ONNX 策略">
        {cachedPolicies.value.map((policy) => (
          <button
            class={store.state.selectedPolicyId === policy.id ? style.selected : undefined}
            type="button"
            disabled={busy.value !== null}
            onClick={() => void selectPolicy(policy.id)}
          >
            {policy.name}
          </button>
        ))}
        {!cachedPolicies.value.length && (
          <p class={style.previewEmpty}>当前电脑还没有已缓存的 ONNX</p>
        )}
      </div>
      <ScrollArea className={style.previewParameters} fill>
        {selectedPolicy.value ? (
          <div class={style.previewParameterPanel}>
            <div class={style.previewParameterHeader}>
              <strong>{selectedPolicy.value.name}</strong>
              <small>{selectedPolicy.value.kind === "perpetual" ? "持续策略" : "动作策略"}</small>
            </div>
            {isWalkingPolicy(selectedPolicy.value) ? (
              <>
                <div class={style.previewParameterField}>
                  <span>前进速度（m/s）</span>
                  <Slider
                    value={previewInput.value.twist[0] ?? 0}
                    min={-2}
                    max={2}
                    step={0.01}
                    ariaLabel="前进速度"
                    onInput={(event) => {
                      const value = event.value;
                      previewInput.value = {
                        ...previewInput.value,
                        twist: [value, previewInput.value.twist[1], previewInput.value.twist[2]],
                      };
                      void updatePreviewInput();
                    }}
                  />
                </div>
                <div class={style.previewParameterField}>
                  <span>侧移速度（m/s）</span>
                  <Slider
                    value={previewInput.value.twist[1] ?? 0}
                    min={-2}
                    max={2}
                    step={0.01}
                    ariaLabel="侧移速度"
                    onInput={(event) => {
                      const value = event.value;
                      previewInput.value = {
                        ...previewInput.value,
                        twist: [previewInput.value.twist[0], value, previewInput.value.twist[2]],
                      };
                      void updatePreviewInput();
                    }}
                  />
                </div>
                <div class={style.previewParameterField}>
                  <span>转向速度（rad/s）</span>
                  <Slider
                    value={previewInput.value.twist[2] ?? 0}
                    min={-4}
                    max={4}
                    step={0.01}
                    ariaLabel="转向速度"
                    onInput={(event) => {
                      const value = event.value;
                      previewInput.value = {
                        ...previewInput.value,
                        twist: [previewInput.value.twist[0], previewInput.value.twist[1], value],
                      };
                      void updatePreviewInput();
                    }}
                  />
                </div>
                <div class={style.previewParameterField}>
                  <span>本次前进距离（m，0 为持续行走）</span>
                  <Slider
                    value={previewInput.value.targetDistanceM ?? 0}
                    disabled={physicalSyncEnabled.value}
                    min={0}
                    max={10}
                    step={0.1}
                    ariaLabel="目标前进距离"
                    onInput={(event) => {
                      const value = event.value;
                      previewInput.value = {
                        ...previewInput.value,
                        targetDistanceM: value > 0 ? value : null,
                      };
                      void updatePreviewInput();
                    }}
                  />
                </div>
              </>
            ) : selectedPolicy.value.command?.encoding === "posture_flag" ? (
              <div class={style.previewParameterField}>
                <span>姿态</span>
                <Select
                  value={
                    previewInput.value.twist[0] === selectedPolicy.value.command.sit
                      ? "sit"
                      : "stand"
                  }
                  onChange={(event: Event) => {
                    const posture =
                      event.currentTarget instanceof HTMLSelectElement
                        ? event.currentTarget.value
                        : "stand";
                    const value =
                      posture === "sit"
                        ? (selectedPolicy.value?.command?.sit ?? 1)
                        : (selectedPolicy.value?.command?.stand ?? 0);
                    previewInput.value = {
                      ...previewInput.value,
                      twist: [value, 0, 0],
                    };
                    void updatePreviewInput();
                  }}
                >
                  <option value="stand">站立</option>
                  <option value="sit">坐下</option>
                </Select>
              </div>
            ) : null}
            {selectedPolicy.value.command?.head && (
              <div class={style.previewParameterGroup}>
                <strong>头部控制</strong>
                {[
                  ["颈部俯仰", 0],
                  ["头部俯仰", 1],
                  ["头部偏航", 2],
                  ["头部横滚", 3],
                ].map(([label, index]) => (
                  <div class={style.previewParameterField} key={String(index)}>
                    <span>{label}</span>
                    <Slider
                      value={previewInput.value.head[Number(index)] ?? 0}
                      min={-Math.PI}
                      max={Math.PI}
                      step={0.01}
                      ariaLabel={String(label)}
                      onInput={(event) => {
                        const value = event.value;
                        updateHead(Number(index), value);
                        void updatePreviewInput();
                      }}
                    />
                  </div>
                ))}
              </div>
            )}
            {selectedPolicy.value.command?.body && (
              <div class={style.previewParameterGroup}>
                <strong>身体姿态</strong>
                {[
                  ["高度", 0],
                  ["横滚", 1],
                  ["俯仰", 2],
                ].map(([label, index]) => (
                  <div class={style.previewParameterField} key={String(index)}>
                    <span>{label}</span>
                    <Slider
                      value={previewInput.value.body[Number(index)] ?? 0}
                      min={-Math.PI}
                      max={Math.PI}
                      step={0.01}
                      ariaLabel={String(label)}
                      onInput={(event) => {
                        const value = event.value;
                        updateBody(Number(index), value);
                        void updatePreviewInput();
                      }}
                    />
                  </div>
                ))}
              </div>
            )}
            {!isWalkingPolicy(selectedPolicy.value) &&
              selectedPolicy.value.command?.encoding !== "posture_flag" &&
              !selectedPolicy.value.command?.head &&
              !selectedPolicy.value.command?.body && (
                <p class={style.previewParameterHint}>
                  {selectedPolicy.value.command?.encoding === "phase"
                    ? "此策略由控制环自动生成相位输入。"
                    : "此策略没有需要在预览页输入的运动参数。"}
                </p>
              )}
          </div>
        ) : (
          <p class={style.previewEmpty}>先选择一个已缓存的策略</p>
        )}
      </ScrollArea>
      <div class={style.previewLoopBar}>
        {physicalSyncEnabled.value && (
          <button
            type="button"
            disabled={loopBusy.value || (!torqueEnabled.value && !physicalSyncAvailable.value)}
            onClick={() => void toggleTorque()}
          >
            {torqueEnabled.value ? "关闭力矩" : "开启力矩"}
          </button>
        )}
        <button
          type="button"
          disabled={
            loopBusy.value ||
            runtime.value?.runtimeState === "faulted" ||
            (!controlRunning.value &&
              (!selectedPolicy.value ||
                (physicalSyncEnabled.value &&
                  (!torqueEnabled.value || !physicalSyncAvailable.value))))
          }
          onClick={toggleLoop}
        >
          {controlRunning.value
            ? "停止控制环"
            : physicalSyncEnabled.value
              ? "开启真机控制环"
              : "开启控制环"}
        </button>
        <button type="button" onClick={() => void openLogs()}>
          打开日志
        </button>
        {runtime.value?.runtimeState === "faulted" && (
          <button type="button" disabled={loopBusy.value} onClick={() => void resetFaultedLoop()}>
            重置控制环
          </button>
        )}
        {runtime.value && (
          <small>
            {runtime.value.runtimeState === "faulted"
              ? `控制环故障：${runtime.value.lastError ?? "执行失败"}`
              : controlRunning.value
                ? "策略运行中"
                : physicalSyncEnabled.value
                  ? torqueEnabled.value
                    ? "力矩已开启，等待策略启动"
                    : "只读同步真机体态"
                  : "采样中，等待策略启动"}
          </small>
        )}
        {error.value && <span class={style.previewError}>{error.value}</span>}
        <div class={style.previewSystemActions}>
          <button
            type="button"
            disabled={
              physicalSyncEnabled.value || worldAtConfiguredPosition.value || loopBusy.value
            }
            onClick={() => void resetWorld()}
          >
            归零
          </button>
          <button type="button" onClick={requestDuckFocus}>
            聚焦
          </button>
        </div>
      </div>
    </div>
  );
});

export const HomePage = defineComponent(() => {
  const store = useDesktopStore();
  onMounted(() => {
    if (!store.state.catalog.length && !store.state.loading) void load();
  });

  return () => (
    <div
      class={[style.page, style.homePage, store.state.tab === "settings" && style.hidden]}
      aria-hidden={store.state.tab === "settings"}
    >
      <OfficialDuckStage />
      {store.state.tab === "debug" && <DebugPage />}
      {store.state.tab === "preview" && <PreviewPage />}
      {store.state.tab === "training" && <TrainingPage />}
    </div>
  );
});
