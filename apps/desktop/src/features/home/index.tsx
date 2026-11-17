import { computed, defineComponent, onMounted, onUnmounted, ref, shallowRef, watch } from "vue";
import { useEventListener, useIntervalFn } from "@vueuse/core";
import type { LoopStatus, PolicyEntry, PreviewInput, WorldState } from "../../bridge/types";
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
import { desktopBackend, persistWorldCache, pruneWorldCache, useDesktopStore } from "../../store";
import {
  DebugPage,
  debugBindingFor,
  debugPhysicalMachineFor,
  load,
  officialSceneFrame,
  officialSceneModel,
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
const officialServoIds = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];
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

const isWalkingPolicy = (policy: PolicyEntry | undefined) =>
  policy?.slot === "walk" || /walking|velstand/i.test(policy?.file ?? "");

const inputForPolicy = (policy: PolicyEntry | undefined): PreviewInput => ({
  twist: isWalkingPolicy(policy) ? [0.15, 0, 0] : [0, 0, 0],
  head: [0, 0, 0, 0],
  body: [0, 0, 0],
  targetDistanceM: isWalkingPolicy(policy) ? 1 : null,
});

export const OfficialDuckStage = defineComponent(() => {
  const store = useDesktopStore();
  const host = shallowRef<HTMLDivElement>();
  const viewer = shallowRef<DuckViewer>();
  const scene = ref<Awaited<ReturnType<typeof officialSceneModel>> | null>(null);
  const frame = ref<Awaited<ReturnType<typeof officialSceneFrame>> | null>(null);
  const debug = computed(() => store.state.tab === "debug");
  const activeLoop = computed(() =>
    debug.value || store.state.physicalSyncEnabled ? store.state.loop : store.state.previewLoop,
  );
  const cachedWorld = computed(() => {
    const machineId = store.state.selectedVirtualMachine;
    return machineId ? store.state.worldCache[machineId] : undefined;
  });
  const displayWorld = computed(() => activeLoop.value?.world ?? cachedWorld.value);
  const activePositions = computed(() =>
    debug.value || store.state.physicalSyncEnabled
      ? (store.state.rangePreview ?? store.state.loop?.positions ?? store.state.debugPose)
      : store.state.previewLoop?.positions,
  );
  const markerPos = ref<({ x: number; y: number } | null)[]>([]);
  const labelPos = ref<({ x: number; y: number } | null)[]>([]);
  const selectedServoId = ref<number | null>(null);
  let frameRequestInFlight = false;
  let pendingFramePositions: number[] | null = null;
  let stageActive = true;

  const mappedMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const physicalSyncAvailable = computed(() => {
    const machine = mappedMachine.value;
    if (!machine?.connected || machine.onlineServoIds.length !== JOINT_COUNT) return false;
    return new Set(machine.onlineServoIds).size === JOINT_COUNT;
  });
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const rangeStateBySlot = computed(
    () => new Map(store.state.jointRangeStates.map((state) => [state.slotId, state])),
  );
  watch(physicalSyncAvailable, (available) => {
    if (!available && store.state.physicalSyncEnabled) {
      store.state.physicalSyncEnabled = false;
      if (store.state.loop) {
        void desktopBackend()
          .loopClose()
          .then(() => {
            store.state.loop = null;
            store.state.jointRangeStates = [];
          })
          .catch(() => undefined);
      }
    }
  });
  watch(
    () => store.state.selectedVirtualMachine,
    (current, previous) => {
      if (current === previous || !store.state.physicalSyncEnabled) return;
      store.state.physicalSyncEnabled = false;
      if (store.state.loop) {
        void desktopBackend()
          .loopClose()
          .then(() => {
            store.state.loop = null;
            store.state.jointRangeStates = [];
          })
          .catch(() => undefined);
      }
    },
  );
  watch(
    [() => store.state.virtualMachines.map((machine) => machine.id), () => store.state.loading],
    ([machineIds, loading]) => {
      if (loading || (!machineIds.length && !store.state.catalog.length)) return;
      pruneWorldCache(store.state.worldCache, machineIds);
    },
    { immediate: true },
  );
  /** Every virtual machine in the scene; the selected one is the live duck. */
  const sceneMachines = computed(() => {
    const machines = store.state.virtualMachines;
    const selected = machines.find((machine) => machine.id === store.state.selectedVirtualMachine);
    return selected ? [selected, ...machines.filter((machine) => machine !== selected)] : machines;
  });
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
  /** Every duck gets a label so selecting a machine remains visible in both views. */
  const labels = computed(() => sceneMachines.value);
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
        debugBindingFor(selectedMachine.value, name)?.physicalId ?? officialServoIds[index],
    })),
  );
  const markerState = (physicalId: number) => {
    const online = mappedMachine.value?.onlineServoIds ?? [];
    if (!mappedMachine.value) return "unknown";
    if (activeLoop.value?.sensorFailures) return "warn";
    return online.includes(physicalId) ? "online" : "missing";
  };

  async function refreshFrame(positions: number[]) {
    if (!scene.value) return;
    if (frameRequestInFlight) {
      pendingFramePositions = [...positions];
      return;
    }
    frameRequestInFlight = true;
    try {
      const rawFrame = await officialSceneFrame(
        positions.length === JOINT_COUNT ? positions : Array(JOINT_COUNT).fill(0),
      );
      const markerFrame = officialJointNames.map(
        (_, index) => rawFrame.markers[index] ?? [0, 0, 0],
      );
      frame.value = { ...rawFrame, markers: markerFrame };
      viewer.value?.setFrame(frame.value);
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
    const world = displayWorld.value;
    viewer.value.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
    await refreshFrame(activePositions.value ?? Array(JOINT_COUNT).fill(0));
  }

  useEventListener(window, focusDuckEvent, focusCurrentDuck);
  useEventListener(window, defaultPoseEvent, restoreDefaultPose);
  useEventListener(window, restoreDuckViewEvent, restorePreviousDuckView);
  useEventListener(window, servoSelectionEvent, onServoSelection);

  onMounted(async () => {
    stageActive = true;
    scene.value = await officialSceneModel();
    if (!stageActive) return;
    await buildViewer();
  });

  onUnmounted(() => {
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
    void refreshFrame(Array(JOINT_COUNT).fill(0));
  }

  function selectSceneDuck(event: MouseEvent) {
    const index = viewer.value?.pickDuck(event.clientX, event.clientY);
    if (index == null) return;
    const machine = sceneMachines.value[index];
    if (!machine) return;
    store.state.selectedVirtualMachine = machine.id;
    store.state.selectedVirtualMachineManual = true;
  }

  watch(
    () => activePositions.value,
    (positions) => {
      if (positions) void refreshFrame(positions);
    },
  );
  watch(
    () => displayWorld.value,
    (world) => {
      viewer.value?.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
    },
    { deep: true },
  );
  watch(
    () => activeLoop.value?.jointRangeStates,
    (states) => {
      store.state.jointRangeStates = states ? states.map((state) => ({ ...state })) : [];
    },
    { deep: true, immediate: true },
  );
  watch(
    [() => debug.value, () => sceneMachines.value, () => worldPositions.value],
    () => viewer.value?.setGhosts(ghostCount(), 0.25, worldPositions.value),
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
                  title={`${marker.name} · ID ${marker.physicalId} · ${markerLabel[state]}`}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    requestServoSelection(selectedServoId.value === marker.id ? null : marker.id);
                  }}
                >
                  {marker.physicalId}
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
  let ticking = false;
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
  const physicalSyncAvailable = computed(() => {
    const machine = physicalMachine.value;
    if (!machine?.connected || machine.onlineServoIds.length !== JOINT_COUNT) return false;
    return new Set(machine.onlineServoIds).size === JOINT_COUNT;
  });
  const worldAtConfiguredPosition = computed(() => {
    const machineId = selectedMachine.value?.id;
    const world = machineId
      ? (store.state.worldCache[machineId] ??
        (store.state.physicalSyncEnabled ? store.state.loop : store.state.previewLoop)?.world)
      : undefined;
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
  const previewInput = ref<PreviewInput>({
    twist: [0.15, 0, 0],
    head: [0, 0, 0, 0],
    body: [0, 0, 0],
    targetDistanceM: 1,
  });
  const backendWorld = ref<WorldState | null>(null);
  watch(
    () => selectedMachine.value?.id,
    () => {
      backendWorld.value = null;
    },
    { immediate: true },
  );
  watch(
    () => selectedMachine.value?.id,
    (current, previous) => {
      if (!previous || current === previous || store.state.physicalSyncEnabled) return;
      if (!store.state.previewLoop) return;
      void desktopBackend()
        .previewLoopClose()
        .then(() => {
          store.state.previewLoop = null;
          backendWorld.value = null;
        })
        .catch((reason) => {
          error.value = reason instanceof Error ? reason.message : String(reason);
        });
    },
  );

  const updateWorldStatus = (status: LoopStatus): LoopStatus => {
    const machineId = selectedMachine.value?.id;
    if (!machineId) return status;
    const previous = backendWorld.value ?? status.world;
    const cached = store.state.worldCache[machineId] ?? {
      position: [0, 0] as [number, number],
      yaw: 0,
      distanceM: 0,
    };
    const next: WorldState = {
      position: [
        (cached.position[0] ?? 0) + (status.world.position[0] ?? 0) - (previous.position[0] ?? 0),
        (cached.position[1] ?? 0) + (status.world.position[1] ?? 0) - (previous.position[1] ?? 0),
      ],
      yaw: (cached.yaw ?? 0) + (status.world.yaw ?? 0) - (previous.yaw ?? 0),
      distanceM:
        (cached.distanceM ?? 0) + (status.world.distanceM ?? 0) - (previous.distanceM ?? 0),
    };
    backendWorld.value = status.world;
    store.state.worldCache[machineId] = next;
    persistWorldCache(store.state.worldCache);
    return { ...status, world: next };
  };

  const updatePreviewInput = async () => {
    try {
      if (store.state.physicalSyncEnabled) {
        if (!store.state.loop) return;
        await desktopBackend().loopPolicyCommand(previewInput.value);
        const status = await desktopBackend().loopStatus();
        if (status) store.state.loop = updateWorldStatus(status);
      } else {
        if (!store.state.previewLoop) return;
        store.state.previewLoop = updateWorldStatus(
          await desktopBackend().previewLoopCommand(previewInput.value),
        );
      }
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const openLogs = async () => {
    try {
      await desktopBackend().openLogs();
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const resetWorld = async () => {
    const machineId = selectedMachine.value?.id;
    if (!machineId) return;
    try {
      const status = store.state.physicalSyncEnabled
        ? store.state.loop
          ? await desktopBackend().loopStatus()
          : null
        : store.state.previewLoop
          ? await desktopBackend().previewLoopResetWorld()
          : null;
      delete store.state.worldCache[machineId];
      persistWorldCache(store.state.worldCache);
      if (status) {
        backendWorld.value = status.world;
        const resetStatus = {
          ...status,
          world: { position: [0, 0] as [number, number], yaw: 0, distanceM: 0 },
        };
        if (store.state.physicalSyncEnabled) store.state.loop = resetStatus;
        else store.state.previewLoop = resetStatus;
      }
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
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
      store.state.policies = await desktopBackend().policyList();
      store.state.selectedPolicyId =
        store.state.policies.find((policy) => policy.selected)?.id ?? null;
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const selectPolicy = async (id: string) => {
    busy.value = id;
    try {
      const selected = await desktopBackend().policySelect(id);
      store.state.selectedPolicyId = selected.id;
      previewInput.value = inputForPolicy(selected);
      if (store.state.physicalSyncEnabled && store.state.loop) {
        await desktopBackend().loopClose();
        store.state.loop = null;
        store.state.jointRangeStates = [];
        await startLoop();
      } else if (store.state.previewLoop) {
        store.state.previewLoop = updateWorldStatus(await desktopBackend().previewLoopPolicy(id));
        await updatePreviewInput();
      }
      await refresh();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      busy.value = null;
    }
  };

  const startLoop = async (allowBusy = false) => {
    if (!selectedMachine.value || (loopBusy.value && !allowBusy)) return;
    loopBusy.value = true;
    backendWorld.value = null;
    try {
      if (store.state.physicalSyncEnabled) {
        const machine = physicalMachine.value;
        if (!machine || !physicalSyncAvailable.value) {
          throw new Error("真机未连接完整的 15 个舵机");
        }
        store.state.loop = updateWorldStatus(
          await desktopBackend().loopOpen({
            virtualMachineId: selectedMachine.value.id,
            servoIds: officialServoIds,
            physicalMachineId: machine.id,
            policyId: selectedPolicy.value?.id,
          }),
        );
        store.state.loop = updateWorldStatus(
          await desktopBackend().loopAction({ kind: "power", on: true }),
        );
        for (
          let tickCount = 0;
          store.state.loop.phase === "homing" && tickCount < 8;
          tickCount += 1
        )
          store.state.loop = updateWorldStatus(await desktopBackend().loopTick());
        if (store.state.loop.phase === "ready" || store.state.loop.phase === "holding")
          store.state.loop = updateWorldStatus(
            await desktopBackend().loopAction({ kind: "drive", on: true }),
          );
        await updatePreviewInput();
      } else {
        store.state.previewLoop = updateWorldStatus(
          await desktopBackend().previewLoopOpen(selectedMachine.value.id),
        );
        if (selectedPolicy.value) {
          store.state.previewLoop = updateWorldStatus(
            await desktopBackend().previewLoopPolicy(selectedPolicy.value.id),
          );
          await updatePreviewInput();
        }
      }
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const stopLoop = async () => {
    if (loopBusy.value) return;
    loopBusy.value = true;
    try {
      if (store.state.physicalSyncEnabled) {
        await desktopBackend().loopClose();
        store.state.loop = null;
      } else {
        await desktopBackend().previewLoopClose();
        store.state.previewLoop = null;
      }
      backendWorld.value = null;
      store.state.jointRangeStates = [];
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const toggleLoop = () => {
    if (store.state.physicalSyncEnabled ? store.state.loop : store.state.previewLoop) {
      void stopLoop();
      return;
    }
    void startLoop();
  };

  const togglePhysicalSync = async () => {
    if (!physicalSyncAvailable.value || loopBusy.value) return;
    loopBusy.value = true;
    try {
      if (store.state.physicalSyncEnabled) {
        store.state.physicalSyncEnabled = false;
        if (store.state.loop) {
          await desktopBackend().loopClose();
          store.state.loop = null;
          store.state.jointRangeStates = [];
        }
      } else {
        store.state.physicalSyncEnabled = true;
        if (store.state.previewLoop) {
          await desktopBackend().previewLoopClose();
          store.state.previewLoop = null;
        }
        if (!store.state.loop) await startLoop(true);
      }
      error.value = null;
    } catch (reason) {
      store.state.physicalSyncEnabled = false;
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const { pause: stopTimer, resume: startTimer } = useIntervalFn(
    () => {
      const activeLoop = store.state.physicalSyncEnabled
        ? store.state.loop
        : store.state.previewLoop;
      if (!activeLoop || ticking) return;
      ticking = true;
      void (
        store.state.physicalSyncEnabled
          ? desktopBackend().loopTick()
          : desktopBackend().previewLoopTick()
      )
        .then((status) => {
          if (store.state.physicalSyncEnabled) store.state.loop = updateWorldStatus(status);
          else store.state.previewLoop = updateWorldStatus(status);
        })
        .catch((reason) => {
          error.value = reason instanceof Error ? reason.message : String(reason);
        })
        .finally(() => {
          ticking = false;
        });
    },
    20,
    { immediate: false },
  );

  onMounted(() => {
    if (!store.state.policies.length && !store.state.loading) void refresh();
    startTimer();
  });
  onUnmounted(stopTimer);

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
          class={[style.previewSource, store.state.physicalSyncEnabled ? style.syncOn : undefined]}
          disabled={!physicalSyncAvailable.value || loopBusy.value}
          onClick={() => void togglePhysicalSync()}
          title={
            physicalSyncAvailable.value
              ? "将当前 VLA 控制环切换到真机"
              : "需要真机在线且 15 个舵机全部在线"
          }
        >
          真机同步
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
                  <span>目标前进距离（m）</span>
                  <Slider
                    value={previewInput.value.targetDistanceM ?? 0}
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
        <button type="button" disabled={loopBusy.value} onClick={toggleLoop}>
          {store.state.physicalSyncEnabled
            ? store.state.loop
              ? "停止控制环"
              : "开启真机控制环"
            : store.state.previewLoop
              ? "停止控制环"
              : "开启控制环"}
        </button>
        <button type="button" onClick={() => void openLogs()}>
          打开日志
        </button>
        {store.state.previewLoop && (
          <small>{store.state.previewLoop.torque ? "策略运行中" : "控制环已开启，等待策略"}</small>
        )}
        {error.value && <span class={style.previewError}>{error.value}</span>}
        <div class={style.previewSystemActions}>
          <button
            type="button"
            disabled={worldAtConfiguredPosition.value || loopBusy.value}
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
