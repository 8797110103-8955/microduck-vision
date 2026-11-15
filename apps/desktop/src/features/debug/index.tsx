import { computed, defineComponent, onMounted, onUnmounted, ref, watch } from "vue";
import { useEventListener, useIntervalFn } from "@vueuse/core";
import type {
  ActionRecord,
  JointRange,
  LoopAction,
  PhysicalMachine,
  VirtualMachine,
} from "../../bridge/types";
import { showMessage } from "../../components/message";
import { ScrollArea } from "../../components/scroll-area";
import { Slider } from "../../components/slider";
import {
  requestDuckFocus,
  requestDefaultPose,
  requestRestoreDuckView,
  requestServoSelection,
  servoSelectionEvent,
} from "../../components/viewer-events";
import { desktopBackend, useDesktopStore } from "../../store";
import style from "./style.module.scss";

export { actionKind, servoModel, SectionHeading } from "./helpers";

const debugServoIndexes = Array.from({ length: 15 }, (_, index) => index + 1);
const officialServoIds = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];
const formatParameter = (value: number | null, suffix = "") =>
  value === null ? "—" : `${value.toFixed(3)}${suffix}`;

function unboundVirtualMachines(
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  const boundVirtualMachineIds = new Set(
    physicalMachines.map((machine) => machine.virtualMachineId).filter((id): id is string => !!id),
  );
  return virtualMachines.filter((machine) => !boundVirtualMachineIds.has(machine.id));
}

export function bindingConflictsFor(machine: VirtualMachine | undefined) {
  if (!machine) return [];
  const byId = new Map<number, string[]>();
  for (const binding of machine.jointBindings ?? []) {
    const slots = byId.get(binding.physicalId) ?? [];
    slots.push(binding.slotId);
    byId.set(binding.physicalId, slots);
  }
  return [...byId]
    .filter(([, slots]) => slots.length > 1)
    .map(([id, slots]) => `${id}:${slots.toSorted().join(",")}`)
    .toSorted();
}

export function debugPhysicalMachineFor(
  virtualMachineId: string | null,
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  if (!virtualMachineId) return null;
  const explicitlyBound =
    physicalMachines.find(
      (machine) => machine.virtualMachineId === virtualMachineId && machine.connected,
    ) ?? physicalMachines.find((machine) => machine.virtualMachineId === virtualMachineId);
  if (explicitlyBound) return explicitlyBound;

  const unboundMachines = unboundVirtualMachines(virtualMachines, physicalMachines);
  const unboundPhysicalMachines = physicalMachines.filter(
    (machine) => machine.connected && !machine.virtualMachineId,
  );
  const index = unboundMachines.findIndex((machine) => machine.id === virtualMachineId);
  return index >= 0 ? (unboundPhysicalMachines[index] ?? null) : null;
}

/**
 * Resolve the body-to-servo mapping used by the debug view and the 3D stage.
 *
 * Physical machines do not own bindings.  Their census only describes which
 * IDs are currently online; the simulated machine mapping is the source of
 * truth used by both the debug controls and the 3D stage.
 */
export function debugBindingFor(virtualMachine: VirtualMachine | undefined, slotId: string) {
  const virtualBindings = virtualMachine?.jointBindings ?? [];
  return virtualBindings.find((binding) => binding.slotId === slotId);
}

export function defaultDebugVirtualMachineId(
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  if (!physicalMachines.some((machine) => machine.connected && !machine.virtualMachineId))
    return null;
  return unboundVirtualMachines(virtualMachines, physicalMachines)[0]?.id ?? null;
}

export const DebugPage = defineComponent(() => {
  const store = useDesktopStore();
  const selectedId = ref<number | null>(null);
  const secondaryPanel = ref<"binding" | "safety" | "world">("safety");
  const rangeView = ref(false);
  const jointSlots = ref<string[]>([]);
  const jointRanges = ref<JointRange[]>([]);
  const selectedRangeEndpoint = ref<{ slotId: string; endpoint: "min" | "max" } | null>(null);
  const bindingConflictSignature = ref("");
  let suppressServoSelectionEvent = false;
  let poseRequestInFlight = false;
  let posePollingActive = false;
  const bindingOverrides = ref<Record<string, number>>({});
  const targetDraft = ref("");
  const worldPositionDraft = ref<[number, number, number]>([0, 0, 0]);
  const bindingSelects = new Map<string, HTMLSelectElement>();
  const selectedIndex = computed(() =>
    selectedId.value === null ? 0 : Math.max(0, selectedId.value - 1),
  );
  const selectedPosition = computed(
    () =>
      store.state.loop?.positions[selectedIndex.value] ??
      store.state.debugPose?.[selectedIndex.value] ??
      null,
  );
  const selectedTarget = computed(() => store.state.loop?.targets[selectedIndex.value] ?? null);
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const selectedPhysicalMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const physicalConnected = computed(() => selectedPhysicalMachine.value?.connected === true);
  const anyTorqueEnabled = computed(
    () => store.state.loop?.torque ?? selectedPhysicalMachine.value?.torqueEnabled === true,
  );
  const applyDefaultSelection = () => {
    if (store.state.selectedVirtualMachineManual) return;
    const defaultId = defaultDebugVirtualMachineId(
      store.state.virtualMachines,
      store.state.physicalMachines,
    );
    if (defaultId) store.state.selectedVirtualMachine = defaultId;
  };
  useEventListener(window, servoSelectionEvent, onServoSelection);
  const { pause: stopPosePolling, resume: startPosePolling } = useIntervalFn(
    () => {
      const machine = selectedPhysicalMachine.value;
      if (!posePollingActive || !machine?.connected || store.state.loop || poseRequestInFlight)
        return;
      if (bindingConflictsFor(selectedMachine.value).length) {
        checkBindingConflict();
        return;
      }
      poseRequestInFlight = true;
      void desktopBackend()
        .physicalMachinePose(machine.id, store.state.selectedVirtualMachine)
        .then((pose) => {
          if (posePollingActive) store.state.debugPose = pose;
        })
        .catch(() => {
          if (posePollingActive) checkBindingConflict();
        })
        .finally(() => {
          poseRequestInFlight = false;
        });
    },
    180,
    { immediate: false },
  );

  onMounted(async () => {
    posePollingActive = true;
    if (!store.state.catalog.length && !store.state.loading) await load();
    if (!posePollingActive) return;
    jointSlots.value = await desktopBackend().officialJointSlots();
    jointRanges.value = await desktopBackend().officialJointRanges();
    if (!posePollingActive) return;
    applyDefaultSelection();
    if (!physicalConnected.value) secondaryPanel.value = "binding";
    checkBindingConflict();
    startPosePolling();
  });
  onUnmounted(() => {
    stopPosePolling();
    posePollingActive = false;
    poseRequestInFlight = false;
    store.state.debugPose = null;
  });
  onUnmounted(() => {
    if (store.state.loop) void closeLoop();
  });
  watch(selectedTarget, (value) => {
    targetDraft.value = value == null ? "" : String(value);
  });
  watch(physicalConnected, (connected) => {
    if (!connected && secondaryPanel.value === "safety") secondaryPanel.value = "binding";
  });
  watch(
    () => selectedMachine.value?.id,
    () => {
      const position = selectedMachine.value?.simulation.worldPosition;
      worldPositionDraft.value = [position?.[0] ?? 0, position?.[1] ?? 0, position?.[2] ?? 0];
    },
    { immediate: true },
  );
  const saveWorldPosition = async (position = worldPositionDraft.value) => {
    const machine = selectedMachine.value;
    if (!machine) return;
    try {
      const updated = await desktopBackend().setVirtualMachineWorldPosition(machine.id, position);
      const index = store.state.virtualMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.virtualMachines[index] = updated;
      store.state.error = null;
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const updateWorldPosition = (index: number, value: number) => {
    const next: [number, number, number] = [...worldPositionDraft.value];
    next[index] = value;
    worldPositionDraft.value = next;
    const machine = selectedMachine.value;
    if (machine) machine.simulation.worldPosition = next;
    void saveWorldPosition(next);
  };
  const send = () => {
    if (bindingConflictsFor(selectedMachine.value).length) {
      checkBindingConflict();
      return;
    }
    void sendTargets((store.state.loop?.targets ?? []).map((v) => v + 0.01));
  };
  const move = async (draftValue = targetDraft.value) => {
    if (bindingConflictsFor(selectedMachine.value).length) {
      checkBindingConflict();
      return;
    }
    const value = Number(draftValue);
    if (!Number.isFinite(value)) return;
    if (!store.state.loop) {
      const machine = selectedPhysicalMachine.value;
      const slotId =
        jointSlots.value[selectedIndex.value] ??
        (await desktopBackend().officialJointSlots())[selectedIndex.value];
      if (!machine?.connected || !slotId) return;
      try {
        const binding = bindingFor(slotId);
        const bodyTarget = Math.min(
          binding?.maxRad ?? Number.POSITIVE_INFINITY,
          Math.max(binding?.minRad ?? Number.NEGATIVE_INFINITY, value),
        );
        const servoTarget =
          (bodyTarget + (binding?.zeroOffsetRad ?? 0)) * (binding?.direction ?? 1);
        const torqueMachine = await desktopBackend().setPhysicalMachineTorque(
          machine.id,
          true,
          store.state.selectedVirtualMachine,
        );
        const updated = await desktopBackend().writePhysicalMachinePosition({
          physicalMachineId: machine.id,
          servoId: binding?.physicalId ?? physicalIdForIndex(selectedIndex.value),
          positionRad: servoTarget,
        });
        const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
        if (index >= 0) store.state.physicalMachines[index] = { ...torqueMachine, ...updated };
        store.state.error = null;
      } catch (error) {
        store.state.error = error instanceof Error ? error.message : String(error);
      }
      return;
    }
    const loop = store.state.loop;
    const targets = [...loop.targets];
    targets[selectedIndex.value] = value;
    let status = store.state.loop;
    if (!status.torque && status.phase !== "estop") {
      status = await sendAction({ kind: "power", on: true });
    }
    for (let tickCount = 0; status.phase === "homing" && tickCount < 8; tickCount += 1) {
      status = await tick();
    }
    if (status.phase === "ready" || status.phase === "holding") {
      await sendTargets(targets);
      status = store.state.loop;
      if (status?.phase === "ready" || status?.phase === "holding") {
        status = await sendAction({ kind: "drive", on: true });
      }
    } else if (status.phase === "driving") {
      await sendTargets(targets);
      status = store.state.loop;
    }
    if (status?.phase === "driving") await tick();
  };
  const refresh = async () => {
    store.state.loop = await desktopBackend().loopStatus();
  };
  function checkBindingConflict() {
    const conflicts = bindingConflictsFor(selectedMachine.value);
    const signature = conflicts.join("|");
    if (!signature) {
      bindingConflictSignature.value = "";
      return;
    }
    rangeView.value = false;
    secondaryPanel.value = "binding";
    if (bindingConflictSignature.value === signature) return;
    bindingConflictSignature.value = signature;
    showMessage(`绑定冲突：${conflicts.map((item) => item.replace(":", " → ")).join("；")}`, {
      type: "warning",
      duration: 6_000,
    });
  }
  watch(
    () => store.state.selectedVirtualMachine,
    () => {
      bindingConflictSignature.value = "";
      checkBindingConflict();
    },
  );
  const clearServoSelection = () => {
    if (selectedId.value === null) return;
    selectedId.value = null;
    suppressServoSelectionEvent = true;
    requestServoSelection(null);
    suppressServoSelectionEvent = false;
    checkBindingConflict();
  };
  const selectServo = (id: number) => {
    if (selectedId.value === id) {
      clearServoSelection();
      return;
    }
    selectedId.value = id;
    suppressServoSelectionEvent = true;
    requestServoSelection(id);
    suppressServoSelectionEvent = false;
    checkBindingConflict();
  };
  const selectBinding = (id: number) => {
    if (selectedId.value === id) return;
    selectServo(id);
  };
  function onServoSelection(event: Event) {
    if (suppressServoSelectionEvent) return;
    if (!(event instanceof CustomEvent)) return;
    if (event.detail === null) {
      if (selectedId.value !== null) {
        selectedId.value = null;
        checkBindingConflict();
      }
      return;
    }
    if (typeof event.detail !== "number") return;
    selectedId.value = Math.max(1, Math.min(15, event.detail));
    checkBindingConflict();
  }
  const bindingFor = (slotId: string) => {
    const binding = debugBindingFor(selectedMachine.value, slotId);
    const override = bindingOverrides.value[slotId];
    return (
      binding ??
      (override === undefined
        ? undefined
        : {
            slotId,
            physicalId: override,
            direction: 1,
            zeroOffsetRad: 0,
            minRad: null,
            maxRad: null,
          })
    );
  };
  const physicalIdForIndex = (index: number) => {
    const slotId = jointSlots.value[index];
    return slotId
      ? (bindingFor(slotId)?.physicalId ?? officialServoIds[index])
      : officialServoIds[index];
  };
  const bindingOptionsFor = (index: number) => {
    const officialId = officialServoIds[index];
    const census = selectedPhysicalMachine.value?.census ?? [];
    return [
      { id: officialId, label: `${officialId} · 官方默认` },
      ...census
        .filter((servo) => servo.servoId !== officialId)
        .toSorted((a, b) => a.servoId - b.servoId)
        .map((servo) => ({ id: servo.servoId, label: `${servo.servoId} · ${servo.model}` })),
    ];
  };
  const updateBinding = async (slotId: string, value: string) => {
    const virtualMachineId = store.state.selectedVirtualMachine;
    if (!virtualMachineId) return;
    try {
      const updated = await desktopBackend().bindVirtualMachine({
        virtualMachineId,
        slotId,
        physicalId: value ? Number(value) : null,
      });
      const virtualIndex = store.state.virtualMachines.findIndex((item) => item.id === updated.id);
      if (virtualIndex >= 0) {
        store.state.virtualMachines[virtualIndex] = updated;
      }
      if (value) bindingOverrides.value[slotId] = Number(value);
      else delete bindingOverrides.value[slotId];
      store.state.error = null;
      checkBindingConflict();
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const openLogs = async () => {
    try {
      await desktopBackend().openLogs();
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const setPoseZero = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!machine?.connected) return;
    if (bindingConflictsFor(selectedMachine.value).length) {
      checkBindingConflict();
      return;
    }
    try {
      const updated = await desktopBackend().setPhysicalMachinePoseZero(
        machine.id,
        store.state.selectedVirtualMachine,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.physicalMachines[index] = updated;
      showMessage("已绑定当前姿态为零位", { type: "info" });
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const toggleAllTorque = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!machine?.connected) return;
    if (bindingConflictsFor(selectedMachine.value).length) {
      checkBindingConflict();
      return;
    }
    try {
      const updated = await desktopBackend().setPhysicalMachineTorque(
        machine.id,
        !machine.torqueEnabled,
        store.state.selectedVirtualMachine,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.physicalMachines[index] = updated;
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const scenePose = async () => {
    const machine = selectedPhysicalMachine.value;
    if (machine?.connected && !store.state.loop) {
      if (bindingConflictsFor(selectedMachine.value).length) {
        checkBindingConflict();
        requestDuckFocus();
        return;
      }
      try {
        store.state.debugPose = await desktopBackend().physicalMachinePose(
          machine.id,
          store.state.selectedVirtualMachine,
        );
      } catch (error) {
        store.state.error = error instanceof Error ? error.message : String(error);
      }
    } else {
      await refresh();
    }
    requestDuckFocus();
  };
  const rangeFor = (slotId: string) =>
    jointRanges.value.find((range) => range.slotId === slotId) ?? {
      slotId,
      minRad: -Math.PI,
      maxRad: Math.PI,
    };

  return () => (
    <ScrollArea className={[style.page, style.debugPage].filter(Boolean).join(" ")}>
      <div class={style.debugMachineRow}>
        <span class={style.machineTitle}>{selectedMachine.value?.name ?? "机器"}</span>
        <span>{selectedPhysicalMachine.value?.name ?? "未关联实机"}</span>
        <i
          class={[
            style.debugStatusDot,
            selectedPhysicalMachine.value?.connected ? style.online : style.offline,
          ]}
          title={selectedPhysicalMachine.value?.connected ? "实机在线" : "实机离线"}
        />
        <span
          class={[rangeView.value ? style.machineTitle : undefined, style.machineViewToggle]}
          onClick={() => (rangeView.value = !rangeView.value)}
        >
          肢体范围
        </span>
      </div>
      <div class={style.systemActions}>
        {rangeView.value ? (
          <button type="button" onClick={() => void scenePose()}>
            场景姿态预览
          </button>
        ) : (
          <button type="button" onClick={requestDefaultPose}>
            还原姿态零位
          </button>
        )}
        <button type="button" onClick={requestDuckFocus}>
          聚焦
        </button>
        <button type="button" onClick={requestRestoreDuckView}>
          回到聚焦前位置
        </button>
      </div>
      <article class={[style.panelCard, rangeView.value ? style.rangeOnlyCard : undefined]}>
        <div class={style.panelHeader}>
          <div class={style.panelTabs}>
            {(
              [
                ["safety", "安全状态"],
                ["binding", "肢体绑定"],
                ["world", "世界位置"],
              ] as const
            ).map(([key, label]) => (
              <button
                class={secondaryPanel.value === key ? style.active : undefined}
                type="button"
                disabled={key === "safety" && !physicalConnected.value}
                onClick={() => (secondaryPanel.value = key)}
              >
                {label}
              </button>
            ))}
          </div>
        </div>
        {!rangeView.value && secondaryPanel.value === "binding" && (
          <div class={style.bindingList}>
            {jointSlots.value.map((slotId, index) => (
              <div
                class={[
                  style.bindingRow,
                  selectedId.value === index + 1 ? style.activeRow : undefined,
                ]}
                onClick={() => selectServo(index + 1)}
              >
                <span>{slotId}</span>
                <select
                  value={String(bindingFor(slotId)?.physicalId ?? officialServoIds[index])}
                  ref={(element) => {
                    // oxlint-disable-next-line no-unsafe-type-assertion
                    if (element) bindingSelects.set(slotId, element as HTMLSelectElement);
                    else bindingSelects.delete(slotId);
                  }}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    selectBinding(index + 1);
                  }}
                  onChange={() => {
                    void updateBinding(slotId, bindingSelects.get(slotId)?.value ?? "");
                  }}
                >
                  {!bindingOptionsFor(index).some(
                    (option) =>
                      option.id === (bindingFor(slotId)?.physicalId ?? officialServoIds[index]),
                  ) && (
                    <option hidden value={String(bindingFor(slotId)?.physicalId)}>
                      {bindingFor(slotId)?.physicalId} · 当前绑定
                    </option>
                  )}
                  {bindingOptionsFor(index).map((option) => (
                    <option key={option.id} value={option.id}>
                      {option.label}
                    </option>
                  ))}
                </select>
              </div>
            ))}
          </div>
        )}
        {rangeView.value ? (
          <div class={style.rangePanel}>
            <div class={style.rangeHeader}>
              <span>肢体名</span>
              <span>最小角度限制</span>
              <span>最大角度限制</span>
            </div>
            {jointSlots.value.map((slotId) => {
              const range = rangeFor(slotId);
              return (
                <div class={style.rangeRow}>
                  <span>{slotId}</span>
                  <button
                    type="button"
                    class={
                      selectedRangeEndpoint.value?.slotId === slotId &&
                      selectedRangeEndpoint.value.endpoint === "min"
                        ? style.active
                        : undefined
                    }
                    onClick={() => (selectedRangeEndpoint.value = { slotId, endpoint: "min" })}
                  >
                    {(range.minRad ?? -Math.PI).toFixed(3)} rad
                  </button>
                  <button
                    type="button"
                    class={
                      selectedRangeEndpoint.value?.slotId === slotId &&
                      selectedRangeEndpoint.value.endpoint === "max"
                        ? style.active
                        : undefined
                    }
                    onClick={() => (selectedRangeEndpoint.value = { slotId, endpoint: "max" })}
                  >
                    {(range.maxRad ?? Math.PI).toFixed(3)} rad
                  </button>
                </div>
              );
            })}
          </div>
        ) : null}
        {!rangeView.value && secondaryPanel.value === "world" && (
          <div class={style.worldPositionPanel}>
            {(["x", "y", "z"] as const).map((axis, index) => (
              <div class={style.worldPositionRow}>
                <span>{axis}</span>
                <Slider
                  value={worldPositionDraft.value[index]}
                  min={-10}
                  max={10}
                  step={0.01}
                  progress={true}
                  progressCenter={0}
                  ariaLabel={`${axis} 世界位置`}
                  onInput={(event) => updateWorldPosition(index, event.value)}
                />
              </div>
            ))}
          </div>
        )}
        {!rangeView.value && secondaryPanel.value === "safety" && (
          <>
            <ul class={style.diagnosticList}>
              <li>
                <span>控制环 session</span>
                <b>{store.state.loop?.session ?? "—"}</b>
              </li>
              <li>
                <span>目标数量</span>
                <b>{store.state.loop?.targets.length ?? 0}</b>
              </li>
              <li>
                <span>传感器连续失败</span>
                <b>{store.state.loop?.sensorFailures ?? 0}</b>
              </li>
              <li
                class={style.logEntry}
                role="button"
                tabindex={0}
                onClick={() => void openLogs()}
                onKeydown={(event: KeyboardEvent) => {
                  if (event.key === "Enter" || event.key === " ") void openLogs();
                }}
              >
                <span>实机日志</span>
                <b>打开日志</b>
              </li>
            </ul>
          </>
        )}
      </article>
      <div
        class={[style.panelActions, !physicalConnected.value ? style.disabled : undefined]}
        v-show={!rangeView.value}
      >
        <button
          type="button"
          disabled={!physicalConnected.value}
          onClick={() => void setPoseZero()}
        >
          绑定姿态零位
        </button>
        <button
          type="button"
          disabled={!physicalConnected.value}
          onClick={() => void toggleAllTorque()}
        >
          {anyTorqueEnabled.value ? "全部力矩关" : "全部力矩开"}
        </button>
      </div>
      <article
        hidden={rangeView.value}
        aria-disabled={!physicalConnected.value}
        class={[style.panelCard, !physicalConnected.value ? style.disabled : undefined]}
      >
        <div class={style.servoToolbar}>
          <div class={style.servoIdList} aria-label="舵机编号">
            {debugServoIndexes.map((id) => (
              <button
                class={selectedId.value === id ? style.active : undefined}
                type="button"
                onClick={() => selectServo(id)}
              >
                {physicalIdForIndex(id - 1)}
              </button>
            ))}
          </div>
        </div>
        <div class={style.parameterGrid} aria-label="舵机状态">
          <div>
            <span>位置</span>
            <b>{formatParameter(selectedPosition.value, " rad")}</b>
          </div>
          <div>
            <span>速度</span>
            <b>—</b>
          </div>
          <div>
            <span>负载</span>
            <b>—</b>
          </div>
          <div>
            <span>电压</span>
            <b>—</b>
          </div>
          <div>
            <span>温度</span>
            <b>—</b>
          </div>
          <div>
            <span>电流</span>
            <b>—</b>
          </div>
        </div>
        <div class={style.controlGroup}>
          <div class={style.field}>
            目标位置
            <div class={style.targetRow}>
              <Slider
                value={Number(targetDraft.value) || 0}
                min={-Math.PI}
                max={Math.PI}
                step={0.001}
                progress={true}
                progressCenter={0}
                ariaLabel="目标位置"
                onInput={(event) => {
                  targetDraft.value = String(event.value);
                  void move(String(event.value));
                }}
              />
            </div>
          </div>
          <div class={style.actions}>
            <button class={style.primary} onClick={() => void move()}>
              移动
            </button>
            <button
              onClick={() => {
                if (bindingConflictsFor(selectedMachine.value).length) {
                  checkBindingConflict();
                  return;
                }
                if (store.state.loop) {
                  void sendAction({ kind: "power", on: !anyTorqueEnabled.value });
                } else {
                  void toggleAllTorque();
                }
              }}
            >
              {anyTorqueEnabled.value ? "力矩关" : "力矩开"}
            </button>
            <button onClick={() => void refresh()}>刷新状态</button>
          </div>
          <div class={style.actions}>
            <button onClick={() => send()}>发送微调目标</button>
            <button
              onClick={() => {
                if (bindingConflictsFor(selectedMachine.value).length) {
                  checkBindingConflict();
                  return;
                }
                void tick();
              }}
            >
              推进一拍
            </button>
            {store.state.loop && (
              <button class={style.danger} onClick={() => void closeLoop()}>
                停止控制环
              </button>
            )}
          </div>
        </div>
        <p class={style.muted}>速度、加速度和实时 telemetry 会在后端提供单舵机读数后恢复。</p>
      </article>
    </ScrollArea>
  );
});

export async function load() {
  const store = useDesktopStore();
  store.state.loading = true;
  store.state.error = null;
  try {
    [
      store.state.catalog,
      store.state.virtualMachines,
      store.state.physicalMachines,
      store.state.actions,
      store.state.autoReconnectEnabled,
      store.state.policies,
    ] = await Promise.all([
      desktopBackend().servoCatalog(),
      desktopBackend().virtualMachines(),
      desktopBackend().physicalMachines(),
      desktopBackend().actions(),
      desktopBackend().autoReconnectEnabled(),
      desktopBackend().policyList(),
    ]);
    store.state.selectedPolicyId =
      store.state.policies.find((policy) => policy.selected)?.id ?? null;
    store.state.selectedVirtualMachine =
      store.state.virtualMachines.find((machine) => machine.startupSelected)?.id ?? null;
    store.state.selectedVirtualMachineManual = false;
  } catch (error) {
    store.state.error = error instanceof Error ? error.message : String(error);
  } finally {
    store.state.loading = false;
  }
}

export async function createAction(name: string, kind: ActionRecord["kind"] = "pose") {
  const store = useDesktopStore();
  const action = await desktopBackend().createAction({ name, kind });
  store.state.actions.push(action);
  return action;
}

export async function deleteAction(id: string) {
  const store = useDesktopStore();
  await desktopBackend().deleteAction(id);
  store.state.actions = store.state.actions.filter((action) => action.id !== id);
}

export async function openLoop(servoIds: number[], physicalMachineId?: string, policyId?: string) {
  const store = useDesktopStore();
  if (!store.state.selectedVirtualMachine) throw new Error("请选择模拟机");
  store.state.loop = await desktopBackend().loopOpen({
    virtualMachineId: store.state.selectedVirtualMachine,
    servoIds,
    physicalMachineId,
    policyId,
  });
  return store.state.loop;
}

export async function sendAction(action: LoopAction) {
  const store = useDesktopStore();
  store.state.loop = await desktopBackend().loopAction(action);
  return store.state.loop;
}

export async function sendTargets(targets: number[]) {
  const store = useDesktopStore();
  await desktopBackend().loopCommand(targets);
  store.state.loop = await desktopBackend().loopStatus();
}

export async function tick() {
  const store = useDesktopStore();
  store.state.loop = await desktopBackend().loopTick();
  return store.state.loop;
}

export async function closeLoop() {
  const store = useDesktopStore();
  await desktopBackend().loopClose();
  store.state.loop = null;
}

export async function officialSceneModel() {
  return desktopBackend().officialSceneModel();
}

export async function officialSceneFrame(positions: number[]) {
  return desktopBackend().officialSceneFrame(positions);
}
