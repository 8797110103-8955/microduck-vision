import * as commands from "../../../generated/types";
import { computed, defineComponent, onMounted, onScopeDispose, ref, watch } from "vue";
import { useEventListener, useIntervalFn } from "@vueuse/core";
import type {
  ActionRecord,
  JointRange,
  PhysicalMachine,
  VirtualMachine,
} from "../../../generated/types";
import { showMessage } from "../../components/message";
import { ScrollArea } from "../../components/scroll-area";
import { Select } from "../../components/select";
import { Slider } from "../../components/slider";
import {
  requestDuckFocus,
  requestDefaultPose,
  requestRestoreDuckView,
  requestServoSelection,
  servoSelectionEvent,
} from "../../components/viewer-events";
import { useDesktopStore } from "../../store";
import style from "./style.module.scss";

export { actionKind, servoModel, SectionHeading } from "./helpers";

const officialServoIds = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];

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

type BindingDrafts = Record<string, number | null>;

export function bindingConflictsForDrafts(
  machine: VirtualMachine | undefined,
  drafts: BindingDrafts,
) {
  if (!machine) return [];
  const bindings = new Map(
    (machine.jointBindings ?? []).map((binding) => [binding.slotId, binding]),
  );
  for (const [slotId, physicalId] of Object.entries(drafts)) {
    if (physicalId === null) bindings.delete(slotId);
    else {
      const current = bindings.get(slotId);
      bindings.set(slotId, {
        slotId,
        physicalId,
        direction: current?.direction ?? 1,
        zeroOffsetRad: current?.zeroOffsetRad ?? 0,
        minRad: current?.minRad ?? null,
        maxRad: current?.maxRad ?? null,
      });
    }
  }
  const byId = new Map<number, string[]>();
  for (const binding of bindings.values()) {
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
 * The simulated machine owns the body-to-servo slot mapping.  A physical
 * machine adds installation-specific direction overrides alongside it.
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
  const rangeView = ref(false);
  const manualControl = computed(
    () => !!store.state.loop?.hasExecution && !store.state.loop.simulation,
  );
  const accessBusy = ref(false);
  const toggleManualControl = async () => {
    const physical = selectedPhysicalMachine.value;
    const virtual = selectedMachine.value;
    if (!physical?.connected || !virtual || accessBusy.value) return;
    accessBusy.value = true;
    try {
      const current = await commands.loopStatus();
      const input = {
        virtualMachineId: virtual.id,
        physicalMachineId: physical.id,
        access: manualControl.value ? ("read_only" as const) : ("control" as const),
        policyId: null,
      };
      store.state.loop = await (current
        ? commands.runtimeReconfigure(current.runtimeGeneration, current.configRevision, input)
        : commands.runtimeOpen(input));
      store.state.previewLoop = null;
    } catch (error) {
      showMessage(error instanceof Error ? error.message : String(error), { type: "warning" });
    } finally {
      accessBusy.value = false;
    }
  };
  const jointSlots = ref<string[]>([]);
  const jointRanges = ref<JointRange[]>([]);
  const selectedRangeEndpoints = ref<Record<string, "min" | "max">>({});
  const rangeEntryPose = ref<number[] | null>(null);
  const bindingConflictSignature = ref("");
  const bindingDrafts = ref<BindingDrafts>({});
  const pendingBindingSlots = ref<string[]>([]);
  let dismissPendingBindingMessage: (() => void) | undefined;
  let suppressServoSelectionEvent = false;
  let loopRequestInFlight = false;
  let loopPollingActive = false;
  const worldPositionDraft = ref<[number, number, number]>([0, 0, 0]);
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
  const poseZeroAllowed = computed(
    () =>
      physicalConnected.value && !(store.state.loop?.hasExecution && !store.state.loop.simulation),
  );
  const anyTorqueEnabled = computed(() => selectedPhysicalMachine.value?.torqueEnabled === true);
  const bindingDraftFor = (slotId: string) =>
    Object.prototype.hasOwnProperty.call(bindingDrafts.value, slotId)
      ? bindingDrafts.value[slotId]
      : undefined;
  const bindingFor = (slotId: string) => {
    const machineBinding = debugBindingFor(selectedMachine.value, slotId);
    const draft = bindingDraftFor(slotId);
    if (draft === undefined) return machineBinding;
    if (draft === null) return undefined;
    return {
      slotId,
      physicalId: draft,
      direction: machineBinding?.direction ?? 1,
      zeroOffsetRad: machineBinding?.zeroOffsetRad ?? 0,
      minRad: machineBinding?.minRad ?? null,
      maxRad: machineBinding?.maxRad ?? null,
    };
  };
  const applyDefaultSelection = () => {
    if (store.state.selectedVirtualMachineManual) return;
    const defaultId = defaultDebugVirtualMachineId(
      store.state.virtualMachines,
      store.state.physicalMachines,
    );
    if (defaultId) store.state.selectedVirtualMachine = defaultId;
  };
  useEventListener(window, servoSelectionEvent, onServoSelection);
  const refreshLoopStatus = async () => {
    if (!loopPollingActive || !store.state.loop || loopRequestInFlight) return;
    loopRequestInFlight = true;
    const machineId = selectedPhysicalMachine.value?.id;
    const currentLoop = store.state.loop;
    try {
      const status = await commands.loopStatus();
      if (!status) return;
      const source = status.readSources?.find((candidate) => candidate.id === machineId);
      const physicalStatus = source
        ? {
            ...status,
            positions: source.positions,
            sensorFailures: source.health === "offline" ? 1 : 0,
            lastError: source.lastError,
          }
        : status;
      if (
        loopPollingActive &&
        selectedPhysicalMachine.value?.id === machineId &&
        store.state.loop === currentLoop
      )
        store.state.loop = physicalStatus;
    } catch (error) {
      if (loopPollingActive) console.warn("读取真机姿态失败", error);
    } finally {
      loopRequestInFlight = false;
    }
  };
  const { pause: stopLoopPolling, resume: startLoopPolling } = useIntervalFn(
    () => void refreshLoopStatus(),
    180,
    { immediate: false },
  );
  const restartReadLoop = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!store.state.loop || !machine?.connected || manualControl.value) return;
    await closeLoop();
    await openLoop(machine.id);
    await refreshLoopStatus();
  };
  const refreshVirtualMachineCache = async () => {
    const machines = await commands.virtualMachines();
    store.state.virtualMachines = machines;
    return machines.find((machine) => machine.id === store.state.selectedVirtualMachine);
  };
  const announcePendingBindingConflict = (slotId: string, conflicts: string[]) => {
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = showMessage(
      `绑定冲突，${slotId}，${conflicts.join(", ")}, 等待其他绑定解决。`,
      {
        type: "warning",
        duration: 4_000,
      },
    );
    bindingConflictSignature.value = conflicts.join("|");
  };
  const syncBindingDrafts = async () => {
    if (!Object.keys(bindingDrafts.value).length) return;
    const machine = await refreshVirtualMachineCache();
    if (!machine) return;
    const conflicts = bindingConflictsForDrafts(machine, bindingDrafts.value);
    if (conflicts.length) {
      pendingBindingSlots.value = [
        ...pendingBindingSlots.value,
        ...Object.keys(bindingDrafts.value).filter(
          (slotId) => !pendingBindingSlots.value.includes(slotId),
        ),
      ];
      checkBindingConflict();
      return;
    }
    const drafts = Object.entries(bindingDrafts.value);
    for (const [slotId, physicalId] of drafts) {
      const current = machine.jointBindings?.find((binding) => binding.slotId === slotId);
      if ((current?.physicalId ?? null) === physicalId) continue;
      await commands.virtualMachineBind({
        virtualMachineId: machine.id,
        slotId,
        physicalId,
      });
    }
    await refreshVirtualMachineCache();
    bindingDrafts.value = {};
    pendingBindingSlots.value = [];
    bindingConflictSignature.value = "";
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = undefined;
    await restartReadLoop();
  };

  onMounted(async () => {
    loopPollingActive = true;
    if (!store.state.catalog.length && !store.state.loading) await load();
    if (!loopPollingActive) return;
    jointSlots.value = await commands.officialJointSlots();
    jointRanges.value = await commands.officialJointRanges();
    if (!loopPollingActive) return;
    applyDefaultSelection();
    checkBindingConflict();
    if (selectedPhysicalMachine.value?.connected) {
      try {
        await openLoop(selectedPhysicalMachine.value.id);
        store.state.error = null;
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        store.state.error = null;
        showMessage(`调试读取 loop 启动失败：${message}`, {
          type: "warning",
          duration: 6_000,
        });
      }
    }
    startLoopPolling();
    void refreshLoopStatus();
  });
  onScopeDispose(() => {
    stopLoopPolling();
    loopPollingActive = false;
    loopRequestInFlight = false;
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = undefined;
    store.state.rangePreview = null;
    store.state.debugPose = null;
    if (store.state.loop) void closeLoop();
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
      const updated = await commands.virtualMachineSetWorldPosition(machine.id, position);
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
  function checkBindingConflict() {
    const conflicts = bindingConflictsForDrafts(selectedMachine.value, bindingDrafts.value);
    const signature = conflicts.join("|");
    if (!signature) {
      bindingConflictSignature.value = "";
      dismissPendingBindingMessage?.();
      dismissPendingBindingMessage = undefined;
      return;
    }
    if (bindingConflictSignature.value === signature) return;
    bindingConflictSignature.value = signature;
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = showMessage(
      `绑定冲突：${conflicts.map((item) => item.replace(":", " → ")).join("；")}`,
      {
        type: "warning",
        duration: 6_000,
      },
    );
  }
  watch(
    () => store.state.selectedVirtualMachine,
    () => {
      bindingDrafts.value = {};
      pendingBindingSlots.value = [];
      dismissPendingBindingMessage?.();
      dismissPendingBindingMessage = undefined;
      bindingConflictSignature.value = "";
      checkBindingConflict();
    },
  );
  const promptPendingBindingSelection = () => {
    if (!pendingBindingSlots.value.length) return;
    dismissPendingBindingMessage?.();
    const slots = [...pendingBindingSlots.value];
    dismissPendingBindingMessage = showMessage("仍有绑定冲突待处理", {
      type: "warning",
      duration: 0,
      dismissible: false,
      action: {
        label: slots.join("、"),
        onClick: () => {
          const index = jointSlots.value.indexOf(slots[0]);
          if (index >= 0) selectServo(index + 1);
        },
      },
    });
  };
  const clearServoSelection = () => {
    if (selectedId.value === null) {
      promptPendingBindingSelection();
      return;
    }
    selectedId.value = null;
    suppressServoSelectionEvent = true;
    requestServoSelection(null);
    suppressServoSelectionEvent = false;
    promptPendingBindingSelection();
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
      clearServoSelection();
      return;
    }
    if (typeof event.detail !== "number") return;
    selectedId.value = Math.max(1, Math.min(15, event.detail));
    checkBindingConflict();
  }
  const directionFor = (slotId: string) =>
    selectedPhysicalMachine.value?.jointDirections?.find((binding) => binding.slotId === slotId)
      ?.direction ??
    bindingFor(slotId)?.direction ??
    1;
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
    const physicalId = value ? Number(value) : null;
    bindingDrafts.value = { ...bindingDrafts.value, [slotId]: physicalId };
    const conflicts = bindingConflictsForDrafts(selectedMachine.value, bindingDrafts.value);
    if (conflicts.length) {
      if (!pendingBindingSlots.value.includes(slotId)) {
        pendingBindingSlots.value = [...pendingBindingSlots.value, slotId];
      }
      announcePendingBindingConflict(slotId, conflicts);
      return;
    }
    try {
      await syncBindingDrafts();
      store.state.error = null;
      checkBindingConflict();
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const updateDirection = async (slotId: string, direction: number) => {
    const physical = selectedPhysicalMachine.value;
    const virtualMachineId = store.state.selectedVirtualMachine;
    if (!physical?.connected || !virtualMachineId) return;
    const index = jointSlots.value.indexOf(slotId);
    const physicalId = bindingFor(slotId)?.physicalId ?? officialServoIds[index];
    if (physicalId === undefined) return;
    try {
      const updated = await commands.physicalMachineBind({
        physicalMachineId: physical.id,
        virtualMachineId,
        slotId,
        physicalId,
        direction,
      });
      const physicalIndex = store.state.physicalMachines.findIndex(
        (item) => item.id === updated.id,
      );
      if (physicalIndex >= 0) store.state.physicalMachines[physicalIndex] = updated;
      store.state.physicalMachines = await commands.physicalMachines();
      await restartReadLoop();
      store.state.error = null;
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const setPoseZero = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!machine?.connected || !poseZeroAllowed.value) return;
    if (bindingConflictsForDrafts(selectedMachine.value, bindingDrafts.value).length) {
      checkBindingConflict();
      return;
    }
    try {
      const updated = await commands.physicalMachineSetPoseZero(
        machine.id,
        store.state.selectedVirtualMachine,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.physicalMachines[index] = updated;
      store.state.virtualMachines = await commands.virtualMachines();
      if (store.state.loop) {
        await closeLoop();
        await openLoop(machine.id);
        await refreshLoopStatus();
      }
      store.state.error = null;
      showMessage("已绑定当前姿态为零位", { type: "info" });
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const toggleAllTorque = async () => {
    const machine = selectedPhysicalMachine.value;
    const virtualMachineId = store.state.selectedVirtualMachine;
    if (!machine?.connected || !virtualMachineId || !manualControl.value || accessBusy.value)
      return;
    if (bindingConflictsForDrafts(selectedMachine.value, bindingDrafts.value).length) {
      checkBindingConflict();
      return;
    }
    try {
      const updated = await commands.physicalMachineSetTorque(
        machine.id,
        !anyTorqueEnabled.value,
        virtualMachineId,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.physicalMachines[index] = updated;
      await refreshLoopStatus();
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    }
  };
  const rangeFor = (slotId: string) =>
    jointRanges.value.find((range) => range.slotId === slotId) ?? {
      slotId,
      minRad: -Math.PI,
      maxRad: Math.PI,
    };
  const rangeStateFor = (slotId: string) =>
    store.state.jointRangeStates.find((state) => state.slotId === slotId);
  const updateRangePreview = () => {
    store.state.rangePreview = jointSlots.value.map((slotId) => {
      const range = rangeFor(slotId);
      const endpoint = selectedRangeEndpoints.value[slotId];
      if (endpoint === "min") return range.minRad ?? -Math.PI;
      if (endpoint === "max") return range.maxRad ?? Math.PI;
      return 0;
    });
    requestDuckFocus();
  };
  const toggleRangeView = () => {
    if (rangeView.value) {
      rangeView.value = false;
      store.state.rangePreview = null;
      if (rangeEntryPose.value) store.state.debugPose = [...rangeEntryPose.value];
      rangeEntryPose.value = null;
      return;
    }
    rangeEntryPose.value = [
      ...(store.state.loop?.positions ?? store.state.debugPose ?? Array(15).fill(0)),
    ];
    selectedRangeEndpoints.value = {};
    updateRangePreview();
    rangeView.value = true;
  };
  const selectRangeEndpoint = (slotId: string, endpoint: "min" | "max") => {
    const current = selectedRangeEndpoints.value[slotId];
    const next = { ...selectedRangeEndpoints.value };
    if (current === endpoint) delete next[slotId];
    else next[slotId] = endpoint;
    selectedRangeEndpoints.value = next;
    updateRangePreview();
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
      </div>
      <div class={style.systemActions} aria-label="调试全局操作">
        <button
          type="button"
          disabled={!physicalConnected.value || accessBusy.value}
          onClick={() => void toggleManualControl()}
        >
          {manualControl.value ? "返回只读" : "切换为手动控制"}
        </button>
        <button
          class={style.poseAction}
          type="button"
          disabled={!poseZeroAllowed.value || accessBusy.value}
          title="读取当前实机姿态，并将它绑定为模拟机的姿态零位"
          onClick={() => void setPoseZero()}
        >
          绑定
        </button>
        <button
          class={style.poseAction}
          type="button"
          title="将当前模拟机姿态还原到默认零位"
          onClick={requestDefaultPose}
        >
          还原
        </button>
        <button
          class={style.viewAction}
          type="button"
          title="将视图聚焦到当前选中的模拟机"
          onClick={requestDuckFocus}
        >
          聚焦
        </button>
        <button
          class={style.viewAction}
          type="button"
          title="将视图恢复到聚焦当前模拟机之前的位置"
          onClick={requestRestoreDuckView}
        >
          还原
        </button>
        <button
          class={style.torqueAction}
          type="button"
          disabled={!physicalConnected.value || !manualControl.value || accessBusy.value}
          title="打开或关闭当前实机全部舵机的扭矩输出"
          onClick={() => void toggleAllTorque()}
        >
          {anyTorqueEnabled.value ? "全部力矩关" : "全部力矩开"}
        </button>
      </div>
      <article class={style.panelCard}>
        <div class={style.panelHeader}>
          <button
            class={style.panelTitleButton}
            type="button"
            title={rangeView.value ? "切换到肢体绑定" : "切换到肢体范围"}
            onClick={toggleRangeView}
          >
            {rangeView.value ? "肢体范围" : "肢体绑定"}
          </button>
        </div>
        {rangeView.value ? (
          <>
            <div class={style.rangePanel}>
              <div class={style.rangeHeader}>
                <span>肢体名</span>
                <span
                  class={
                    store.state.jointRangeStates.some((state) => state.belowMin)
                      ? style.rangeMinTitle
                      : undefined
                  }
                >
                  最小角度限制
                </span>
                <span
                  class={
                    store.state.jointRangeStates.some((state) => state.aboveMax)
                      ? style.rangeMaxTitle
                      : undefined
                  }
                >
                  最大角度限制
                </span>
              </div>
              {jointSlots.value.map((slotId) => {
                const range = rangeFor(slotId);
                const rangeState = rangeStateFor(slotId);
                return (
                  <div class={style.rangeRow}>
                    <span>{slotId}</span>
                    <button
                      type="button"
                      class={[
                        selectedRangeEndpoints.value[slotId] === "min" ? style.active : undefined,
                        rangeState?.belowMin ? style.rangeMinExceeded : undefined,
                      ]}
                      onClick={() => selectRangeEndpoint(slotId, "min")}
                    >
                      {(range.minRad ?? -Math.PI).toFixed(3)} rad
                    </button>
                    <button
                      type="button"
                      class={[
                        selectedRangeEndpoints.value[slotId] === "max" ? style.active : undefined,
                        rangeState?.aboveMax ? style.rangeMaxExceeded : undefined,
                      ]}
                      onClick={() => selectRangeEndpoint(slotId, "max")}
                    >
                      {(range.maxRad ?? Math.PI).toFixed(3)} rad
                    </button>
                  </div>
                );
              })}
            </div>
          </>
        ) : (
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
                <div class={style.bindingControl}>
                  <Select
                    value={String(bindingFor(slotId)?.physicalId ?? officialServoIds[index])}
                    onClick={(event: MouseEvent) => {
                      event.stopPropagation();
                      selectBinding(index + 1);
                    }}
                    onChange={(event: Event) => {
                      const value =
                        event.currentTarget instanceof HTMLSelectElement
                          ? event.currentTarget.value
                          : "";
                      void updateBinding(slotId, value);
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
                  </Select>
                  <div
                    class={style.directionButtons}
                    onClick={(event: MouseEvent) => event.stopPropagation()}
                  >
                    <button
                      type="button"
                      disabled={!physicalConnected.value}
                      onClick={() =>
                        void updateDirection(slotId, directionFor(slotId) === 1 ? -1 : 1)
                      }
                    >
                      {directionFor(slotId)}
                    </button>
                  </div>
                </div>
              </div>
            ))}
          </div>
        )}
      </article>
      <article class={[style.panelCard, style.worldCard]}>
        <div class={style.panelHeader}>
          <strong>世界位置</strong>
        </div>
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
      commands.servoCatalog(),
      commands.virtualMachines(),
      commands.physicalMachines(),
      commands.actions(),
      commands.autoReconnectEnabled(),
      commands.policyList(),
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
  const action = await commands.actionCreate({ name, kind });
  store.state.actions.push(action);
  return action;
}

export async function deleteAction(id: string) {
  const store = useDesktopStore();
  await commands.actionDelete(id);
  store.state.actions = store.state.actions.filter((action) => action.id !== id);
}

export async function openLoop(physicalMachineId: string) {
  const store = useDesktopStore();
  if (!store.state.selectedVirtualMachine) throw new Error("请选择模拟机");
  store.state.loop = await commands.runtimeOpen({
    virtualMachineId: store.state.selectedVirtualMachine,
    physicalMachineId,
    access: "read_only",
    policyId: null,
  });
  return store.state.loop;
}

export async function closeLoop() {
  const store = useDesktopStore();
  await commands.runtimeClose(store.state.loop?.runtimeGeneration ?? 0, "readOnlySource");
  const remaining = await commands.loopStatus();
  // Releasing the debug source must preserve a physical execution owned by another page.
  store.state.loop = remaining?.simulation ? null : remaining;
  store.state.jointRangeStates = [];
}
