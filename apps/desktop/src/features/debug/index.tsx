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

export function bindingConflictsFor(machine: Pick<VirtualMachine, "jointBindings"> | undefined) {
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
  machine: Pick<VirtualMachine, "jointBindings"> | undefined,
  drafts: BindingDrafts,
) {
  if (!machine) return [];
  const bindings = new Map(
    (machine.jointBindings ?? []).map((binding) => [binding.slotId, binding]),
  );
  for (const [slotId, physicalId] of Object.entries(drafts)) {
    if (physicalId === null) bindings.delete(slotId);
    else {
      bindings.set(slotId, {
        slotId,
        physicalId,
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

export function debugBindingFor(
  machine: Pick<VirtualMachine, "jointBindings"> | undefined,
  slotId: string,
) {
  return machine?.jointBindings?.find((binding) => binding.slotId === slotId);
}

export function debugBindingMachineFor(
  virtual: VirtualMachine | undefined,
  physical: PhysicalMachine | null | undefined,
) {
  if (!physical) return virtual;
  const virtualBindings = virtual?.jointBindings ?? [];
  return {
    id: physical.id,
    jointBindings: virtualBindings.map((binding) => {
      const calibration = physical.jointBindings?.find(
        (candidate) => candidate.slotId === binding.slotId,
      );
      return {
        ...binding,
        zeroOffsetRad: calibration?.zeroOffsetRad ?? 0,
        minRad: calibration?.minRad ?? null,
        maxRad: calibration?.maxRad ?? null,
        direction:
          physical.jointDirections?.find((direction) => direction.slotId === binding.slotId)
            ?.direction ?? 1,
      };
    }),
  };
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
  const bindingBusy = ref(false);
  watch(
    bindingBusy,
    (pending) => {
      store.state.debugMappingPending = pending;
    },
    { flush: "sync" },
  );
  onScopeDispose(() => {
    store.state.debugMappingPending = false;
  });
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
  const readLoopReady = ref(false);
  let readLoopSync = Promise.resolve();
  let readLoopVirtualId: string | null = null;
  let readLoopPhysicalId: string | null = null;
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
  const bindingMachine = computed(() =>
    debugBindingMachineFor(selectedMachine.value, selectedPhysicalMachine.value),
  );
  const physicalConnected = computed(() => selectedPhysicalMachine.value?.connected === true);
  const mappingOwnedByExecution = computed(
    () =>
      !!selectedPhysicalMachine.value &&
      store.state.loop?.activeBody?.physicalMachineId === selectedPhysicalMachine.value.id,
  );
  const mappingEditDisabled = computed(() => bindingBusy.value || mappingOwnedByExecution.value);
  const resumeLivePose = () => {
    rangeView.value = false;
    rangeEntryPose.value = null;
    store.state.rangePreview = null;
    store.state.debugPose = null;
  };
  const poseZeroAllowed = computed(
    () => physicalConnected.value && !store.state.loop?.activeBody?.physicalMachineId,
  );
  const encoderCalibrationAllowed = computed(
    () =>
      poseZeroAllowed.value &&
      selectedPhysicalMachine.value?.census?.some((servo) => servo.model === "hd1910") === true,
  );
  const bindingDraftFor = (slotId: string) =>
    Object.prototype.hasOwnProperty.call(bindingDrafts.value, slotId)
      ? bindingDrafts.value[slotId]
      : undefined;
  const bindingFor = (slotId: string) => {
    const machineBinding = debugBindingFor(bindingMachine.value, slotId);
    const direction =
      machineBinding && "direction" in machineBinding ? machineBinding.direction : 1;
    const zeroOffsetRad =
      machineBinding && "zeroOffsetRad" in machineBinding ? machineBinding.zeroOffsetRad : 0;
    const minRad = machineBinding && "minRad" in machineBinding ? machineBinding.minRad : null;
    const maxRad = machineBinding && "maxRad" in machineBinding ? machineBinding.maxRad : null;
    const draft = bindingDraftFor(slotId);
    if (draft === undefined) return machineBinding;
    if (draft === null) return undefined;
    return {
      slotId,
      physicalId: draft,
      direction,
      zeroOffsetRad,
      minRad,
      maxRad,
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
      if (
        loopPollingActive &&
        selectedPhysicalMachine.value?.id === machineId &&
        store.state.loop?.runtimeGeneration === currentLoop.runtimeGeneration
      )
        store.applyLoopStatus(status);
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
    const virtual = selectedMachine.value;
    const machine = bindingMachine.value;
    if (!machine || !virtual) return;
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
    const updated = await commands.virtualMachineBindings(
      virtual.id,
      drafts.map(([slotId, physicalId]) => ({ slotId, physicalId })),
    );
    const index = store.state.virtualMachines.findIndex((entry) => entry.id === updated.id);
    if (index >= 0) store.state.virtualMachines[index] = updated;
    bindingDrafts.value = {};
    pendingBindingSlots.value = [];
    bindingConflictSignature.value = "";
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = undefined;
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
    readLoopReady.value = true;
    startLoopPolling();
    void refreshLoopStatus();
  });
  const reportReadLoopError = (error: unknown) => {
    if (!loopPollingActive) return;
    const message = error instanceof Error ? error.message : String(error);
    showMessage(`调试读取 loop 启动失败：${message}`, { type: "warning", duration: 6_000 });
  };
  // Serialize connection/selection changes and disposal so an in-flight open cannot leave
  // a source behind after the page exits or attach a previous machine after a new one.
  watch(
    [
      readLoopReady,
      () => selectedMachine.value?.id,
      () => (selectedPhysicalMachine.value?.connected ? selectedPhysicalMachine.value.id : null),
    ],
    () => {
      readLoopSync = readLoopSync
        .then(async () => {
          if (!loopPollingActive || !readLoopReady.value) return;
          const virtualId = selectedMachine.value?.id ?? null;
          const physicalId = selectedPhysicalMachine.value?.connected
            ? selectedPhysicalMachine.value.id
            : null;
          if (readLoopPhysicalId === physicalId && readLoopVirtualId === virtualId) return;
          if (
            readLoopPhysicalId &&
            store.state.loop?.readSources.some((source) => source.id === readLoopPhysicalId)
          ) {
            await closeLoop();
          }
          readLoopPhysicalId = null;
          readLoopVirtualId = null;
          if (!physicalId || !virtualId) return;
          await openLoop(physicalId, virtualId);
          readLoopPhysicalId = physicalId;
          readLoopVirtualId = virtualId;
        })
        .catch(reportReadLoopError);
    },
  );
  onScopeDispose(() => {
    stopLoopPolling();
    loopPollingActive = false;
    loopRequestInFlight = false;
    dismissPendingBindingMessage?.();
    dismissPendingBindingMessage = undefined;
    store.state.rangePreview = null;
    store.state.debugPose = null;
    readLoopSync = readLoopSync
      .then(async () => {
        if (
          readLoopPhysicalId &&
          store.state.loop?.readSources.some((source) => source.id === readLoopPhysicalId)
        ) {
          await closeLoop();
        }
      })
      .catch((error) => console.warn("关闭调试读取 loop 失败", error));
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
    const conflicts = bindingConflictsForDrafts(bindingMachine.value, bindingDrafts.value);
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
    () => bindingMachine.value?.id,
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
  const directionFor = (slotId: string) => {
    const binding = bindingFor(slotId);
    return (
      selectedPhysicalMachine.value?.jointDirections?.find(
        (installation) => installation.slotId === slotId,
      )?.direction ?? (binding && "direction" in binding ? binding.direction : 1)
    );
  };
  const bindingOptionsFor = (index: number) => {
    const officialId = officialServoIds[index];
    const census = selectedPhysicalMachine.value?.census ?? [];
    const savedId = bindingFor(jointSlots.value[index])?.physicalId;
    return [
      { id: officialId, label: `${officialId} · 官方默认` },
      ...(savedId !== undefined &&
      savedId !== officialId &&
      !census.some((servo) => servo.servoId === savedId)
        ? [{ id: savedId, label: `${savedId} · 已保存` }]
        : []),
      ...census
        .filter((servo) => servo.servoId !== officialId)
        .toSorted((a, b) => a.servoId - b.servoId)
        .map((servo) => ({ id: servo.servoId, label: `${servo.servoId} · ${servo.model}` })),
    ];
  };
  const updateBinding = async (slotId: string, value: string) => {
    const virtualMachineId = store.state.selectedVirtualMachine;
    if (!virtualMachineId || mappingEditDisabled.value) return;
    const physicalId = value ? Number(value) : null;
    bindingDrafts.value = { ...bindingDrafts.value, [slotId]: physicalId };
    const conflicts = bindingConflictsForDrafts(bindingMachine.value, bindingDrafts.value);
    if (conflicts.length) {
      if (!pendingBindingSlots.value.includes(slotId)) {
        pendingBindingSlots.value = [...pendingBindingSlots.value, slotId];
      }
      announcePendingBindingConflict(slotId, conflicts);
      return;
    }
    bindingBusy.value = true;
    try {
      resumeLivePose();
      await syncBindingDrafts();
      store.state.error = null;
      checkBindingConflict();
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    } finally {
      bindingBusy.value = false;
    }
  };
  const updateDirection = async (slotId: string, direction: number) => {
    const physical = selectedPhysicalMachine.value;
    const virtualMachineId = store.state.selectedVirtualMachine;
    if (!physical?.connected || !virtualMachineId || mappingEditDisabled.value) return;
    bindingBusy.value = true;
    try {
      resumeLivePose();
      const updated = await commands.physicalMachineBind({
        physicalMachineId: physical.id,
        slotId,
        direction,
      });
      const physicalIndex = store.state.physicalMachines.findIndex(
        (item) => item.id === updated.id,
      );
      if (physicalIndex >= 0) store.state.physicalMachines[physicalIndex] = updated;
      store.state.error = null;
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    } finally {
      bindingBusy.value = false;
    }
  };
  const setPoseZero = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!machine?.connected || !poseZeroAllowed.value || bindingBusy.value) return;
    if (bindingConflictsForDrafts(bindingMachine.value, bindingDrafts.value).length) {
      checkBindingConflict();
      return;
    }
    bindingBusy.value = true;
    try {
      resumeLivePose();
      const updated = await commands.physicalMachineSetPoseZero(
        machine.id,
        store.state.selectedVirtualMachine,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === updated.id);
      if (index >= 0) store.state.physicalMachines[index] = updated;
      store.state.error = null;
      showMessage("已绑定当前姿态为零位", { type: "info" });
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    } finally {
      bindingBusy.value = false;
    }
  };
  const centerEncoders = async () => {
    const machine = selectedPhysicalMachine.value;
    if (!machine || !encoderCalibrationAllowed.value || bindingBusy.value) return;
    if (bindingConflictsForDrafts(bindingMachine.value, bindingDrafts.value).length) {
      checkBindingConflict();
      return;
    }
    bindingBusy.value = true;
    try {
      resumeLivePose();
      const result = await commands.physicalMachineCenterEncoders(
        machine.id,
        store.state.selectedVirtualMachine,
      );
      const index = store.state.physicalMachines.findIndex((item) => item.id === result.machine.id);
      if (index >= 0) store.state.physicalMachines[index] = result.machine;
      store.state.error = result.error;
      showMessage(
        result.error
          ? `已校准 ${result.calibratedServoIds.length} 个电机，后续校准停止：${result.error}`
          : `已校准 ${result.calibratedServoIds.length} 个电机的编码器中位，保留当前关节姿态`,
        { type: result.error ? "warning" : "info", duration: 8_000 },
      );
    } catch (error) {
      store.state.error = error instanceof Error ? error.message : String(error);
    } finally {
      bindingBusy.value = false;
    }
  };
  const rangeFor = (slotId: string) =>
    jointRanges.value.find((range) => range.slotId === slotId) ?? {
      slotId,
      minRad: -Math.PI,
      maxRad: Math.PI,
    };
  const rangeStateFor = (slotId: string) =>
    store.state.loop?.jointRangeStates.find((state) => state.slotId === slotId);
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
      ...(store.state.loop?.readSources.find(
        (source) => source.id === selectedPhysicalMachine.value?.id,
      )?.positions ??
        (store.state.loop?.activeBody?.physicalMachineId
          ? store.state.loop.positions
          : store.state.debugPose) ??
        Array(15).fill(0)),
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
        <span>只读</span>
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
          class={style.poseAction}
          type="button"
          disabled={!poseZeroAllowed.value || bindingBusy.value}
          title="将当前实机姿态保存为这台真机的零位"
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
      </div>
      {selectedPhysicalMachine.value?.census?.some((servo) => servo.model === "hd1910") && (
        <article class={style.panelCard}>
          <div class={style.panelHeader}>
            <strong>实机编码器中位</strong>
            <button
              type="button"
              disabled={!encoderCalibrationAllowed.value || bindingBusy.value}
              title="将已绑定在线电机的当前位置校到编码器中间，并迁移这台实机的软件零位"
              onClick={() => void centerEncoders()}
            >
              {bindingBusy.value ? "处理中…" : "校准编码器中位"}
            </button>
          </div>
          <p>托住机身并保持肢体静止。校准会关闭力矩并保存到电机，保留当前关节角度与方向。</p>
          {mappingOwnedByExecution.value && <p>请先在 ONNX 切回模拟机，再执行校准。</p>}
        </article>
      )}
      <article class={style.panelCard}>
        {mappingOwnedByExecution.value && <p>请先停止这台实机的控制环，再修改肢体映射或方向。</p>}
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
                    store.state.loop?.jointRangeStates.some((state) => state.belowMin)
                      ? style.rangeMinTitle
                      : undefined
                  }
                >
                  最小角度限制
                </span>
                <span
                  class={
                    store.state.loop?.jointRangeStates.some((state) => state.aboveMax)
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
                    disabled={mappingEditDisabled.value}
                    value={
                      bindingFor(slotId)?.physicalId == null
                        ? ""
                        : String(bindingFor(slotId)?.physicalId)
                    }
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
                    <option value="">未绑定</option>
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
                      aria-label={`${slotId} 方向`}
                      disabled={!physicalConnected.value || mappingEditDisabled.value}
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

export async function openLoop(physicalMachineId: string, virtualMachineId?: string) {
  const store = useDesktopStore();
  const virtualId = virtualMachineId ?? store.state.selectedVirtualMachine;
  if (!virtualId) throw new Error("请选择模拟机");
  store.applyLoopStatus(
    await commands.runtimeOpen({
      virtualMachineId: virtualId,
      physicalMachineId,
      access: "read_only",
      policyId: null,
    }),
  );
  return store.state.loop;
}

export async function closeLoop() {
  const store = useDesktopStore();
  await commands.runtimeClose(store.state.loop?.runtimeGeneration ?? 0, "readOnlySource");
  const remaining = await commands.loopStatus();
  // Releasing the debug source must preserve a physical execution owned by another page.
  store.state.loop = remaining;
}
