import { computed, defineComponent, onMounted, onScopeDispose, ref, watch } from "vue";
import type { LoopStatus, TrainingFrame } from "../../generated/types";
import * as commands from "../../generated/types";
import { type DisplayFrame, OfficialDuckStage } from "../components/duck-stage";
import { pruneWorldCache, useDesktopStore } from "../store";
import { debugBindingFor, debugBindingMachineFor, debugPhysicalMachineFor } from "./debug";

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

export const SceneStage = defineComponent(
  (props: { status?: LoopStatus; training?: TrainingFrame | null }) => {
    const store = useDesktopStore();
    const viewerReady = ref(false);
    const scene = ref<Awaited<ReturnType<typeof commands.officialSceneModel>> | null>(null);
    const frame = ref<DisplayFrame>();
    const debug = computed(() => store.state.tab === "debug");
    const activeLoop = computed(() => props.status);
    const previewPhysicalId = computed(() =>
      store.state.tab === "preview" ? store.state.requestedPhysicalMachine : null,
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
      if (
        props.training !== undefined ||
        !viewerReady.value ||
        (debug.value && store.state.debugMappingPending)
      )
        return;
      const simulation = displayingSimulation.value ? activeLoop.value?.simulation : undefined;
      if (simulation) {
        pendingFramePositions = null;
        frame.value = { kind: "simulation", value: simulation };
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
        frame.value = { kind: "kinematic", value: { ...rawFrame, markers: markerFrame } };
        if (
          debug.value &&
          viewerReady.value &&
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

    onMounted(async () => {
      const loaded = await commands.officialSceneModel();
      if (stageActive) scene.value = loaded;
    });

    onScopeDispose(() => {
      stageActive = false;
      pendingFramePositions = null;
    });

    watch([frameContext, activePositions, displayFrame], ([, positions]) => {
      void refreshFrame(positions ?? []);
    });

    const tabs = computed(() =>
      [
        { id: "preview" as const, label: "ONNX", active: store.state.tab === "preview" },
        { id: "training" as const, label: "训练", active: store.state.tab === "training" },
        { id: "debug" as const, label: "调试", active: debug.value },
      ].map(({ id, label, active }) => ({
        label,
        active,
        select: () => (store.state.tab = active ? "home" : id),
      })),
    );

    const trainingView = computed(() => {
      const sample = props.training;
      const model = sample?.previewScene ?? scene.value;
      const defaultFrame = model?.defaultFrame;
      const display: DisplayFrame | undefined = sample
        ? { kind: "simulation", value: sample, preserveWorld: true }
        : defaultFrame && { kind: "kinematic", value: defaultFrame };
      return {
        scene: model,
        frame: display,
        worldPositions: [[0, 0, 0] as [number, number, number]],
        worldPoses: [],
        selected: 0,
        labels: sample
          ? [{ id: sample.runId, name: `${sample.modelKind} · 环境 ${sample.environmentIndex}` }]
          : [],
        markers: [],
        highlightBodies: [],
        debug: false,
        tabs: tabs.value,
      };
    });

    const view = computed(() => ({
      scene: scene.value,
      frame: frame.value,
      world: displayWorld.value,
      worldPositions: worldPositions.value,
      worldPoses: worldPoses.value,
      selected: selectedSceneIndex.value,
      labels: labels.value,
      markers: markers.value.map((marker) => ({
        ...marker,
        appearance: markerState(marker.physicalId),
      })),
      highlightBodies: markers.value
        .filter((marker) => {
          const state = rangeStateBySlot.value.get(marker.name);
          return state?.belowMin || state?.aboveMax;
        })
        .map((marker) => marker.body),
      debug: debug.value,
      tabs: tabs.value,
    }));

    return () => (
      <OfficialDuckStage
        view={props.training === undefined ? view.value : trainingView.value}
        onReady={() => {
          viewerReady.value = true;
          void refreshFrame(activePositions.value ?? []);
        }}
        onDefaultPose={() => {
          pendingFramePositions = null;
        }}
        onSelectDuck={(index) => {
          if (props.training !== undefined) return;
          const machine = sceneMachines.value[index];
          if (!machine) return;
          store.state.selectedVirtualMachine = machine.id;
          store.state.selectedVirtualMachineManual = true;
        }}
      />
    );
  },
  { props: ["status", "training"] },
);
