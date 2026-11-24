import * as commands from "../generated/types";
import { reactive } from "vue";
import { defineStore } from "pinia";
import type { PreviewInput, ServoModelKey, TrainingFrame, WorldState } from "../generated/types";

const selectedServoStorageKey = "microduck.selected-servo";
const worldCacheStorageKey = "microduck.world-cache";
const isRecord = (value: unknown): value is Record<string, unknown> =>
  !!value && typeof value === "object" && !Array.isArray(value);

function isWorldState(value: unknown): value is WorldState {
  if (!isRecord(value)) return false;
  const candidate = value;
  return (
    Array.isArray(candidate.position) &&
    candidate.position.length === 2 &&
    candidate.position.every((item) => typeof item === "number" && Number.isFinite(item)) &&
    typeof candidate.yaw === "number" &&
    Number.isFinite(candidate.yaw) &&
    typeof candidate.distanceM === "number" &&
    Number.isFinite(candidate.distanceM)
  );
}

function readWorldCache(): Record<string, WorldState> {
  try {
    const raw = globalThis.localStorage?.getItem(worldCacheStorageKey);
    if (!raw) return {};
    const parsed: unknown = JSON.parse(raw);
    if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) return {};
    const cache: Record<string, WorldState> = {};
    for (const [id, value] of Object.entries(parsed)) {
      if (isWorldState(value)) cache[id] = value;
    }
    return cache;
  } catch {
    return {};
  }
}

export function persistWorldCache(cache: Record<string, WorldState>) {
  try {
    globalThis.localStorage?.setItem(worldCacheStorageKey, JSON.stringify(cache));
  } catch {
    // Browser storage is optional in the embedded test renderer.
  }
}

export function pruneWorldCache(cache: Record<string, WorldState>, machineIds: string[]) {
  const known = new Set(machineIds);
  let changed = false;
  for (const id of Object.keys(cache)) {
    if (known.has(id)) continue;
    delete cache[id];
    changed = true;
  }
  if (changed) persistWorldCache(cache);
}

function readStoredServo(): ServoModelKey {
  const stored = globalThis.localStorage?.getItem(selectedServoStorageKey);
  return stored === "xl330" || stored === "hd1910" ? stored : "hd1910";
}

export function createDesktopStore() {
  const previewInput: PreviewInput = {
    twist: [0.15, 0, 0],
    head: [0, 0, 0, 0],
    body: [0, 0, 0],
    targetDistanceM: null,
  };
  const state = reactive({
    tab: "home",
    machineView: "simulation",
    catalog: [] as Awaited<ReturnType<typeof commands.servoCatalog>>,
    selectedServo: readStoredServo(),
    virtualMachines: [] as Awaited<ReturnType<typeof commands.virtualMachines>>,
    physicalMachines: [] as Awaited<ReturnType<typeof commands.physicalMachines>>,
    autoReconnectEnabled: true,
    detectedPhysicalMachine: null as Awaited<
      ReturnType<typeof commands.physicalMachineDetect>
    > | null,
    physicalPorts: [] as Awaited<ReturnType<typeof commands.physicalPorts>>,
    physicalBaudChoices: [] as Awaited<ReturnType<typeof commands.physicalBaudChoices>>,
    policies: [] as Awaited<ReturnType<typeof commands.policyList>>,
    selectedPolicyId: null as string | null,
    actions: [] as Awaited<ReturnType<typeof commands.actions>>,
    loop: null as Awaited<ReturnType<typeof commands.loopStatus>>,
    teacherDatasetPath: "",
    debugPose: null as number[] | null,
    debugMappingPending: false,
    rangePreview: null as number[] | null,
    requestedPhysicalMachine: null as string | null,
    previewInput,
    worldCache: readWorldCache(),
    training: null as Awaited<ReturnType<typeof commands.trainingStatus>>,
    trainingFrame: null as TrainingFrame | null,
    selectedVirtualMachine: null as string | null,
    selectedVirtualMachineManual: false,
    error: null as string | null,
    loading: false,
  });

  function applyLoopStatus(status: NonNullable<typeof state.loop>) {
    const current = state.loop;
    if (
      current &&
      (status.runtimeGeneration < current.runtimeGeneration ||
        (status.runtimeGeneration === current.runtimeGeneration &&
          (status.executionGeneration < current.executionGeneration ||
            (status.executionGeneration === current.executionGeneration &&
              (status.configRevision < current.configRevision ||
                (status.configRevision === current.configRevision &&
                  status.tick < current.tick))))))
    )
      return;
    state.loop = status;
    const id = status.activeBody?.virtualMachineId;
    if (id) {
      state.worldCache[id] = status.world;
      persistWorldCache(state.worldCache);
    }
  }

  function applyTrainingProgress(progress: typeof state.training) {
    state.training = progress;
    if (
      progress?.runId !== state.trainingFrame?.runId ||
      (progress?.phase !== "starting" && progress?.phase !== "running")
    ) {
      state.trainingFrame = null;
    }
  }

  function applyTrainingFrame(frame: TrainingFrame) {
    const progress = state.training;
    if (
      progress?.runId === frame.runId &&
      (progress.phase === "starting" || progress.phase === "running") &&
      (!state.trainingFrame || frame.sequence > state.trainingFrame.sequence)
    ) {
      state.trainingFrame = {
        ...frame,
        previewScene: frame.previewScene ?? state.trainingFrame?.previewScene ?? null,
      };
    }
  }

  return { state, applyLoopStatus, applyTrainingProgress, applyTrainingFrame };
}

export const useDesktopStore = defineStore("desktop", () => {
  return createDesktopStore();
});
