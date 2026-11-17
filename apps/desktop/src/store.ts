import { reactive } from "vue";
import { defineStore } from "pinia";
import type { DesktopBackend, JointRangeState, ServoModelKey, WorldState } from "./bridge/types";

let configuredBackend: DesktopBackend | null = null;

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

export function configureDesktopBackend(backend: DesktopBackend) {
  configuredBackend = backend;
}

export function desktopBackend() {
  if (!configuredBackend) throw new Error("desktop backend is not configured");
  return configuredBackend;
}

export function createDesktopStore() {
  const state = reactive({
    tab: "home",
    machineView: "simulation",
    catalog: [] as Awaited<ReturnType<DesktopBackend["servoCatalog"]>>,
    selectedServo: readStoredServo(),
    virtualMachines: [] as Awaited<ReturnType<DesktopBackend["virtualMachines"]>>,
    physicalMachines: [] as Awaited<ReturnType<DesktopBackend["physicalMachines"]>>,
    autoReconnectEnabled: true,
    detectedPhysicalMachine: null as Awaited<
      ReturnType<DesktopBackend["detectPhysicalMachine"]>
    > | null,
    physicalPorts: [] as Awaited<ReturnType<DesktopBackend["physicalPorts"]>>,
    physicalBaudChoices: [] as Awaited<ReturnType<DesktopBackend["physicalBaudChoices"]>>,
    policies: [] as Awaited<ReturnType<DesktopBackend["policyList"]>>,
    selectedPolicyId: null as string | null,
    actions: [] as Awaited<ReturnType<DesktopBackend["actions"]>>,
    loop: null as Awaited<ReturnType<DesktopBackend["loopStatus"]>>,
    jointRangeStates: [] as JointRangeState[],
    debugPose: null as number[] | null,
    rangePreview: null as number[] | null,
    physicalSyncEnabled: false,
    worldCache: readWorldCache(),
    previewLoop: null as Awaited<ReturnType<DesktopBackend["loopStatus"]>>,
    training: null as Awaited<ReturnType<DesktopBackend["trainingStatus"]>>,
    selectedVirtualMachine: null as string | null,
    selectedVirtualMachineManual: false,
    error: null as string | null,
    loading: false,
  });

  return { state };
}

export const useDesktopStore = defineStore("desktop", () => {
  if (!configuredBackend) throw new Error("desktop backend is not configured");
  return createDesktopStore();
});
