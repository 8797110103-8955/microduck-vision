import { reactive } from "vue";
import { defineStore } from "pinia";
import type { DesktopBackend, ServoModelKey } from "./bridge/types";

let configuredBackend: DesktopBackend | null = null;

const selectedServoStorageKey = "microduck.selected-servo";

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
    previewLoop: null as Awaited<ReturnType<DesktopBackend["loopStatus"]>>,
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
