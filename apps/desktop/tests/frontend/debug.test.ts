import { describe, expect, it } from "vitest";
import type { PhysicalMachine, VirtualMachine } from "../../src/bridge/types";
import {
  bindingConflictsFor,
  debugBindingFor,
  debugPhysicalMachineFor,
  defaultDebugVirtualMachineId,
} from "../../src/features/debug";

const virtualMachine = (id: string, name: string): VirtualMachine => ({
  id,
  name,
  bodyId: "official",
  servoModel: "hd1910",
  simulation: {
    damping: 0,
    frictionLoss: 0,
    armature: 0,
    kp: 0,
    forceRange: 0,
    worldPosition: [0, 0, 0],
  },
  startupSelected: id === "machine-0",
  referencedBy: [],
});

const physicalMachine = (
  id: string,
  name: string,
  virtualMachineId: string | null,
): PhysicalMachine => ({
  id,
  name,
  usbPort: id,
  machineCode: id,
  census: [{ servoId: 1, model: "hd1910" }],
  reconnectHint: null,
  virtualMachineId,
  onlineServoIds: [1],
  connected: true,
  lastSeen: "刚刚",
});

describe("debug physical pairing", () => {
  it("pairs unbound machines by creation order without changing the binding", () => {
    const virtualMachines = [
      virtualMachine("machine-0", "机器 0"),
      virtualMachine("machine-1", "机器 1"),
    ];
    const physicalMachines = [
      physicalMachine("physical-1", "xxxx", "machine-0"),
      physicalMachine("physical-2", "xxxx2", null),
    ];

    expect(defaultDebugVirtualMachineId(virtualMachines, physicalMachines)).toBe("machine-1");
    expect(debugPhysicalMachineFor("machine-1", virtualMachines, physicalMachines)?.id).toBe(
      "physical-2",
    );
    expect(debugPhysicalMachineFor("machine-0", virtualMachines, physicalMachines)?.id).toBe(
      "physical-1",
    );
    expect(physicalMachines[1].virtualMachineId).toBeNull();
  });

  it("uses the virtual mapping consistently for debug IDs and scene IDs", () => {
    const virtual = virtualMachine("machine-0", "机器 0");
    virtual.jointBindings = [
      { slotId: "left_hip_yaw", physicalId: 42, direction: 1, zeroOffsetRad: 0 },
    ];
    expect(debugBindingFor(virtual, "left_hip_yaw")?.physicalId).toBe(42);
    expect(debugBindingFor(virtual, "right_hip_yaw")).toBeUndefined();
  });

  it("reports duplicate virtual IDs without needing a physical machine", () => {
    const virtual = virtualMachine("machine-0", "机器 0");
    virtual.jointBindings = [
      { slotId: "left_hip_yaw", physicalId: 42, direction: 1, zeroOffsetRad: 0 },
      { slotId: "right_hip_yaw", physicalId: 42, direction: 1, zeroOffsetRad: 0 },
    ];

    expect(bindingConflictsFor(virtual)).toEqual(["42:left_hip_yaw,right_hip_yaw"]);
  });
});
