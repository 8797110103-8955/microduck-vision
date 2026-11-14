import { describe, expect, it } from "vitest";
import { createPinia, setActivePinia } from "pinia";
import { MemoryBackend } from "../../src/bridge";
import { configureDesktopBackend, useDesktopStore } from "../../src/store";
import {
  createAction,
  deleteAction,
  load,
  openLoop,
  sendAction,
  sendTargets,
  tick,
} from "../../src/features/debug";
import {
  connectPhysicalMachine,
  createPhysicalMachine,
  deletePhysicalMachine,
  mountPhysicalMachine,
  probePhysicalMachine,
  renamePhysicalMachine,
  savePhysicalMachine,
  scanPhysicalMachine,
} from "../../src/features/machines";
import {
  createVirtualMachine,
  deleteVirtualMachine,
  renameVirtualMachine,
  setStartupVirtualMachine,
} from "../../src/features/simulation";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";

function setup(backend: MemoryBackend) {
  configureDesktopBackend(backend);
  setActivePinia(createPinia());
  return useDesktopStore();
}

describe("desktop to loop data flow", () => {
  it("runs the preview control loop in isolation from the hardware loop", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);

    await load();
    await backend.policyDownload("alpha-walking");
    app.state.previewLoop = await backend.previewLoopOpen("official-hd1910");
    for (let index = 0; index < 5; index += 1)
      app.state.previewLoop = await backend.previewLoopTick();
    expect(app.state.previewLoop?.phase).toBe("ready");

    app.state.previewLoop = await backend.previewLoopPolicy("alpha-walking");
    app.state.previewLoop = await backend.previewLoopTick();
    expect(app.state.previewLoop?.phase).toBe("driving");
    expect(app.state.previewLoop?.torque).toBe(true);
    expect(app.state.loop).toBeNull();
    expect(backend.calls.map((call) => call.name)).not.toContain("loopOpen");
  });

  it("selects a cached official policy and opens a simulation loop without a motor", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);

    await load();
    const policy = await backend.policyDownload("velstand");
    app.state.policies = await backend.policyList();
    app.state.selectedPolicyId = policy.id;
    await openLoop(
      Array.from({ length: 15 }, (_, index) => index + 1),
      undefined,
      policy.id,
    );

    expect(app.state.loop?.phase).toBe("limp");
    expect(backend.calls.at(-1)).toMatchObject({
      name: "loopOpen",
      value: { policyId: "velstand" },
    });
  });

  it("loads servo catalog, selects a virtual machine, drives the loop, and estops", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);

    await load();
    expect(app.state.catalog.map((entry) => entry.key)).toEqual(["hd1910", "xl330"]);
    expect(app.state.selectedVirtualMachine).toBe("official-hd1910");

    const virtualMachine = await createVirtualMachine("XL330 仿真机", "xl330");
    await setStartupVirtualMachine(virtualMachine.id);
    const physicalMachine = await createPhysicalMachine("测试实机", "COM1", "duck-001");
    await connectPhysicalMachine(physicalMachine.id);
    await scanPhysicalMachine(physicalMachine.id);
    await mountPhysicalMachine(physicalMachine.id, virtualMachine.id);
    expect(app.state.physicalMachines[0]).toMatchObject({
      connected: true,
      onlineServoIds: [1, 2],
      virtualMachineId: virtualMachine.id,
    });
    await openLoop([1, 2]);
    await sendAction({ kind: "power", on: true });
    for (let index = 0; index < 5; index += 1) await tick();
    expect(app.state.loop?.phase).toBe("ready");

    await sendAction({ kind: "drive", on: true });
    await sendTargets([0.25, -0.25]);
    await tick();
    expect(app.state.loop?.phase).toBe("driving");
    expect(app.state.loop?.targets).toEqual([0.25, -0.25]);
    expect(app.state.loop?.positions).toEqual([0.25, -0.25]);

    await sendAction({ kind: "estop" });
    expect(app.state.loop?.phase).toBe("estop");
    expect(app.state.loop?.torque).toBe(false);
    expect(backend.calls.map((call) => call.name)).toEqual([
      "servoCatalog",
      "virtualMachines",
      "physicalMachines",
      "actions",
      "autoReconnectEnabled",
      "policyList",
      "createVirtualMachine",
      "setStartupVirtualMachine",
      "createPhysicalMachine",
      "connectPhysicalMachine",
      "scanPhysicalMachine",
      "mountPhysicalMachine",
      "loopOpen",
      "loopAction",
      "loopTick",
      "loopTick",
      "loopTick",
      "loopTick",
      "loopTick",
      "loopAction",
      "loopCommand",
      "loopStatus",
      "loopTick",
      "loopAction",
    ]);
  });

  it("does not allow a command before a virtual machine is selected", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    setup(backend);
    await load();
    await deleteVirtualMachine("official-hd1910");
    await expect(openLoop([1])).rejects.toThrow("请选择模拟机");
  });

  it("clears a physical machine cache without deleting its virtual template", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);
    await load();
    const physicalMachine = await createPhysicalMachine("可删除实机", "COM2", "duck-002");
    await mountPhysicalMachine(physicalMachine.id, "official-hd1910");
    await deletePhysicalMachine(physicalMachine.id);
    expect(app.state.physicalMachines).toHaveLength(0);
    expect(app.state.virtualMachines).toHaveLength(1);
    expect(app.state.virtualMachines[0].id).toBe("official-hd1910");
    expect(app.state.virtualMachines[0].referencedBy).toEqual([]);
  });

  it("renames a simulation and deletes user actions while preserving built-ins", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);
    await load();
    await renameVirtualMachine("official-hd1910", "训练用 HD1910");
    expect(app.state.virtualMachines[0].name).toBe("训练用 HD1910");
    const action = await createAction("挥手");
    expect(app.state.actions.some((entry) => entry.id === action.id)).toBe(true);
    await deleteAction(action.id);
    expect(app.state.actions.some((entry) => entry.id === action.id)).toBe(false);
    await expect(deleteAction("stand")).rejects.toThrow("内置动作");
  });

  it("renames a physical machine without changing its USB or hardware identity", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);
    await load();
    const physicalMachine = await createPhysicalMachine("默认名称", "COM7", "duck-007");

    await renamePhysicalMachine(physicalMachine.id, "客厅实机");

    expect(app.state.physicalMachines[0]).toMatchObject({
      name: "客厅实机",
      usbPort: "COM7",
      machineCode: "duck-007",
    });
    expect(backend.calls.at(-1)?.name).toBe("renamePhysicalMachine");
  });

  it("keeps a complete census as the only save identity", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);
    await load();
    const candidate = await probePhysicalMachine({
      portName: "COM8",
      baud: 1_000_000,
      reconnectHint: {
        portName: "COM8",
        vid: 1,
        pid: 2,
        adapterSerial: "adapter",
        product: "USB",
      },
    });
    expect(candidate.md5).toBe("471fff54c8b7a35c79d2be772be0aa82");
    expect(candidate.matchResult).toBe("new");
    expect(app.state.physicalMachines).toHaveLength(0);
    const saved = await savePhysicalMachine({
      census: candidate.census,
      reconnectHint: candidate.reconnectHint,
      name: null,
    });
    expect(saved.name).toBe(candidate.md5);
    expect(saved.machineCode).toBe(candidate.md5);
    expect(saved.census).toEqual(candidate.census);
    expect(app.state.physicalMachines).toHaveLength(1);
    await expect(
      savePhysicalMachine({
        census: candidate.census,
        reconnectHint: candidate.reconnectHint,
        name: null,
      }),
    ).rejects.toThrow();
  });
});
