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
  it("keeps virtual-machine bindings when no physical machine exists", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const app = setup(backend);

    await load();
    const updated = await backend.bindVirtualMachine({
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_yaw",
      physicalId: 42,
    });
    app.state.virtualMachines[0] = updated;

    expect(app.state.virtualMachines[0].jointBindings).toEqual([
      expect.objectContaining({ slotId: "left_hip_yaw", physicalId: 42 }),
    ]);
    expect((await backend.virtualMachines())[0].jointBindings).toEqual(
      app.state.virtualMachines[0].jointBindings,
    );
  });

  it("persists each virtual machine's default world position", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);

    const updated = await backend.setVirtualMachineWorldPosition(
      "official-hd1910",
      [1.25, -0.4, 2],
    );

    expect(updated.simulation.worldPosition).toEqual([1.25, -0.4, 2]);
    await expect(backend.virtualMachines()).resolves.toEqual(
      expect.arrayContaining([
        expect.objectContaining({
          id: "official-hd1910",
          simulation: expect.objectContaining({ worldPosition: [1.25, -0.4, 2] }),
        }),
      ]),
    );
  });

  it("changes only the virtual binding when hardware is present", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const physical = await backend.savePhysicalMachine({
      census: [{ servoId: 1, model: "hd1910" }],
      reconnectHint: null,
      name: "实机绑定隔离测试",
    });
    await backend.mountPhysicalMachine(physical.id, "official-hd1910");
    await backend.bindPhysicalMachine({
      physicalMachineId: physical.id,
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_yaw",
      physicalId: 1,
    });

    await backend.bindVirtualMachine({
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_yaw",
      physicalId: 20,
    });

    expect((await backend.virtualMachines())[0].jointBindings).toEqual([
      expect.objectContaining({ slotId: "left_hip_yaw", physicalId: 20 }),
    ]);
    expect(await backend.physicalMachines()).not.toEqual(
      expect.arrayContaining([expect.objectContaining({ jointBindings: expect.anything() })]),
    );
  });

  it("keeps a binding conflict visible until the selected servo is cleared", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const machine = await backend.savePhysicalMachine({
      census: [
        { servoId: 1, model: "hd1910" },
        { servoId: 2, model: "hd1910" },
      ],
      reconnectHint: null,
      name: "冲突测试机",
    });
    await backend.mountPhysicalMachine(machine.id, "official-hd1910");

    await backend.bindPhysicalMachine({
      physicalMachineId: machine.id,
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_yaw",
      physicalId: 1,
    });
    await backend.bindPhysicalMachine({
      physicalMachineId: machine.id,
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_roll",
      physicalId: 1,
    });
    expect((await backend.virtualMachines()).at(0)?.jointBindings).toHaveLength(2);

    const cleared = await backend.validatePhysicalMachineBindings(machine.id);
    expect(cleared).not.toHaveProperty("jointBindings");
    expect((await backend.virtualMachines()).at(0)?.jointBindings).toEqual([]);
  });

  it("keeps virtual bindings when the physical machine is deleted", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const physical = await backend.savePhysicalMachine({
      census: [{ servoId: 42, model: "hd1910" }],
      reconnectHint: null,
      name: "删除测试机",
    });
    await backend.mountPhysicalMachine(physical.id, "official-hd1910");
    await backend.bindVirtualMachine({
      virtualMachineId: "official-hd1910",
      slotId: "left_hip_yaw",
      physicalId: 42,
    });

    await backend.deletePhysicalMachine(physical.id);

    expect((await backend.virtualMachines())[0].jointBindings).toEqual([
      expect.objectContaining({ slotId: "left_hip_yaw", physicalId: 42 }),
    ]);
  });

  it("reads the complete physical register table and writes writable registers", async () => {
    const backend = new MemoryBackend(memoryServoCatalog);
    const machine = await backend.savePhysicalMachine({
      census: [{ servoId: 1, model: "hd1910" }],
      reconnectHint: null,
      name: "寄存器测试机",
    });

    const rows = await backend.physicalMachineRegisters(machine.id);
    expect(rows).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ name: "model_number", writable: false }),
        expect.objectContaining({ name: "torque_enable", writable: true }),
      ]),
    );
    await expect(
      backend.readPhysicalMachineRegister(machine.id, 1, "torque_enable"),
    ).resolves.toMatchObject({ name: "torque_enable", address: 40 });
    const updated = await backend.writePhysicalMachineRegister({
      physicalMachineId: machine.id,
      servoId: 1,
      name: "torque_enable",
      value: 1,
    });
    expect(updated).toMatchObject({ name: "torque_enable", value: 1, writable: true });
  });

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

  it("keeps training runs, metadata, and progress isolated per runtime", async () => {
    const first = new MemoryBackend(memoryServoCatalog);
    const second = new MemoryBackend(memoryServoCatalog);
    const input = {
      program: "uv",
      args: ["run", "train", "task-a"],
      workingDir: "vendor/microduck_rl",
      virtualMachineId: "official-hd1910",
      policyId: "alpha-walking",
      experimentId: null,
      recipe: "student-distillation",
      teacherMotionProvenance: "teacher:official-motion",
    };

    const firstProgress = await first.trainingStart(input);
    const secondProgress = await second.trainingStart({
      ...input,
      args: ["run", "train", "task-b"],
      recipe: "stand-recovery",
    });

    expect(firstProgress.runId).not.toBe(secondProgress.runId);
    expect(firstProgress.metadata.recipe).toBe("student-distillation");
    expect(secondProgress.metadata.recipe).toBe("stand-recovery");
    await first.trainingStop();
    expect((await first.trainingStatus())?.phase).toBe("stopped");
    expect((await second.trainingStatus())?.phase).toBe("starting");
    expect((await second.trainingStatus())?.runId).toBe(secondProgress.runId);
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
