import { buttonByText, mountApp, setupAppTests, viewerSetFrame } from "./app-harness";
import { flushPromises } from "@vue/test-utils";
import { describe, expect, it, vi } from "vitest";
import { TestBackend } from "../bridge/test-backend";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";
import * as commands from "../../generated/types";
import { disconnectPhysicalMachine } from "../../src/features/machines";
import { useDesktopStore } from "../../src/store";

function encoderAngle(count: number) {
  return (count * Math.PI * 2) / 4096 - Math.PI;
}

describe("desktop frontend flow", () => {
  setupAppTests();

  it("uses the scene default pose before telemetry without projecting zero joint angles", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper } = await mountApp(backend);
    expect(backend.calls.some((call) => call.name === "official_scene_frame")).toBe(false);
    const scene = await commands.officialSceneModel();
    window.dispatchEvent(new Event("microduck:default-pose"));
    await flushPromises();
    expect(viewerSetFrame).toHaveBeenLastCalledWith(scene.defaultFrame);
    expect(backend.calls.some((call) => call.name === "official_scene_frame")).toBe(false);
    wrapper.unmount();
    host.remove();
  });

  it("boots from the frontend, detects and saves hardware, then drives the loop", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);

    expect(store.state.catalog.map((entry) => entry.key)).toEqual(["hd1910", "xl330"]);
    expect(store.state.selectedVirtualMachine).toBe("official-hd1910");
    expect(backend.calls.filter((call) => call.name === "servo_catalog")).toHaveLength(1);
    expect(wrapper.text()).toContain("ONNX");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    expect(wrapper.text()).toContain("策略");
    expect(wrapper.text()).not.toContain("通用");
    const physicalTab = wrapper
      .findAll('button[role="tab"]')
      .find((button) => button.text() === "实机");
    if (!physicalTab) throw new Error("physical tab not found");
    await physicalTab.trigger("click");
    await flushPromises();
    const portSelect = wrapper.find('section[aria-label="舵机连接"] select');
    await portSelect.setValue("COM1");
    if (!(portSelect.element instanceof HTMLSelectElement))
      throw new Error("port select not found");
    expect(portSelect.element.value).toBe("COM1");
    await buttonByText(wrapper, "连接/检测").trigger("click");
    await flushPromises();

    expect(backend.calls.map((call) => call.name)).toContain("physical_machine_probe");
    expect(store.state.detectedPhysicalMachine?.matchResult).toBe("new");
    expect(wrapper.get('section[aria-label="检测候选"]').text()).toContain("ID 1 · ID 2");
    await wrapper.get('section[aria-label="检测候选"]').trigger("click");
    await flushPromises();

    expect(store.state.physicalMachines).toHaveLength(1);
    expect(store.state.physicalMachines[0]).toMatchObject({
      connected: true,
      onlineServoIds: [1, 2],
    });

    await disconnectPhysicalMachine("physical-1");
    expect(store.state.physicalMachines[0].connected).toBe(false);
    await buttonByText(wrapper, "自动重连").trigger("click");
    await flushPromises();
    expect(store.state.physicalMachines[0].connected).toBe(true);
    expect(wrapper.get('[aria-label="实机在线"]')).toBeTruthy();

    const machineRow = wrapper.find("article");
    await machineRow.find("select").setValue("official-hd1910");
    await flushPromises();
    await buttonByText(wrapper, "扫描").trigger("click");
    await flushPromises();

    expect(store.state.physicalMachines[0].virtualMachineId).toBe("official-hd1910");
    expect(store.state.physicalMachines[0].onlineServoIds).toEqual([1, 2]);
    expect(store.state.virtualMachines[0].referencedBy).toContain("physical-1");
    const registerReadsBeforeSync = backend.calls.filter(
      (call) => call.name === "physical_machine_registers",
    ).length;
    await buttonByText(wrapper, "同步实机参数").trigger("click");
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "physical_machine_registers")).toHaveLength(
      registerReadsBeforeSync + 1,
    );
    expect(wrapper.text()).not.toContain("硬件码：");
    await backend.invoke("virtual_machine_bind", {
      input: {
        virtualMachineId: "official-hd1910",
        slotId: "left_hip_yaw",
        physicalId: 1,
      },
    });
    store.state.virtualMachines[0].jointBindings = [{ slotId: "left_hip_yaw", physicalId: 1 }];
    store.state.physicalMachines = await commands.physicalMachines();

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "调试").trigger("click");
    expect(wrapper.text()).toContain("机器 0");
    expect(wrapper.text()).not.toContain("参数沿用旧版舵机调试面板");
    expect(wrapper.text()).not.toContain("舵机参数");
    await wrapper.get('button[title="将视图聚焦到当前选中的模拟机"]').trigger("click");
    await wrapper.get('button[title="将视图恢复到聚焦当前模拟机之前的位置"]').trigger("click");
    expect(wrapper.text()).not.toContain("安全状态");
    expect(wrapper.text()).not.toContain("目标位置");
    await wrapper.get('button[title="切换到肢体范围"]').trigger("click");
    expect(wrapper.text()).not.toContain("场景姿态预览");
    const rangeRow = wrapper.findAll('[class*="rangeRow"]')[0];
    await rangeRow.findAll("button")[0].trigger("click");
    expect(store.state.rangePreview?.[0]).toBeLessThan(0);
    await wrapper.get('button[title="切换到肢体绑定"]').trigger("click");
    expect(wrapper.get('button[title="切换到肢体范围"]')).toBeTruthy();
    const bindingSelects = wrapper.findAll("select");
    expect((bindingSelects[0].element as HTMLSelectElement).value).toBe("1");
    const physicalBindCount = backend.calls.filter(
      (call) => call.name === "physical_machine_bind",
    ).length;
    const virtualBindCount = backend.calls.filter(
      (call) => call.name === "virtual_machine_bindings",
    ).length;
    const bindingStart = backend.calls.length;
    const bindingGeneration = store.state.loop?.readSources[0]?.generation;
    await bindingSelects[10].setValue("1");
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "physical_machine_bind")).toHaveLength(
      physicalBindCount,
    );
    expect(wrapper.text()).toContain("绑定冲突");
    const bindingRows = wrapper.findAll('[class*="bindingRow"]');
    await bindingRows[10].trigger("click");
    await bindingRows[10].trigger("click");
    await flushPromises();
    const pendingConflictButton = wrapper
      .findAll("button")
      .find((button) => button.text().includes("right_hip_yaw"));
    if (!pendingConflictButton) throw new Error("pending conflict button not found");
    await pendingConflictButton.trigger("click");
    expect(wrapper.findAll('[class*="activeRow"]')).toHaveLength(1);
    await bindingSelects[0].setValue("2");
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "physical_machine_bind")).toHaveLength(
      physicalBindCount,
    );
    expect(backend.calls.filter((call) => call.name === "virtual_machine_bindings")).toHaveLength(
      virtualBindCount + 1,
    );
    expect(
      backend.calls
        .slice(bindingStart)
        .some((call) =>
          ["runtime_open", "runtime_close", "physical_machines", "virtual_machines"].includes(
            call.name,
          ),
        ),
    ).toBe(false);
    expect(store.state.loop?.readSources[0]?.generation).toBe(bindingGeneration);
    expect(store.state.virtualMachines[0].jointBindings).toEqual(
      expect.arrayContaining([expect.objectContaining({ slotId: "left_hip_yaw", physicalId: 2 })]),
    );
    const directionButton = wrapper.findAll("button").find((button) => button.text() === "1");
    if (!directionButton) throw new Error("direction button not found");
    const loopOpensBeforeDirection = backend.calls.filter(
      (call) => call.name === "runtime_open",
    ).length;
    await directionButton.trigger("click");
    await flushPromises();
    expect(backend.calls.map((call) => call.name)).toContain("physical_machine_bind");
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(
      loopOpensBeforeDirection,
    );
    expect(
      store.state.physicalMachines[0].jointDirections?.some((binding) => binding.direction === -1),
    ).toBe(true);
    const loopOpensBeforeZero = backend.calls.filter((call) => call.name === "runtime_open").length;
    await wrapper.get('button[title="将当前实机姿态保存为这台真机的零位"]').trigger("click");
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(
      loopOpensBeforeZero,
    );
    expect(store.state.physicalMachines[0].jointBindings?.[0]?.zeroOffsetRad).toBe(0);
    expect(store.state.loop?.positions[0]).toBeCloseTo(0);
    expect(store.state.loop?.hasExecution).toBe(false);
    expect(wrapper.text()).toContain("只读");
    expect(wrapper.text()).not.toContain("切换为手动控制");
    expect(wrapper.text()).not.toContain("返回只读");
    expect(wrapper.text()).not.toContain("全部力矩开");
    expect(wrapper.text()).not.toContain("全部力矩关");
    expect(backend.calls.map((call) => call.name)).not.toContain("runtime_reconfigure");
    expect(backend.calls.map((call) => call.name)).not.toContain("physical_machine_set_torque");
    expect(backend.calls.map((call) => call.name)).not.toContain("physical_machine_write_position");
    const debugOpens = backend.calls.filter((call) => call.name === "runtime_open");
    expect(debugOpens.length).toBeGreaterThan(0);
    for (const call of debugOpens)
      expect(call.payload).toMatchObject({ input: { access: "read_only", policyId: null } });
    expect(backend.calls.map((call) => call.name)).not.toContain("loop_action");
    expect(backend.calls.map((call) => call.name)).not.toContain("loop_command");
    expect(wrapper.text()).not.toContain("发送微调目标");
    expect(wrapper.text()).not.toContain("推进一拍");
    expect(wrapper.text()).not.toContain("启动控制环");
    expect(wrapper.text()).not.toContain("刷新状态");
    expect(store.state.loop?.hasExecution).toBe(false);

    wrapper.unmount();
    host.remove();
  });

  it("shows and persists saved servo IDs while the virtual machine is offline", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("virtual_machine_bind", {
      input: { virtualMachineId: "official-hd1910", slotId: "left_hip_yaw", physicalId: 9 },
    });
    const { host, wrapper, store } = await mountApp(backend);
    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "调试").trigger("click");
    await flushPromises();
    const switchToBindings = wrapper.find('button[title="切换到肢体绑定"]');
    if (switchToBindings.exists()) await switchToBindings.trigger("click");
    const binding = wrapper.findAll("select")[0];
    expect((binding.element as HTMLSelectElement).value).toBe("9");
    expect(binding.find('option[value="9"]').text()).toContain("已保存");
    await binding.setValue("20");
    await flushPromises();
    expect(store.state.virtualMachines[0].jointBindings?.[0]?.physicalId).toBe(20);
    expect(backend.calls.some((call) => call.name === "physical_machine_bind")).toBe(false);
    wrapper.unmount();
    host.remove();
  });

  it("loads saved hardware direction overrides independently of virtual bindings", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("physical_machine_save", {
      input: {
        name: "test",
        census: [
          { servoId: 1, model: "hd1910" },
          { servoId: 2, model: "hd1910" },
        ],
        reconnectHint: null,
      },
    });
    await backend.invoke("physical_machine_mount", {
      id: "physical-1",
      virtualMachineId: "official-hd1910",
    });
    await backend.invoke("physical_machine_bind", {
      input: {
        physicalMachineId: "physical-1",
        slotId: "left_hip_yaw",
        direction: -1,
      },
    });
    const { host, wrapper, store } = await mountApp(backend);
    store.state.tab = "debug";
    await flushPromises();
    const switchToBindings = wrapper.find('button[title="切换到肢体绑定"]');
    if (switchToBindings.exists()) await switchToBindings.trigger("click");
    const row = wrapper.findAll('[class*="bindingRow"]')[0];
    expect(row.text()).toContain("-1");
    expect(store.state.virtualMachines[0].jointBindings ?? []).toEqual([]);
    expect(store.state.physicalMachines[0].jointBindings ?? []).toEqual([]);
    wrapper.unmount();
    await flushPromises();
    host.remove();
  });

  it("starts a read-only debug loop after hardware connects and releases it on exit", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("physical_machine_save", {
      input: {
        name: "test",
        census: [
          { servoId: 1, model: "hd1910" },
          { servoId: 2, model: "hd1910" },
        ],
        reconnectHint: null,
      },
    });
    await backend.invoke("physical_machine_mount", {
      id: "physical-1",
      virtualMachineId: "official-hd1910",
    });
    await backend.invoke("physical_machine_disconnect", { id: "physical-1" });
    const { host, wrapper, store } = await mountApp(backend);
    store.state.tab = "debug";
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(0);
    await commands.physicalMachineConnect("physical-1");
    store.state.physicalMachines = await commands.physicalMachines();
    await flushPromises();
    expect(backend.calls.find((call) => call.name === "runtime_open")?.payload).toEqual({
      input: {
        virtualMachineId: "official-hd1910",
        physicalMachineId: "physical-1",
        access: "read_only",
        policyId: null,
      },
    });
    expect(store.state.loop?.hasExecution).toBe(false);
    expect(store.state.loop?.readSources[0]?.id).toBe("physical-1");
    await commands.physicalMachineDisconnect("physical-1");
    store.state.physicalMachines = await commands.physicalMachines();
    await flushPromises();
    expect(store.state.loop).toBeNull();
    await commands.physicalMachineConnect("physical-1");
    store.state.physicalMachines = await commands.physicalMachines();
    await flushPromises();
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(2);
    store.state.tab = "preview";
    await flushPromises();
    expect(store.state.loop).toMatchObject({ hasExecution: true, readSources: [], phase: "ready" });
    expect(
      backend.calls.some(
        (call) => call.name === "runtime_reconfigure" || call.name === "runtime_command",
      ),
    ).toBe(false);
    wrapper.unmount();
    host.remove();
  });

  it("switches the debug read source with the selected virtual machine", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    const second = await commands.virtualMachineCreate({
      name: "second",
      bodyId: "official",
      executionKind: "simulation",
      modelKind: "hd1910-compatible",
      servoModel: "hd1910",
      simulation: store.state.virtualMachines[0].simulation,
    });
    store.state.virtualMachines = await commands.virtualMachines();
    for (const [index, virtualId] of ["official-hd1910", second.id].entries()) {
      await backend.invoke("physical_machine_save", {
        input: {
          name: `physical ${index}`,
          census: [
            { servoId: 1, model: "hd1910" },
            { servoId: 2, model: "hd1910" },
          ],
          reconnectHint: null,
        },
      });
      await commands.physicalMachineMount(`physical-${index + 1}`, virtualId);
    }
    store.state.physicalMachines = await commands.physicalMachines();
    store.state.tab = "debug";
    await flushPromises();
    expect(store.state.loop?.readSources[0]?.id).toBe("physical-1");
    store.state.selectedVirtualMachine = second.id;
    await flushPromises();
    expect(store.state.loop?.readSources.map((source) => source.id)).toEqual(["physical-2"]);
    const opens = backend.calls.filter((call) => call.name === "runtime_open");
    expect(opens.at(-1)?.payload).toMatchObject({
      input: { virtualMachineId: second.id, physicalMachineId: "physical-2", access: "read_only" },
    });
    wrapper.unmount();
    await flushPromises();
    host.remove();
  });

  it("attaches debug reads without replacing a simulation execution and preserves it on exit", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("physical_machine_save", {
      input: {
        name: "test",
        census: [
          { servoId: 1, model: "hd1910" },
          { servoId: 2, model: "hd1910" },
        ],
        reconnectHint: null,
      },
    });
    await backend.invoke("physical_machine_mount", {
      id: "physical-1",
      virtualMachineId: "official-hd1910",
    });
    const { host, wrapper, store } = await mountApp(backend);
    const running = await commands.runtimeOpen({
      virtualMachineId: "official-hd1910",
      physicalMachineId: null,
      access: "read_only",
      policyId: null,
    });
    store.applyLoopStatus(running);
    store.state.tab = "debug";
    await flushPromises();
    expect(store.state.loop?.activeBody).toEqual(running.activeBody);
    expect(store.state.loop?.runtimeGeneration).toBe(running.runtimeGeneration);
    expect(store.state.loop?.readSources[0]?.id).toBe("physical-1");
    store.state.tab = "preview";
    await flushPromises();
    expect(store.state.loop?.activeBody).toEqual(running.activeBody);
    expect(store.state.loop?.hasExecution).toBe(true);
    expect(store.state.loop?.readSources).toEqual([]);
    expect(backend.calls.some((call) => call.name === "runtime_reconfigure")).toBe(false);
    wrapper.unmount();
    host.remove();
  });

  it.each(["manual", "policy"] as const)(
    "only observes an existing %s physical execution when entering and leaving debug",
    async (source) => {
      const backend = new TestBackend(memoryServoCatalog);
      await backend.invoke("physical_machine_save", {
        input: {
          name: "test",
          census: [
            { servoId: 1, model: "hd1910" },
            { servoId: 2, model: "hd1910" },
          ],
          reconnectHint: null,
        },
      });
      await backend.invoke("physical_machine_mount", {
        id: "physical-1",
        virtualMachineId: "official-hd1910",
      });
      const { host, wrapper, store } = await mountApp(backend);
      let running = await commands.runtimeOpen({
        virtualMachineId: "official-hd1910",
        physicalMachineId: source === "manual" ? "physical-1" : null,
        access: source === "manual" ? "control" : "read_only",
        policyId: null,
      });
      if (source === "policy")
        running = await commands.runtimeSelectBody(
          running.runtimeGeneration,
          running.executionGeneration,
          "official-hd1910",
          "physical-1",
        );
      store.applyLoopStatus(running);
      const firstDebugCall = backend.calls.length;
      store.state.tab = "debug";
      await flushPromises();
      expect(store.state.loop).toEqual(running);
      expect(wrapper.text()).toContain("只读");
      expect(wrapper.text()).not.toMatch(/切换为手动控制|返回只读|全部力矩[开关]/);
      expect(
        wrapper.get('button[aria-label="left_knee 方向"]').attributes("disabled"),
      ).toBeDefined();
      expect(wrapper.findAll("select")[0].attributes("disabled")).toBeDefined();
      expect(
        wrapper.get('button[title="将当前实机姿态保存为这台真机的零位"]').attributes("disabled"),
      ).toBeDefined();
      await wrapper.get('button[title="切换到肢体范围"]').trigger("click");
      await wrapper.findAll('[class*="rangeRow"]')[0].findAll("button")[0].trigger("click");
      expect(store.state.rangePreview).not.toBeNull();
      store.state.tab = "home";
      await flushPromises();
      expect(store.state.loop).toEqual(running);
      const debugCalls = backend.calls.slice(firstDebugCall);
      for (const call of debugCalls.filter((entry) => entry.name === "runtime_open"))
        expect(call.payload).toMatchObject({ input: { access: "read_only", policyId: null } });
      expect(debugCalls.map((call) => call.name)).not.toContain("runtime_reconfigure");
      expect(debugCalls.map((call) => call.name)).not.toContain("runtime_command");
      expect(debugCalls.map((call) => call.name)).not.toContain("runtime_close");
      expect(debugCalls.map((call) => call.name)).not.toContain("physical_machine_set_torque");
      expect(debugCalls.map((call) => call.name)).not.toContain("physical_machine_write_position");
      wrapper.unmount();
      host.remove();
    },
  );

  it("reverses the calibrated knee sent to 3D and drops an old in-flight pose after a direction edit", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("physical_machine_save", {
      input: {
        name: "test",
        census: [
          { servoId: 1, model: "hd1910" },
          { servoId: 2, model: "hd1910" },
        ],
        reconnectHint: null,
      },
    });
    await backend.invoke("physical_machine_mount", {
      id: "physical-1",
      virtualMachineId: "official-hd1910",
    });
    await backend.invoke("virtual_machine_bind", {
      input: {
        virtualMachineId: "official-hd1910",
        slotId: "left_knee",
        physicalId: 1,
      },
    });
    await backend.invoke("physical_machine_bind", {
      input: {
        physicalMachineId: "physical-1",
        slotId: "left_knee",
        direction: 1,
      },
    });
    backend.servoPositions.set("physical-1", new Map([[1, 1.14]]));
    const { host, wrapper, store } = await mountApp(backend);
    store.state.tab = "debug";
    await flushPromises();
    expect(store.state.loop?.readSources[0].positions[3]).toBeCloseTo(1.14);
    const expected = Array(15).fill(0);
    expected[3] = 1.14;
    expect(
      backend.calls.findLast((call) => call.name === "official_scene_frame")?.payload,
    ).toMatchObject({ positions: expected });
    const oldGeneration = store.state.loop!.readSources[0].generation;
    const oldSequence = store.state.loop!.readSources[0].sequence;
    const mutationStart = backend.calls.length;

    const invoke = backend.invoke.bind(backend);
    let finishOldFrame: (() => void) | undefined;
    const waiting = new Promise<void>((resolve) => {
      finishOldFrame = resolve;
    });
    let finishSave: (() => void) | undefined;
    const saving = new Promise<void>((resolve) => {
      finishSave = resolve;
    });
    let interceptFrame = true;
    const intercepted = vi.spyOn(backend, "invoke").mockImplementation(async (command, payload) => {
      if (command === "official_scene_frame" && interceptFrame) {
        interceptFrame = false;
        await waiting;
      }
      if (command === "physical_machine_bind") await saving;
      return invoke(command, payload);
    });
    await wrapper.get('button[title="切换到肢体范围"]').trigger("click");
    await flushPromises();
    expect(store.state.rangePreview).not.toBeNull();
    // Force a fresh visible request to remain pending during the edit.
    store.state.rangePreview = [...store.state.rangePreview!];
    await flushPromises();
    await wrapper.get('button[title="切换到肢体绑定"]').trigger("click");
    store.state.rangePreview = Array(15).fill(0.4);
    await flushPromises();
    viewerSetFrame.mockClear();
    await wrapper.get('button[aria-label="left_knee 方向"]').trigger("click");
    await flushPromises();
    expect(store.state.rangePreview).toBeNull();
    expect(store.state.loop?.readSources[0].generation).toBe(oldGeneration);
    finishOldFrame?.();
    await flushPromises();
    expect(viewerSetFrame).not.toHaveBeenCalled();
    const savedFrameStart = backend.calls.length;
    finishSave?.();
    await flushPromises();
    store.applyLoopStatus((await commands.loopStatus())!);
    await flushPromises();
    expect(store.state.loop?.readSources[0].generation).toBe(oldGeneration);
    expect(store.state.loop?.readSources[0].sequence).toBeGreaterThan(oldSequence);
    expect(store.state.loop?.readSources[0].positions[3]).toBeCloseTo(-1.14);
    expect(
      store.state.physicalMachines[0].jointDirections?.find((entry) => entry.slotId === "left_knee")
        ?.direction,
    ).toBe(-1);
    expect(viewerSetFrame).toHaveBeenCalled();
    const latest = backend.calls.findLast((call) => call.name === "official_scene_frame");
    expected[3] = -1.14;
    expect(latest?.payload).toMatchObject({ positions: expected });
    const appliedRequests = backend.calls
      .slice(savedFrameStart)
      .filter((call) => call.name === "official_scene_frame")
      .slice(-1);
    expect(appliedRequests.length).toBeGreaterThan(0);
    for (const request of appliedRequests)
      expect(request.payload).toMatchObject({ positions: expected });
    const diagnostic = backend.calls.findLast((call) => call.name === "frontend_diagnostic");
    expect(diagnostic?.payload).toMatchObject({
      message: expect.stringContaining(`"sourceGeneration":${oldGeneration}`),
    });
    expect(
      backend.calls.some(
        (call) => call.name === "physical_machine_set_torque" || call.name === "runtime_command",
      ),
    ).toBe(false);
    expect(
      backend.calls
        .slice(mutationStart)
        .some((call) =>
          ["runtime_open", "runtime_close", "physical_machines", "virtual_machines"].includes(
            call.name,
          ),
        ),
    ).toBe(false);
    intercepted.mockRestore();
    wrapper.unmount();
    await flushPromises();
    host.remove();
  });

  it.each([false, true])(
    "centers hardware encoders in debug and applies saved calibration without reopening the source (partial=%s)",
    async (partial) => {
      const backend = new TestBackend(memoryServoCatalog);
      await backend.invoke("physical_machine_save", {
        input: {
          name: "test",
          census: [
            { servoId: 1, model: "hd1910" },
            { servoId: 2, model: "hd1910" },
          ],
          reconnectHint: null,
        },
      });
      await backend.invoke("physical_machine_mount", {
        id: "physical-1",
        virtualMachineId: "official-hd1910",
      });
      await backend.invoke("virtual_machine_bind", {
        input: { virtualMachineId: "official-hd1910", slotId: "left_hip_roll", physicalId: 1 },
      });
      await backend.invoke("physical_machine_bind", {
        input: { physicalMachineId: "physical-1", slotId: "left_hip_roll", direction: -1 },
      });
      backend.servoPositions.set("physical-1", new Map([[1, encoderAngle(15)]]));
      await backend.invoke("physical_machine_set_pose_zero", {
        id: "physical-1",
        virtualMachineId: "official-hd1910",
      });
      backend.servoPositions.set("physical-1", new Map([[1, encoderAngle(4130)]]));
      const { host, wrapper, store } = await mountApp(backend);
      store.state.tab = "debug";
      await flushPromises();
      const generation = store.state.loop!.readSources[0].generation;
      const start = backend.calls.length;
      const invoke = backend.invoke.bind(backend);
      const intercepted = vi
        .spyOn(backend, "invoke")
        .mockImplementation(async (command, payload) => {
          const result = await invoke(command, payload);
          if (partial && command === "physical_machine_center_encoders") {
            if (typeof result !== "object" || result === null)
              throw new Error("missing calibration result");
            return {
              ...result,
              error: "后续电机仍在转动",
            };
          }
          return result;
        });
      await buttonByText(wrapper, "校准编码器中位").trigger("click");
      await flushPromises();
      expect(store.state.physicalMachines[0].jointBindings?.[0].zeroOffsetRad).toBeCloseTo(
        encoderAngle(2029),
      );
      expect(store.state.physicalMachines[0].jointDirections?.[0].direction).toBe(-1);
      expect(store.state.physicalMachines[0].torqueEnabled).toBe(false);
      expect(store.state.error).toBe(partial ? "后续电机仍在转动" : null);
      store.applyLoopStatus((await commands.loopStatus())!);
      await flushPromises();
      expect(store.state.loop?.readSources[0].generation).toBe(generation);
      expect(
        backend.calls
          .slice(start)
          .some((call) =>
            [
              "runtime_open",
              "runtime_close",
              "runtime_command",
              "physical_machine_set_torque",
              "physical_machines",
              "virtual_machines",
            ].includes(call.name),
          ),
      ).toBe(false);
      expect(
        backend.calls.slice(start).find((call) => call.name === "physical_machine_center_encoders")
          ?.payload,
      ).toMatchObject({ id: "physical-1", virtualMachineId: "official-hd1910" });
      store.state.loop!.activeBody = {
        virtualMachineId: "official-hd1910",
        physicalMachineId: "physical-1",
      };
      await flushPromises();
      expect((buttonByText(wrapper, "校准编码器中位").element as HTMLButtonElement).disabled).toBe(
        true,
      );
      intercepted.mockRestore();
      wrapper.unmount();
      await flushPromises();
      host.remove();
    },
  );

  it("releases a debug source that finishes opening after the page exits", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    await backend.invoke("physical_machine_save", {
      input: {
        name: "test",
        census: [
          { servoId: 1, model: "hd1910" },
          { servoId: 2, model: "hd1910" },
        ],
        reconnectHint: null,
      },
    });
    await backend.invoke("physical_machine_mount", {
      id: "physical-1",
      virtualMachineId: "official-hd1910",
    });
    const { host, wrapper } = await mountApp(backend);
    const invoke = backend.invoke.bind(backend);
    let finishOpen: (() => void) | undefined;
    const opening = new Promise<void>((resolve) => {
      finishOpen = resolve;
    });
    const intercepted = vi.spyOn(backend, "invoke").mockImplementation(async (command, payload) => {
      if (command === "runtime_open") await opening;
      return invoke(command, payload);
    });
    const store = useDesktopStore();
    store.state.tab = "debug";
    await flushPromises();
    expect(intercepted.mock.calls.some(([command]) => command === "runtime_open")).toBe(true);
    wrapper.unmount();
    finishOpen?.();
    await flushPromises();
    expect(await commands.loopStatus()).toBeNull();
    expect(backend.calls.find((call) => call.name === "runtime_close")?.payload).toMatchObject({
      scope: "readOnlySource",
    });
    intercepted.mockRestore();
    host.remove();
  });

  it("opens the preview overlay and selects a cached policy for the current machine", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);

    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    expect(store.state.tab).toBe("preview");
    expect(wrapper.text()).toContain("当前电脑还没有已缓存的 ONNX");
    await buttonByText(wrapper, "开启控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({ phase: "ready", torque: false });
    expect(backend.calls.map((call) => call.name)).toContain("runtime_open");
    expect(buttonByText(wrapper, "开始采集").attributes("disabled")).toBeDefined();
    const resetWorldButton = buttonByText(wrapper, "归零");
    expect((resetWorldButton.element as HTMLButtonElement).disabled).toBe(true);
    store.state.loop = {
      ...store.state.loop!,
      world: { position: [1, 0], yaw: 0, distanceM: 1 },
    };
    await wrapper.vm.$nextTick();
    expect((resetWorldButton.element as HTMLButtonElement).disabled).toBe(false);
    await resetWorldButton.trigger("click");
    await flushPromises();
    expect(backend.calls.map((call) => call.name)).toContain("runtime_command");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "策略").trigger("click");
    await buttonByText(wrapper, "下载").trigger("click");
    await flushPromises();
    expect(wrapper.text()).toContain("已缓存");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    expect(store.state.tab).toBe("preview");
    expect(wrapper.text()).toContain("Alpha Walking");
    const callsBeforePolicy = backend.calls.length;
    await buttonByText(wrapper, "Alpha Walking").trigger("click");
    await flushPromises();
    expect(store.state.selectedPolicyId).toBe("alpha-walking");
    expect(store.state.previewInput).toMatchObject({ twist: [0.15, 0, 0], targetDistanceM: null });
    expect(backend.calls.map((call) => call.name)).toContain("policy_select");
    expect(
      backend.calls.slice(callsBeforePolicy).some((call) => call.name === "runtime_command"),
    ).toBe(false);
    await buttonByText(wrapper, "开启控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop?.torque).toBe(true);
    const policyCommands = backend.calls
      .slice(callsBeforePolicy)
      .filter((call) => call.name === "runtime_command");
    expect(policyCommands).toHaveLength(1);
    expect(policyCommands[0].payload).toMatchObject({
      request: {
        kind: "policy",
        policy_id: "alpha-walking",
        input: { twist: [0.15, 0, 0], targetDistanceM: null },
      },
    });
    expect(backend.calls.map((call) => call.name)).toContain("runtime_command");
    const speed = wrapper.find("[data-slider] input");
    await speed.setValue("0.2");
    await speed.trigger("blur");
    await flushPromises();
    expect(backend.calls.map((call) => call.name)).toContain("runtime_command");
    expect(store.state.loop?.world.distanceM).toBeGreaterThanOrEqual(0);
    await backend.invoke("policy_download", { id: "alpha-stand" });
    await backend.invoke("policy_select", { id: "alpha-stand" });
    const switched = await backend.invoke("preview_loop_policy", { policyId: "alpha-stand" });
    expect(switched).toMatchObject({ torque: true });
    expect(backend.calls.map((call) => call.name)).not.toContain("runtime_close");
    await buttonByText(wrapper, "停止控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({ hasExecution: true, phase: "holding" });

    wrapper.unmount();
    host.remove();
  });

  it("samples VLA automatically and separates physical sync, torque and ONNX writes", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const probe = await backend.invoke("physical_machine_probe", { input: {} });
    await backend.invoke("physical_machine_save", { input: probe });
    await backend.invoke("policy_download", { id: "alpha-walking" });
    backend.servoPositions.set(
      "physical-1",
      new Map([
        [1, 0.4],
        [2, -0.2],
      ]),
    );
    const { host, wrapper, store } = await mountApp(backend);
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    const generation = store.state.loop!.runtimeGeneration;
    const virtualId = store.state.selectedVirtualMachine!;
    const virtual = store.state.virtualMachines.find((machine) => machine.id === virtualId)!;
    const bound = await commands.virtualMachineBindings(virtualId, [
      ...(virtual.jointBindings ?? []).map((binding) => ({
        slotId: binding.slotId,
        physicalId: null,
      })),
      { slotId: "left_hip_yaw", physicalId: 1 },
      { slotId: "left_hip_roll", physicalId: 2 },
    ]);
    store.state.virtualMachines = store.state.virtualMachines.map((machine) =>
      machine.id === virtualId ? bound : machine,
    );
    expect(store.state.loop).toMatchObject({ phase: "ready", hasExecution: true });
    await buttonByText(wrapper, "Alpha Walking").trigger("click");
    await flushPromises();
    await buttonByText(wrapper, "真机同步").trigger("click");
    await flushPromises();
    expect(store.state.loop?.activeBody?.physicalMachineId).toBeNull();
    expect(store.state.loop?.readSources[0]?.positions[0]).toBe(0.4);
    expect(wrapper.text()).toContain("只读同步真机体态");
    expect((buttonByText(wrapper, "开启真机控制环").element as HTMLButtonElement).disabled).toBe(
      true,
    );
    expect(backend.calls.some((call) => call.name === "runtime_select_body")).toBe(false);
    expect(
      backend.calls.some(
        (call) =>
          call.name === "runtime_command" &&
          JSON.stringify(call.payload).includes('"kind":"policy"'),
      ),
    ).toBe(false);
    backend.servoPositions.get("physical-1")!.set(1, 0.6);
    store.applyLoopStatus((await commands.loopStatus())!);
    await flushPromises();
    expect(
      backend.calls.findLast((call) => call.name === "official_scene_frame")?.payload,
    ).toMatchObject({ positions: [0.6, -0.2, ...Array(13).fill(0)] });
    await buttonByText(wrapper, "开启力矩").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({
      torque: true,
      phase: "holding",
      activeBody: { physicalMachineId: "physical-1" },
    });
    expect(store.state.loop?.targets[0]).toBe(0.6);
    await buttonByText(wrapper, "开启真机控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop?.phase).toBe("driving");
    await buttonByText(wrapper, "停止控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({
      torque: true,
      phase: "holding",
      runtimeGeneration: generation,
    });
    await buttonByText(wrapper, "关闭力矩").trigger("click");
    await flushPromises();
    expect(store.state.loop?.activeBody?.physicalMachineId).toBeNull();
    expect(store.state.loop?.readSources[0]?.id).toBe("physical-1");
    expect(wrapper.text()).toContain("只读同步真机体态");
    expect((buttonByText(wrapper, "开启真机控制环").element as HTMLButtonElement).disabled).toBe(
      true,
    );
    await buttonByText(wrapper, "真机同步").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({
      runtimeGeneration: generation,
      readSources: [],
      hasExecution: true,
    });
    wrapper.unmount();
    host.remove();
  });
  it("keeps sampling read-only when enabling physical torque fails", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const probe = await backend.invoke("physical_machine_probe", { input: {} });
    await backend.invoke("physical_machine_save", { input: probe });
    const { host, wrapper, store } = await mountApp(backend);
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    await buttonByText(wrapper, "真机同步").trigger("click");
    await flushPromises();
    const generation = store.state.loop!.runtimeGeneration;
    const invoke = backend.invoke.bind(backend);
    const intercepted = vi.spyOn(backend, "invoke").mockImplementation((name, payload) => {
      if (name === "runtime_select_body") throw new Error("开启力矩失败");
      return invoke(name, payload);
    });
    await buttonByText(wrapper, "开启力矩").trigger("click");
    await flushPromises();
    expect(wrapper.text()).toContain("开启力矩失败");
    expect(wrapper.text()).toContain("只读同步真机体态");
    expect(store.state.loop).toMatchObject({
      runtimeGeneration: generation,
      activeBody: { physicalMachineId: null },
    });
    expect(store.state.loop?.readSources[0]?.id).toBe("physical-1");
    intercepted.mockRestore();
    wrapper.unmount();
    host.remove();
  });

  it("restores simulation selection when the physical read source is offline", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const probe = await backend.invoke("physical_machine_probe", { input: {} });
    await backend.invoke("physical_machine_save", { input: probe });
    const { host, wrapper, store } = await mountApp(backend);
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    const generation = store.state.loop!.runtimeGeneration;
    await backend.invoke("physical_machine_disconnect", { id: "physical-1" });
    await buttonByText(wrapper, "真机同步").trigger("click");
    await flushPromises();
    expect(wrapper.text()).toContain("真机离线");
    expect(store.state.requestedPhysicalMachine).toBeNull();
    expect(store.state.loop).toMatchObject({
      runtimeGeneration: generation,
      hasExecution: true,
      activeBody: { physicalMachineId: null },
      readSources: [],
    });
    wrapper.unmount();
    host.remove();
  });

  it("resets a faulted runtime into sampling without automatically starting ONNX", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    store.state.loop = {
      ...store.state.loop!,
      runtimeState: "faulted",
      lastError: "goal_position cannot encode",
    };
    await flushPromises();
    await buttonByText(wrapper, "重置控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop).toMatchObject({ runtimeState: "running", phase: "ready" });
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(2);
    expect(backend.calls.some((call) => call.name === "runtime_command")).toBe(false);
    wrapper.unmount();
    host.remove();
  });

  it("does not reopen a faulted runtime when torque-off cannot be confirmed", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    store.state.loop = {
      ...store.state.loop!,
      runtimeState: "faulted",
      lastError: "torque-off access denied",
    };
    const faulted = store.state.loop;
    const invoke = backend.invoke.bind(backend);
    const intercepted = vi.spyOn(backend, "invoke").mockImplementation((name, payload) => {
      if (name === "runtime_close") throw new Error("torque-off access denied");
      if (name === "loop_status") return Promise.resolve(faulted);
      return invoke(name, payload);
    });
    await flushPromises();
    await buttonByText(wrapper, "重置控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop?.runtimeState).toBe("faulted");
    expect(backend.calls.filter((call) => call.name === "runtime_open")).toHaveLength(1);
    expect(wrapper.text()).toContain("torque-off access denied");
    intercepted.mockRestore();
    wrapper.unmount();
    host.remove();
  });

  it("opens the log folder from settings and displays shell errors", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper } = await mountApp(backend);
    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await wrapper.get('button[aria-label="日志"]').trigger("click");
    await flushPromises();

    const button = buttonByText(wrapper, "打开日志文件夹");
    await button.trigger("click");
    await flushPromises();
    expect(backend.calls.map((call) => call.name)).toContain("open_logs");

    const invoke = backend.invoke.bind(backend);
    vi.spyOn(backend, "invoke").mockImplementation((name, payload) => {
      if (name === "open_logs") return Promise.reject("打开日志文件夹失败：访问被拒绝");
      return invoke(name, payload);
    });
    await button.trigger("click");
    await flushPromises();
    expect(wrapper.findAll('[role="alert"]').map((alert) => alert.text())).toContain(
      "打开日志文件夹失败：访问被拒绝",
    );

    wrapper.unmount();
    host.remove();
  });

  it.each(["停止采集", "停止控制环"])(
    "records in VLA and closes the runtime with %s",
    async (stop) => {
      const backend = new TestBackend(memoryServoCatalog);
      await backend.invoke("policy_download", { id: "alpha-walking" });
      await backend.invoke("policy_download", { id: "alpha-stand" });
      await backend.invoke("policy_select", { id: "alpha-walking" });
      const { host, wrapper, store } = await mountApp(backend);
      await buttonByText(wrapper, "ONNX").trigger("click");
      await flushPromises();
      store.state.loop = {
        ...store.state.loop!,
        world: { position: [2, 1], yaw: 0.4, distanceM: 3 },
      };
      await buttonByText(wrapper, "开始采集").trigger("click");
      await flushPromises();
      expect(store.state.loop).toMatchObject({
        recording: true,
        phase: "driving",
        activeBody: { physicalMachineId: null },
        world: { position: [0, 0], yaw: 0, distanceM: 0 },
      });
      const policies = wrapper.get('[aria-label="已缓存 ONNX 策略"]').findAll("button");
      expect(policies).toHaveLength(2);
      expect(policies.every((button) => button.attributes("disabled") !== undefined)).toBe(true);

      await buttonByText(wrapper, stop).trigger("click");
      await flushPromises();
      expect(store.state.loop).toBeNull();
      expect(await commands.loopStatus()).toBeNull();
      await wrapper.get('button[aria-label="设置"]').trigger("click");
      await wrapper.get('button[aria-label="日志"]').trigger("click");
      await flushPromises();
      const [record] = await commands.recordingList();
      expect(record).toMatchObject({ state: "complete", name: "Alpha Walking" });
      expect(record.id).toMatch(/^Alpha-Walking-/);
      expect(wrapper.text()).toContain(record.name);
      expect(wrapper.text()).not.toContain("诊断事件");
      await buttonByText(wrapper, "原始日志").trigger("click");
      await buttonByText(wrapper, "动作轨迹数据").trigger("click");
      await flushPromises();
      expect(
        backend.calls
          .filter((call) => call.name === "recording_reveal")
          .map((call) => call.payload),
      ).toEqual([
        { id: record.id, motion: false },
        { id: record.id, motion: true },
      ]);
      wrapper.unmount();
      host.remove();
    },
  );

  it("creates a virtual machine from the simulation panel", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "模拟机").trigger("click");
    await flushPromises();
    await wrapper.get('button[aria-label="新建模拟机"]').trigger("click");
    const name = wrapper.get('input[placeholder="模拟机名称"]');
    await name.setValue("测试模拟机");
    await buttonByText(wrapper, "创建").trigger("click");
    await flushPromises();

    expect(backend.calls.map((call) => call.name)).toContain("virtual_machine_create");
    expect(store.state.virtualMachines.some((machine) => machine.name === "测试模拟机")).toBe(true);

    wrapper.unmount();
    host.remove();
  });

  it("selects the native HD1910 simulation model independently of the servo", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "模拟机").trigger("click");
    await flushPromises();
    await wrapper.get('button[aria-label="新建模拟机"]').trigger("click");
    const model = wrapper.get<HTMLSelectElement>('select[aria-label="仿真模型"]');
    expect(model.element.value).toBe("hd1910-compatible");
    await model.setValue("hd1910");
    await wrapper.get('input[placeholder="模拟机名称"]').setValue("HD1910 测试机");
    await buttonByText(wrapper, "创建").trigger("click");
    await flushPromises();
    expect(
      store.state.virtualMachines.find((machine) => machine.name === "HD1910 测试机"),
    ).toMatchObject({ executionKind: "simulation", servoModel: "hd1910", modelKind: "hd1910" });
    expect(wrapper.text()).toContain("HD1910 原生");
    wrapper.unmount();
    host.remove();
  });

  it("selects the clicked 3D duck for preview and debug", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    const startupMachine = store.state.virtualMachines[0];
    store.state.virtualMachines.push({
      ...startupMachine,
      id: "virtual-2",
      name: "机器 1",
      startupSelected: false,
      referencedBy: [],
    });
    await wrapper.vm.$nextTick();

    await wrapper.get('[data-testid="duck-viewport"]').trigger("click", {
      clientX: 100,
      clientY: 100,
    });
    expect(store.state.selectedVirtualMachine).toBe("virtual-2");
    expect(store.state.selectedVirtualMachineManual).toBe(true);
    expect(store.state.virtualMachines.map((machine) => machine.id)).toEqual([
      startupMachine.id,
      "virtual-2",
    ]);

    await buttonByText(wrapper, "调试").trigger("click");
    await flushPromises();
    expect(wrapper.text()).toContain("机器 1");

    wrapper.unmount();
    host.remove();
  });

  it("shows the load error at the frontend and recovers through retry", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    backend.failCatalog = true;
    const { host, wrapper, store } = await mountApp(backend);

    expect(store.state.error).toBe("模拟配置读取失败");
    expect(wrapper.text()).toContain("读取失败");
    backend.failCatalog = false;
    await buttonByText(wrapper, "重新加载").trigger("click");
    await flushPromises();

    expect(store.state.error).toBeNull();
    expect(store.state.catalog).toHaveLength(2);
    expect(wrapper.text()).toContain("ONNX");

    wrapper.unmount();
    host.remove();
  });
});
