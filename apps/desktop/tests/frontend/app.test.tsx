import { flushPromises, mount, type VueWrapper } from "@vue/test-utils";
import { createPinia } from "pinia";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { clearMocks, mockIPC, mockWindows } from "@tauri-apps/api/mocks";
import { createTauriBackend } from "../../src/bridge";
import { openLoop } from "../../src/features/debug";
import { disconnectPhysicalMachine } from "../../src/features/machines";
import { App } from "../../src/features/app";
import { configureDesktopBackend, useDesktopStore } from "../../src/store";
import { TestBackend } from "../bridge/test-backend";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";

vi.mock("../../src/features/home/viewer", () => ({
  DuckViewer: class {
    onRender: (() => void) | null = null;
    dispose() {}
    markerScreen() {
      return null;
    }
    setFrame() {}
    setGhosts() {}
  },
}));

function buttonByText(wrapper: VueWrapper, text: string) {
  const button = wrapper.findAll("button").find((candidate) => candidate.text() === text);
  if (!button) throw new Error(`button not found: ${text}`);
  return button;
}

async function mountApp(backend: TestBackend) {
  mockWindows("main");
  mockIPC((command, payload) => backend.invoke(command, payload), { shouldMockEvents: true });
  configureDesktopBackend(createTauriBackend());
  const host = document.createElement("div");
  document.body.append(host);
  const wrapper = mount(App, {
    attachTo: host,
    global: { plugins: [createPinia()] },
  });
  await flushPromises();
  return { host, wrapper, store: useDesktopStore() };
}

describe("desktop frontend flow", () => {
  beforeEach(() => {
    document.body.innerHTML = "";
    localStorage.clear();
  });

  afterEach(() => {
    clearMocks();
  });

  it("boots from the frontend, detects and saves hardware, then drives the loop", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);

    expect(store.state.catalog.map((entry) => entry.key)).toEqual(["hd1910", "xl330"]);
    expect(store.state.selectedVirtualMachine).toBe("official-hd1910");
    expect(backend.calls.filter((call) => call.name === "servo_catalog")).toHaveLength(1);
    expect(wrapper.text()).toContain("预览");

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

    const machineRow = wrapper.find("article");
    await machineRow.find("select").setValue("official-hd1910");
    await flushPromises();
    await buttonByText(wrapper, "扫描").trigger("click");
    await flushPromises();

    expect(store.state.physicalMachines[0].virtualMachineId).toBe("official-hd1910");
    expect(store.state.physicalMachines[0].onlineServoIds).toEqual([1, 2]);
    expect(store.state.virtualMachines[0].referencedBy).toContain("physical-1");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "调试").trigger("click");
    expect(wrapper.text()).toContain("机器 0");
    expect(wrapper.text()).not.toContain("参数沿用旧版舵机调试面板");
    expect(wrapper.text()).not.toContain("舵机参数");
    await openLoop([1, 2]);
    await wrapper.vm.$nextTick();
    await buttonByText(wrapper, "发送微调目标").trigger("click");
    await flushPromises();
    expect(store.state.loop?.targets).toEqual([0.01, 0.01]);

    await buttonByText(wrapper, "推进一拍").trigger("click");
    await flushPromises();
    expect(store.state.loop?.positions).toEqual([0.01, 0.01]);
    await buttonByText(wrapper, "停止控制环").trigger("click");
    await flushPromises();
    expect(store.state.loop).toBeNull();

    wrapper.unmount();
    host.remove();
  });

  it("opens the preview overlay and selects a cached policy for the current machine", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);

    await buttonByText(wrapper, "预览").trigger("click");
    await flushPromises();
    expect(store.state.tab).toBe("preview");
    expect(wrapper.text()).toContain("当前电脑还没有已缓存的 ONNX");
    await buttonByText(wrapper, "开启控制环").trigger("click");
    await flushPromises();
    expect(store.state.previewLoop).toMatchObject({ phase: "ready", torque: false });
    expect(backend.calls.map((call) => call.name)).toContain("preview_loop_open");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "策略").trigger("click");
    await buttonByText(wrapper, "下载").trigger("click");
    await flushPromises();
    expect(wrapper.text()).toContain("已缓存");

    await wrapper.get('button[aria-label="设置"]').trigger("click");
    await buttonByText(wrapper, "预览").trigger("click");
    await flushPromises();
    expect(store.state.tab).toBe("preview");
    expect(wrapper.text()).toContain("Alpha Walking");
    await buttonByText(wrapper, "Alpha Walking").trigger("click");
    await flushPromises();
    expect(store.state.selectedPolicyId).toBe("alpha-walking");
    expect(backend.calls.map((call) => call.name)).toContain("policy_select");
    expect(store.state.previewLoop?.torque).toBe(true);
    expect(backend.calls.map((call) => call.name)).toContain("preview_loop_policy");
    await buttonByText(wrapper, "停止控制环").trigger("click");
    await flushPromises();
    expect(store.state.previewLoop).toBeNull();

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
    expect(wrapper.text()).toContain("预览");

    wrapper.unmount();
    host.remove();
  });
});
