import { flushPromises, mount, type VueWrapper } from "@vue/test-utils";
import { createPinia } from "pinia";
import { afterEach, beforeEach, vi } from "vitest";
import { clearMocks, mockIPC, mockWindows } from "@tauri-apps/api/mocks";
import { App } from "../../src/features/app";
import { useDesktopStore } from "../../src/store";
import type { TestBackend } from "../bridge/test-backend";

const viewerSetFrame = vi.hoisted(() => vi.fn());
const viewerSetSimulationFrame = vi.hoisted(() => vi.fn());
const viewerSetWorldPose = vi.hoisted(() => vi.fn());

vi.mock("../../src/components/duck-stage/viewer", () => ({
  DuckViewer: class {
    onRender: (() => void) | null = null;
    dispose() {}
    markerScreen() {
      return null;
    }
    setFrame = viewerSetFrame;
    setSimulationFrame = viewerSetSimulationFrame;
    setMachineWorldPoses() {}
    setGhosts() {}
    setSelectedDuck() {}
    setWorldPose = viewerSetWorldPose;
    pickDuck() {
      return 1;
    }
    focusCurrentDuck() {}
    restorePreviousView() {}
  },
}));

export { viewerSetFrame, viewerSetSimulationFrame, viewerSetWorldPose };

export function buttonByText(wrapper: VueWrapper, text: string) {
  const button = wrapper.findAll("button").find((candidate) => candidate.text() === text);
  if (!button) throw new Error(`button not found: ${text}`);
  return button;
}

export async function mountApp(backend: TestBackend) {
  mockWindows("main");
  mockIPC((command, payload) => backend.invoke(command, payload), { shouldMockEvents: true });
  const host = document.createElement("div");
  document.body.append(host);
  const wrapper = mount(App, {
    attachTo: host,
    global: { plugins: [createPinia()] },
  });
  await flushPromises();
  return { host, wrapper, store: useDesktopStore() };
}

export function setupAppTests() {
  beforeEach(() => {
    document.body.innerHTML = "";
    localStorage.clear();
    viewerSetFrame.mockClear();
    viewerSetSimulationFrame.mockClear();
    viewerSetWorldPose.mockClear();
  });

  afterEach(() => {
    clearMocks();
    vi.useRealTimers();
  });
}
