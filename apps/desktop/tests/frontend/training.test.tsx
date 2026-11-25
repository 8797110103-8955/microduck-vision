import {
  buttonByText,
  mountApp,
  setupAppTests,
  viewerSetFrame,
  viewerSetSimulationFrame,
} from "./app-harness";
import { flushPromises } from "@vue/test-utils";
import { describe, expect, it, vi } from "vitest";
import { TestBackend } from "../bridge/test-backend";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";
import * as commands from "../../generated/types";
import { emit } from "@tauri-apps/api/event";
import { OfficialDuckStage } from "../../src/components/duck-stage";
import { MemoryBackend } from "../bridge/memory";

describe("desktop training flow", () => {
  setupAppTests();

  it("displays only current training frames and leaves active runtime state intact", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const { host, wrapper, store } = await mountApp(backend);
    const live = await commands.runtimeOpen({
      virtualMachineId: "official-hd1910",
      physicalMachineId: null,
      access: "read_only",
      policyId: null,
    });
    store.applyLoopStatus(live);
    const training = new MemoryBackend(memoryServoCatalog);
    const input = {
      program: "uv",
      args: ["run", "train", "task"],
      virtualMachineId: "official-hd1910",
      policyId: null,
      experimentId: null,
      recipe: "preview",
      teacherMotionProvenance: "test-fixture",
    };
    const first = await training.trainingStart(input);
    const second = await training.trainingStart(input);
    const scene = await commands.officialSceneModel();
    const frame: commands.TrainingFrame = {
      runId: first.runId,
      sequence: 2,
      environmentIndex: 0,
      modelKind: "hd1910-compatible",
      executionFingerprint: "preview-test-profile",
      positions: [0, null],
      trunkPosition: [0.05, -0.02, 0.125],
      trunkQuaternion: [1, 0, 0, 0],
      frame: scene.defaultFrame,
      previewScene: { ...scene, meshDir: "training-task-meshes" },
    };

    await buttonByText(wrapper, "训练").trigger("click");
    await emit("training-progress", first);
    await emit("training-frame", frame);
    await flushPromises();
    expect(viewerSetSimulationFrame).toHaveBeenLastCalledWith(frame, true);
    const count = viewerSetSimulationFrame.mock.calls.length;
    await emit("training-frame", { ...frame, sequence: 1 });
    await emit("training-frame", { ...frame, runId: second.runId, sequence: 3 });
    await flushPromises();
    expect(viewerSetSimulationFrame).toHaveBeenCalledTimes(count);

    await emit("training-frame", { ...frame, sequence: 3, previewScene: null });
    await flushPromises();
    expect(wrapper.getComponent(OfficialDuckStage).props("view").scene).toEqual(frame.previewScene);

    await emit("training-progress", second);
    await emit("training-frame", { ...frame, sequence: 3 });
    const secondFrame = {
      ...frame,
      runId: second.runId,
      sequence: 1,
      modelKind: "xl330",
      previewScene: null,
    };
    await emit("training-frame", secondFrame);
    await flushPromises();
    expect(viewerSetSimulationFrame).toHaveBeenLastCalledWith(secondFrame, true);
    expect(wrapper.getComponent(OfficialDuckStage).props("view").scene).toEqual(scene);

    await emit("training-progress", { ...second, phase: "finished" });
    await emit("training-frame", { ...secondFrame, sequence: 2 });
    await flushPromises();
    expect(store.state.trainingFrame).toBeNull();
    expect(viewerSetFrame).toHaveBeenLastCalledWith(scene.defaultFrame);
    await buttonByText(wrapper, "训练").trigger("click");
    await flushPromises();
    expect(store.state.loop).toEqual(live);
    wrapper.unmount();
    host.remove();
  });

  it("adds finished training policies to preview without replacing the current selection", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const invoke = backend.invoke.bind(backend);
    const local: commands.PolicyEntry = {
      id: "training:run-preview",
      name: "本地 XL330",
      file: "output.onnx",
      sourceUrl: "",
      kind: "training",
      slot: "walk",
      cached: true,
      selected: false,
      size: 800,
      durationS: null,
      command: null,
    };
    const { wrapper, host, store } = await mountApp(backend);
    const known = [...store.state.policies];
    vi.spyOn(backend, "invoke").mockImplementation(async (command, payload) => {
      const value = await invoke(command, payload);
      return command === "policy_list" ? [...known, local] : value;
    });
    store.state.selectedPolicyId = "alpha-walking";
    const training = new MemoryBackend(memoryServoCatalog);
    const progress = await training.trainingStart({
      program: "uv",
      args: ["run", "train", "fixture"],
      virtualMachineId: "official-hd1910",
      policyId: null,
      experimentId: null,
      recipe: "preview",
      teacherMotionProvenance: "fixture",
    });
    await emit("training-progress", { ...progress, phase: "finished" });
    await flushPromises();
    await buttonByText(wrapper, "ONNX").trigger("click");
    await flushPromises();
    expect(buttonByText(wrapper, local.name).exists()).toBe(true);
    expect(store.state.selectedPolicyId).toBe("alpha-walking");
    wrapper.unmount();
    host.remove();
  });

  it("hides teacher evaluation while DeepMimic is the only enabled task", async () => {
    const backend = new TestBackend(memoryServoCatalog);
    const invoke = backend.invoke.bind(backend);
    const teacher: commands.PolicyEntry = {
      id: "training:student-1",
      name: "教师学生模型",
      file: "student.onnx",
      sourceUrl: "",
      kind: "teacher-training",
      slot: "walk",
      cached: true,
      selected: false,
      size: 800,
      durationS: null,
      command: null,
    };
    vi.spyOn(backend, "invoke").mockImplementation((command, payload) => {
      if (command === "policy_list") return Promise.resolve([teacher]);
      return invoke(command, payload);
    });
    const { wrapper, host } = await mountApp(backend);
    await buttonByText(wrapper, "训练").trigger("click");
    expect(wrapper.find('section[aria-label="教师仿真评估"]').exists()).toBe(false);
    wrapper.unmount();
    host.remove();
  });
});
