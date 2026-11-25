import { flushPromises, type VueWrapper } from "@vue/test-utils";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as commands from "../../generated/types";
import { TestBackend } from "../bridge/test-backend";
import { MemoryBackend } from "../bridge/memory";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";
import { buttonByText, mountApp, setupAppTests } from "./app-harness";

const bundle = {
  id: "bundle-a",
  name: "Velstand reference",
  recording_id: "velstand-1795452546188-10300-1795452570447-0",
  body_id: "official",
  frames: 352,
  duration_s: 7.02,
  control_s: 0.02,
  playback: "finite",
} satisfies Awaited<ReturnType<typeof commands.deepmimicBundles>>[number];

function field(wrapper: VueWrapper, label: string) {
  const row = wrapper.findAll("label").find((candidate) => candidate.find("span").text() === label);
  if (!row) throw new Error(`missing field ${label}`);
  return row.get("input,select");
}

async function open() {
  const app = await mountApp(new TestBackend(memoryServoCatalog));
  const machine = app.store.state.virtualMachines[0];
  app.store.state.virtualMachines.push({
    ...machine,
    id: "official-xl330",
    modelKind: "xl330",
    servoModel: "xl330",
    name: "XL330",
  });
  app.store.state.selectedVirtualMachine = "official-xl330";
  await buttonByText(app.wrapper, "训练").trigger("click");
  await flushPromises();
  return app;
}

async function progress() {
  return new MemoryBackend(memoryServoCatalog).trainingStart({
    program: "uv",
    args: [],
    virtualMachineId: "official-xl330",
    policyId: null,
    experimentId: null,
    recipe: "deepmimic",
    teacherMotionProvenance: "bundle-a",
  });
}

describe("DeepMimic training requests", () => {
  setupAppTests();
  afterEach(() => vi.restoreAllMocks());

  it("uses the current XL330 and sends one structured request with the first experiment defaults", async () => {
    vi.spyOn(commands, "deepmimicBundles").mockResolvedValue([
      bundle,
      { ...bundle, id: "other", body_id: "other" },
    ]);
    vi.spyOn(commands, "trainingStatus").mockResolvedValue(null);
    const { wrapper, host, store } = await open();
    store.state.teacherDatasetPath = "previous-bc-dataset";
    await field(wrapper, "任务").setValue("Microduck-DeepMimic-Velstand-Flat-v1");
    await flushPromises();
    expect(wrapper.get('[aria-label="本地训练"] h2').text()).toBe("XL330");
    expect(field(wrapper, "并行环境数").element).toHaveProperty("value", "64");
    expect(field(wrapper, "迭代数").element).toHaveProperty("value", "50");
    expect(field(wrapper, "随机种子").element).toHaveProperty("value", "0");
    expect(field(wrapper, "奖励来源").findAll("option")).toHaveLength(2);
    const experimentId = wrapper.get<HTMLInputElement>('input[placeholder^="deepmimic-velstand-"]')
      .element.value;
    expect(experimentId).toMatch(/^deepmimic-velstand-[a-f0-9]{7}$/);
    let reject!: (reason: Error) => void;
    const start = vi.spyOn(commands, "trainingStart").mockReturnValue(
      new Promise((_, fail) => {
        reject = fail;
      }),
    );
    await buttonByText(wrapper, "开始训练").trigger("click");
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).toHaveBeenCalledOnce();
    expect(start).toHaveBeenCalledWith(
      expect.objectContaining({
        args: [],
        datasetPath: null,
        policyId: null,
        virtualMachineId: "official-xl330",
        experimentId,
        deepmimic: { bundleId: "bundle-a", numEnvs: 64, iterations: 50, seed: 0, baseline: false },
      }),
    );
    reject(new Error("reference history unavailable"));
    await flushPromises();
    expect(wrapper.text()).toContain("reference history unavailable");
    expect(buttonByText(wrapper, "开始训练").attributes("disabled")).toBeUndefined();
    wrapper.unmount();
    host.remove();
  });

  it("blocks invalid numeric values and incompatible machines, and recovers bundle listing errors", async () => {
    const list = vi
      .spyOn(commands, "deepmimicBundles")
      .mockRejectedValue(new Error("cannot read bundles"));
    vi.spyOn(commands, "trainingStatus").mockResolvedValue(null);
    const start = vi.spyOn(commands, "trainingStart");
    const { wrapper, host, store } = await open();
    await field(wrapper, "任务").setValue("Microduck-DeepMimic-Velstand-Flat-v1");
    await flushPromises();
    expect(wrapper.text()).toContain("cannot read bundles");
    expect(buttonByText(wrapper, "开始训练").attributes("disabled")).toBeDefined();
    list.mockResolvedValue([bundle]);
    await buttonByText(wrapper, "刷新动作包").trigger("click");
    await flushPromises();
    await field(wrapper, "奖励来源").setValue(bundle.id);
    await field(wrapper, "迭代数").setValue("0");
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).not.toHaveBeenCalled();
    expect(wrapper.text()).toContain("环境数和迭代数须为正整数");
    const current = store.state.virtualMachines[0];
    store.state.selectedVirtualMachine = current.id;
    await flushPromises();
    expect(store.state.selectedVirtualMachine).toBe(current.id);
    expect(wrapper.get('[aria-label="本地训练"] h2').text()).toBe(current.name);
    expect(wrapper.text()).toContain("DeepMimic 需要 official XL330 模拟机");
    expect(buttonByText(wrapper, "开始训练").attributes("disabled")).toBeDefined();
    wrapper.unmount();
    host.remove();
  });

  it("opens the baseline page with the current reward and toggles one start and stop button", async () => {
    const saved = { ...(await progress()), phase: "failed" as const, message: "saved failure" };
    const source = {
      trainingMethod: "deepmimic",
      bundleId: bundle.id,
      bundleManifestSha256: "a".repeat(64),
      clipSha256: "b".repeat(64),
      sourceRecordingId: bundle.recording_id,
      seed: 7,
      numEnvs: 4,
      iterations: 3,
      metadataFile: "training-metadata.json",
    } satisfies commands.TrainingProvenance;
    saved.metadata.deepmimic = source;
    const baselineId = "ppo-baseline-velstand-1234567";
    const normalId = "deepmimic-velstand-1234567";
    vi.spyOn(commands, "trainingRounds").mockResolvedValue([
      {
        record: {
          runId: "saved-normal",
          experimentId: normalId,
          round: 1,
          taskId: "Microduck-DeepMimic-Velstand-Flat-v1",
          initialization: null,
        },
        progress: {
          ...saved,
          phase: "finished",
          metadata: { ...saved.metadata, servoProfile: "xl330", experimentId: normalId },
        },
        checkpoints: [{ id: "model_3", file: "model_3.pt", sha256: "a".repeat(64), iteration: 3 }],
      },
      {
        record: {
          runId: "saved-baseline",
          experimentId: baselineId,
          round: 1,
          taskId: "Microduck-DeepMimic-Velstand-Flat-v1",
          initialization: null,
        },
        progress: {
          ...saved,
          phase: "finished",
          metadata: {
            ...saved.metadata,
            servoProfile: "xl330",
            experimentId: baselineId,
            deepmimic: { ...source, trainingMethod: "ppo-baseline", seed: 12 },
          },
        },
        checkpoints: [{ id: "model_3", file: "model_3.pt", sha256: "a".repeat(64), iteration: 3 }],
      },
    ]);
    vi.spyOn(commands, "trainingStatus").mockResolvedValue(saved);
    vi.spyOn(commands, "deepmimicBundles").mockResolvedValue([bundle]);
    const { wrapper, host } = await open();
    expect(field(wrapper, "任务").element).toHaveProperty(
      "value",
      "Microduck-DeepMimic-Velstand-Flat-v1",
    );
    expect(field(wrapper, "随机种子").element).toHaveProperty("value", "7");
    expect(wrapper.text()).toContain(saved.resultDir);
    expect(wrapper.text()).toContain("saved failure");
    const openResult = vi.spyOn(commands, "trainingOpenResult").mockResolvedValue(null);
    await buttonByText(wrapper, "打开结果目录").trigger("click");
    expect(openResult).toHaveBeenCalledWith(saved.runId);
    expect(wrapper.text()).not.toContain("PPO 基线");
    expect(wrapper.text()).not.toContain(baselineId);
    await buttonByText(wrapper, "基线对照").trigger("click");
    const page = wrapper.get('section[aria-label="基线对照"]');
    expect(page.text()).toContain("当前动作");
    expect(page.text()).toContain(bundle.name);
    const history = field(wrapper, "训练历史");
    expect(history.text()).toContain(baselineId);
    expect(history.text()).toContain(normalId);
    expect(history.element).toHaveProperty("value", normalId);
    await history.setValue(baselineId);
    expect(field(wrapper, "随机种子").element).toHaveProperty("value", "12");
    await history.setValue(normalId);
    expect(field(wrapper, "随机种子").element).toHaveProperty("value", "7");
    const baseline = field(wrapper, "PPO 基线");
    expect(baseline.element).toHaveProperty("checked", true);
    expect(baseline.attributes("disabled")).toBeDefined();
    const started = await progress();
    const start = vi.spyOn(commands, "trainingStart").mockImplementation(async (input) => ({
      ...started,
      metadata: {
        ...started.metadata,
        experimentId: input.experimentId ?? "",
        deepmimic: {
          ...source,
          trainingMethod: "ppo-baseline",
        },
      },
    }));
    const stop = vi.spyOn(commands, "trainingStop").mockResolvedValue(null);
    await buttonByText(wrapper, "开始训练").trigger("click");
    await flushPromises();
    expect(start).toHaveBeenCalledWith(
      expect.objectContaining({
        recipe: "ppo-baseline",
        experimentId: expect.stringMatching(/^ppo-baseline-velstand-[a-f0-9]{7}$/),
        initialization: null,
        deepmimic: { bundleId: bundle.id, baseline: true, numEnvs: 4, iterations: 3, seed: 7 },
      }),
    );
    expect(page.text()).not.toContain("开始训练");
    await buttonByText(wrapper, "停止").trigger("click");
    expect(stop).toHaveBeenCalledOnce();
    wrapper.unmount();
    host.remove();
  });

  it("uses the selected historical round's DeepMimic settings when continuing", async () => {
    const saved = { ...(await progress()), phase: "finished" as const };
    const replacement = { ...bundle, id: "bundle-b", name: "Walking reference" };
    saved.metadata.servoProfile = "xl330";
    saved.metadata.deepmimic = {
      trainingMethod: "deepmimic",
      bundleId: bundle.id,
      bundleManifestSha256: "a".repeat(64),
      clipSha256: "b".repeat(64),
      sourceRecordingId: bundle.recording_id,
      seed: 7,
      numEnvs: 4,
      iterations: 3,
      metadataFile: "training-metadata.json",
    };
    vi.spyOn(commands, "trainingStatus").mockResolvedValue(null);
    vi.spyOn(commands, "deepmimicBundles").mockResolvedValue([bundle, replacement]);
    vi.spyOn(commands, "trainingRounds").mockResolvedValue([
      {
        record: {
          runId: saved.runId,
          experimentId: "velstand",
          round: 1,
          taskId: "Microduck-DeepMimic-Velstand-Flat-v1",
          initialization: null,
        },
        progress: saved,
        checkpoints: [{ id: "model_3", file: "model_3.pt", sha256: "a".repeat(64), iteration: 3 }],
      },
    ]);
    const { wrapper, host } = await open();
    await field(wrapper, "任务").setValue("Microduck-DeepMimic-Velstand-Flat-v1");
    await flushPromises();
    const mode = field(wrapper, "训练规则");
    const source = field(wrapper, "奖励来源");
    expect(mode.get('option[value="resume"]').text()).toBe(`继续训练（${bundle.name}）`);
    expect(source.element).toHaveProperty("value", bundle.id);
    expect(source.attributes("disabled")).toBeDefined();
    const start = vi.spyOn(commands, "trainingStart").mockRejectedValue(new Error("test stop"));
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).toHaveBeenCalledWith(
      expect.objectContaining({
        initialization: expect.objectContaining({
          mode: "resume",
          parentRunId: saved.runId,
          checkpoint: "model_3",
        }),
        deepmimic: { bundleId: bundle.id, numEnvs: 4, iterations: 3, seed: 7, baseline: false },
      }),
    );
    await flushPromises();
    await mode.setValue("adapt");
    expect(mode.get('option[value="adapt"]').text()).toBe("继续训练（切换奖励来源）");
    expect(source.attributes("disabled")).toBeUndefined();
    await source.setValue(replacement.id);
    await source.setValue("");
    expect(source.element).toHaveProperty("value", replacement.id);
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).toHaveBeenLastCalledWith(
      expect.objectContaining({
        initialization: expect.objectContaining({ mode: "adapt", parentRunId: saved.runId }),
        deepmimic: expect.objectContaining({ bundleId: replacement.id }),
      }),
    );
    await flushPromises();
    wrapper.unmount();
    host.remove();
  });

  it("ignores an older bundle response after refreshing or disposing the page", async () => {
    let resolve!: (value: (typeof bundle)[]) => void;
    vi.spyOn(commands, "trainingStatus").mockResolvedValue(null);
    vi.spyOn(commands, "deepmimicBundles")
      .mockReturnValueOnce(
        new Promise((done) => {
          resolve = done;
        }),
      )
      .mockResolvedValue([bundle]);
    const { wrapper, host } = await open();
    await buttonByText(wrapper, "训练").trigger("click");
    await buttonByText(wrapper, "训练").trigger("click");
    await flushPromises();
    resolve([]);
    await flushPromises();
    expect(field(wrapper, "奖励来源").text()).toContain(bundle.name);
    wrapper.unmount();
    host.remove();
    await flushPromises();
  });
});
