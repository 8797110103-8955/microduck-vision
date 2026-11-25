import { flushPromises, type VueWrapper } from "@vue/test-utils";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as commands from "../../generated/types";
import { MemoryBackend } from "../bridge/memory";
import { TestBackend } from "../bridge/test-backend";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";
import { buttonByText, mountApp, setupAppTests } from "./app-harness";

const round = {
  record: {
    runId: "run-parent",
    experimentId: "walk",
    round: 1,
    taskId: "Microduck-DeepMimic-Velstand-Flat-v1",
    initialization: null,
  },
  progress: null,
  checkpoints: [
    { id: "model_10", file: "logs/model_10.pt", sha256: "a".repeat(64), iteration: 10 },
  ],
} satisfies Awaited<ReturnType<typeof commands.trainingRounds>>[number];

const reward = {
  id: "velstand-bundle",
  name: "Velstand",
  recording_id: "velstand-1795452546188-10300-1795452570447-0",
  body_id: "official",
  frames: 352,
  duration_s: 7.02,
  control_s: 0.02,
  playback: "finite",
} satisfies Awaited<ReturnType<typeof commands.deepmimicBundles>>[number];

function field(wrapper: VueWrapper, text: string) {
  const label = wrapper.findAll("label").find((item) => item.find("span").text() === text);
  if (!label) throw new Error(`missing ${text}`);
  return label.get("input,select");
}

async function savedProgress() {
  const progress = await new MemoryBackend(memoryServoCatalog).trainingStart({
    program: "uv",
    args: [],
    virtualMachineId: "test-machine",
    policyId: null,
    experimentId: null,
    recipe: "RL",
    teacherMotionProvenance: "test",
  });
  progress.metadata.servoProfile = "xl330";
  progress.metadata.deepmimic = {
    trainingMethod: "deepmimic",
    bundleId: reward.id,
    bundleManifestSha256: "b".repeat(64),
    clipSha256: "c".repeat(64),
    sourceRecordingId: reward.recording_id,
    seed: 0,
    numEnvs: 64,
    iterations: 50,
    metadataFile: "training-metadata.json",
  };
  return progress;
}

async function openTraining() {
  vi.spyOn(commands, "deepmimicBundles").mockResolvedValue([reward]);
  const app = await mountApp(new TestBackend(memoryServoCatalog));
  app.store.state.virtualMachines.push({
    ...app.store.state.virtualMachines[0],
    id: "test-xl330",
    name: "XL330",
    modelKind: "xl330",
    servoModel: "xl330",
  });
  app.store.state.selectedVirtualMachine = "test-xl330";
  await buttonByText(app.wrapper, "训练").trigger("click");
  await flushPromises();
  return app;
}

describe("training rounds and explicit publication", () => {
  setupAppTests();
  afterEach(() => vi.restoreAllMocks());

  it("defaults to the latest checkpoint for the same task and motor", async () => {
    const saved = await savedProgress();
    const latest = {
      ...round,
      progress: saved,
      record: { ...round.record, runId: "run-latest", round: 2 },
      checkpoints: [
        ...round.checkpoints,
        { ...round.checkpoints[0], id: "model_20", iteration: 20 },
      ],
    };
    vi.spyOn(commands, "trainingRounds").mockResolvedValue([
      { ...round, progress: saved },
      {
        ...latest,
        record: { ...latest.record, runId: "run-hd1910", round: 3 },
        progress: { ...saved, metadata: { ...saved.metadata, servoProfile: "hd1910" } },
      },
      latest,
      {
        ...round,
        progress: saved,
        record: { ...round.record, runId: "run-other", experimentId: "other" },
      },
    ]);
    const start = vi.spyOn(commands, "trainingStart").mockRejectedValue(new Error("test stop"));
    const { wrapper, host } = await openTraining();
    expect(field(wrapper, "训练规则").element).toHaveProperty("value", "resume");
    expect(field(wrapper, "训练历史").element).toHaveProperty("value", "walk");
    expect(field(wrapper, "历史轮次（继承或发布）").element).toHaveProperty("value", "run-latest");
    expect(field(wrapper, "历史轮次（继承或发布）").findAll("option")).toHaveLength(2);
    expect(wrapper.text()).not.toContain("Checkpoint");
    await buttonByText(wrapper, "开始训练").trigger("click");
    await flushPromises();
    expect(start).toHaveBeenLastCalledWith(
      expect.objectContaining({
        initialization: {
          mode: "resume",
          parentRunId: "run-latest",
          checkpoint: "model_20",
          resetNormalization: false,
        },
      }),
    );
    const mode = field(wrapper, "训练规则");
    await mode.setValue("fresh");
    expect(field(wrapper, "删除同名训练").element).toHaveProperty("checked", true);
    await field(wrapper, "删除同名训练").setValue(false);
    await field(wrapper, "训练 ID").setValue("other");
    start.mockClear();
    expect(buttonByText(wrapper, "开始训练").attributes("disabled")).toBeUndefined();
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).not.toHaveBeenCalled();
    const message = wrapper.get('[data-message-type="error"]');
    expect(message.text()).toContain("训练 ID 已存在");
    await message.get("button").trigger("click");
    await field(wrapper, "删除同名训练").setValue(true);
    await buttonByText(wrapper, "开始训练").trigger("click");
    expect(start).toHaveBeenCalledWith(
      expect.objectContaining({ experimentId: "other", replaceExistingExperiment: true }),
    );
    await flushPromises();
    await field(wrapper, "训练历史").setValue("other");
    const history = field(wrapper, "历史轮次（继承或发布）");
    expect(history.element).toHaveProperty("value", "run-other");
    expect(history.findAll("option")).toHaveLength(1);
    await mode.setValue("adapt");
    await buttonByText(wrapper, "开始训练").trigger("click");
    await flushPromises();
    expect(start).toHaveBeenLastCalledWith(
      expect.objectContaining({
        experimentId: "other",
        initialization: {
          mode: "adapt",
          parentRunId: "run-other",
          checkpoint: "model_10",
          resetNormalization: false,
        },
      }),
    );
    wrapper.unmount();
    host.remove();
  });

  it("defaults to fresh training for a task without a usable checkpoint", async () => {
    vi.spyOn(commands, "trainingRounds").mockResolvedValue([{ ...round, checkpoints: [] }]);
    const { wrapper, host } = await openTraining();
    expect(field(wrapper, "训练规则").element).toHaveProperty("value", "fresh");
    expect(
      field(wrapper, "训练规则").get('option[value="resume"]').attributes("disabled"),
    ).toBeDefined();
    expect(wrapper.text()).not.toContain("训练历史");
    expect(field(wrapper, "训练 ID").element).toHaveProperty(
      "value",
      expect.stringMatching(/^deepmimic-velstand-[a-f0-9]{7}$/),
    );
    wrapper.unmount();
    host.remove();
  });

  it("opens a separate publication page only for a finished round", async () => {
    const progress = await savedProgress();
    vi.spyOn(commands, "trainingRounds").mockResolvedValue([
      {
        ...round,
        record: { ...round.record, runId: "run-active", round: 2 },
        progress: { ...progress, runId: "run-active", step: 2 },
      },
      {
        ...round,
        progress: {
          ...progress,
          runId: round.record.runId,
          phase: "finished",
          message: "示例训练已完成",
          step: 10,
        },
      },
      {
        ...round,
        record: {
          ...round.record,
          experimentId: "ppo-baseline-velstand-1234567",
          runId: "run-baseline",
        },
        progress: { ...progress, runId: "run-baseline", phase: "finished", step: 50 },
      },
      {
        ...round,
        record: { ...round.record, experimentId: "other", runId: "run-other" },
        progress: {
          ...progress,
          runId: "run-other",
          phase: "finished",
          step: 25,
        },
      },
    ]);
    const openResult = vi.spyOn(commands, "trainingOpenResult").mockResolvedValue(null);
    vi.spyOn(commands, "trainingPublicationTargets").mockResolvedValue([
      { id: "published-xl330", name: "XL330 collection", servoModel: "xl330" },
      { id: "published-hd1910", name: "HD1910 collection", servoModel: "hd1910" },
    ]);
    const publishedPolicy = {
      id: "training:published-a",
      name: "Walk stable",
      file: "Walk stable.onnx",
      sourceUrl: "",
      cached: true,
      selected: false,
      size: 100,
      kind: "training",
      slot: "walk",
      durationS: null,
      command: null,
    } satisfies Awaited<ReturnType<typeof commands.trainingPublish>>;
    const publish = vi.spyOn(commands, "trainingPublish").mockResolvedValue(publishedPolicy);
    const scene = await new MemoryBackend(memoryServoCatalog).officialSceneModel();
    const preview = vi.spyOn(commands, "trainingPreview").mockResolvedValue({
      report: {
        directory: "trainings/previews/example",
        runId: round.record.runId,
        checkpointId: round.checkpoints[0].id,
        policySha256: "a".repeat(64),
        executionFingerprint: "test-body",
        command: {
          twist: [0.15, 0, 0],
          head: [0, 0, 0, 0],
          body: [0, 0, 0],
          targetDistanceM: null,
        },
        steps: 500,
        policySteps: 500,
        seconds: 10,
        displacementM: [0.2, 0],
        minHeightM: 0.15,
        maxTiltDeg: 10,
        uprightFraction: 0.95,
      },
      scene,
      sampleIntervalMs: 40,
      frames: [],
    });
    const { wrapper, host, store } = await openTraining();
    store.applyTrainingProgress({ ...progress, runId: "run-active", phase: "running", step: 2 });
    await flushPromises();
    expect(publish).not.toHaveBeenCalled();
    expect(field(wrapper, "训练历史").text()).not.toContain("ppo-baseline-velstand-1234567");
    expect(buttonByText(wrapper, "策略发布").attributes("disabled")).toBeDefined();
    expect(field(wrapper, "历史轮次（继承或发布）").attributes("disabled")).toBeUndefined();
    await field(wrapper, "历史轮次（继承或发布）").setValue(round.record.runId);
    await buttonByText(wrapper, "策略发布").trigger("click");
    await flushPromises();
    const page = wrapper.get('section[aria-label="策略发布"]');
    expect(
      page
        .get("header")
        .findAll("button")
        .map((button) => button.text()),
    ).toEqual(["预览", "发布", "打开记录", "返回训练"]);
    expect(page.text()).not.toContain("示例训练已完成");
    expect(page.text()).not.toContain("结果目录");
    expect(page.text()).not.toContain("开始训练");
    expect(page.text()).toContain("step 10");
    expect(field(wrapper, "训练历史").text()).toContain("ppo-baseline-velstand-1234567");
    const appendTo = field(wrapper, "追加到");
    expect(appendTo.element).toHaveProperty("value", "");
    expect(appendTo.findAll("option").map((option) => option.text())).toEqual([
      "新建发布",
      "XL330 collection",
    ]);
    await buttonByText(wrapper, "预览").trigger("click");
    await flushPromises();
    expect(preview).toHaveBeenCalledWith({
      runId: round.record.runId,
      checkpointId: round.checkpoints[0].id,
      virtualMachineId: "test-xl330",
    });
    expect(page.text()).toContain("预览 10.0 秒，前向位移 0.20 米，直立占比 95%");
    expect(publish).not.toHaveBeenCalled();
    expect(field(wrapper, "策略名称").element).toHaveProperty(
      "value",
      "deepmimic-velstand-aaaaaaa",
    );
    expect(buttonByText(wrapper, "发布").attributes("disabled")).toBeUndefined();
    await field(wrapper, "策略名称").setValue("   ");
    expect(buttonByText(wrapper, "发布").attributes("disabled")).toBeUndefined();
    await field(wrapper, "历史轮次（继承或发布）").setValue("run-active");
    expect(page.text()).toContain("step 2");
    expect(page.text()).not.toContain("直立占比");
    expect(buttonByText(wrapper, "预览").attributes("disabled")).toBeUndefined();
    await field(wrapper, "策略名称").setValue("unfinished");
    expect(buttonByText(wrapper, "发布").attributes("disabled")).toBeDefined();
    await field(wrapper, "训练历史").setValue("other");
    expect(page.text()).toContain("step 25");
    await buttonByText(wrapper, "打开记录").trigger("click");
    expect(openResult).toHaveBeenCalledWith("run-other");
    await field(wrapper, "策略名称").setValue("duplicate");
    vi.spyOn(commands, "policyList").mockResolvedValueOnce([
      { ...publishedPolicy, name: "Duplicate" },
    ]);
    await buttonByText(wrapper, "发布").trigger("click");
    await flushPromises();
    expect(publish).not.toHaveBeenCalled();
    expect(page.text()).toContain("已有同名策略");
    await field(wrapper, "策略名称").setValue("Walk stable");
    expect(buttonByText(wrapper, "发布").attributes("disabled")).toBeUndefined();
    await buttonByText(wrapper, "发布").trigger("click");
    await flushPromises();
    expect(publish).toHaveBeenCalledWith("run-other", "model_10", "Walk stable", null);
    expect(wrapper.text()).toContain("已发布：Walk stable");
    await field(wrapper, "追加到").setValue("published-xl330");
    await field(wrapper, "训练历史").setValue("ppo-baseline-velstand-1234567");
    expect(field(wrapper, "追加到").element).toHaveProperty("value", "published-xl330");
    await field(wrapper, "策略名称").setValue("Walk baseline");
    await buttonByText(wrapper, "发布").trigger("click");
    await flushPromises();
    expect(publish).toHaveBeenLastCalledWith(
      "run-baseline",
      "model_10",
      "Walk baseline",
      "published-xl330",
    );
    await buttonByText(wrapper, "返回训练").trigger("click");
    expect(wrapper.get('section[aria-label="本地训练"]').text()).toContain("停止");
    wrapper.unmount();
    host.remove();
  });
});
