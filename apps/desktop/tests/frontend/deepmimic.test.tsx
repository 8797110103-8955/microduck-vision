import { flushPromises, mount } from "@vue/test-utils";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import * as commands from "../../generated/types";
import { DeepMimicConversion } from "../../src/features/logs/deepmimic";

function scan(id = "recording-a"): Awaited<ReturnType<typeof commands.deepmimicIntervals>> {
  return {
    recording_id: id,
    mcap_sha256: "a".repeat(64),
    frames_read: 154,
    excluded_reasons: {},
    read_failure: null,
    intervals: [
      {
        id: `${id}-0`,
        body_id: "official",
        model_kind: "xl330",
        control_s: 0.02,
        segment_id: 0,
        execution_generation: 1,
        first_frame: 0,
        last_frame: 52,
        first_simulation_s: 0.02,
        last_simulation_s: 1.06,
        frames: 53,
        excluded_reasons: {},
      },
      {
        id: `${id}-53`,
        body_id: "official",
        model_kind: "xl330",
        control_s: 0.02,
        segment_id: 1,
        execution_generation: 2,
        first_frame: 53,
        last_frame: 153,
        first_simulation_s: 2,
        last_simulation_s: 4,
        frames: 101,
        excluded_reasons: {},
      },
    ],
  };
}

const request: Parameters<typeof commands.deepmimicConvert>[0] = {
  recording_id: "recording-a",
  interval_id: "recording-a-0",
  start_s: 0,
  end_s: 1.04,
  name: "Walk DeepMimic",
  playback: "finite",
};

function job(
  overrides: Partial<Awaited<ReturnType<typeof commands.deepmimicStatus>>> = {},
): Awaited<ReturnType<typeof commands.deepmimicStatus>> {
  return {
    id: "job-a",
    bundle_id: "bundle-a",
    request,
    status: "running",
    stage: "converting",
    processed_frames: 10,
    total_frames: 53,
    error: null,
    ...overrides,
  };
}

const bundle = {
  id: "bundle-a",
  name: request.name,
  recording_id: request.recording_id,
  body_id: "official",
  frames: 53,
  duration_s: 1.04,
  control_s: 0.02,
  playback: "finite",
} satisfies Awaited<ReturnType<typeof commands.deepmimicBundles>>[number];

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((done) => {
    resolve = done;
  });
  return { promise, resolve };
}

describe("DeepMimic recording conversion", () => {
  const mounted: ReturnType<typeof mount>[] = [];
  const panel = async () => {
    const wrapper = mount(DeepMimicConversion, {
      props: { recordingId: "recording-a", recordingName: "Walk" },
    });
    mounted.push(wrapper);
    await flushPromises();
    return wrapper;
  };

  beforeEach(() => {
    vi.spyOn(commands, "deepmimicIntervals").mockImplementation(async (id) => scan(id));
    vi.spyOn(commands, "deepmimicJobs").mockResolvedValue([]);
    vi.spyOn(commands, "deepmimicBundles").mockResolvedValue([]);
    vi.spyOn(commands, "deepmimicConvert").mockResolvedValue(job());
    vi.spyOn(commands, "deepmimicStatus").mockResolvedValue(job());
    vi.spyOn(commands, "deepmimicCancel").mockResolvedValue(job({ status: "cancelled" }));
    vi.spyOn(commands, "deepmimicOpenBundle").mockResolvedValue(null);
  });
  afterEach(() => {
    for (const wrapper of mounted.splice(0)) wrapper.unmount();
    vi.restoreAllMocks();
    vi.useRealTimers();
  });

  it("defaults to finite and resets times when changing interval, while suppressing duplicate submits", async () => {
    const pending = deferred<Awaited<ReturnType<typeof commands.deepmimicConvert>>>();
    vi.mocked(commands.deepmimicConvert).mockReturnValue(pending.promise);
    const wrapper = await panel();
    const inputs = wrapper.findAll("input");
    expect(inputs.map((input) => input.element.value)).toEqual(["Walk DeepMimic", "0", "1.04"]);
    expect(wrapper.text()).toContain("official · xl330 · 控制周期 20 ms");
    const [interval, playback] = wrapper.findAll("select");
    expect(playback.element.value).toBe("finite");
    await inputs[1].setValue("0.2");
    await interval.setValue("recording-a-53");
    expect(inputs[1].element.value).toBe("0");
    expect(inputs[2].element.value).toBe("2");
    await playback.setValue("cyclic");
    await wrapper.get("form").trigger("submit");
    await wrapper.get("form").trigger("submit");
    expect(commands.deepmimicConvert).toHaveBeenCalledOnce();
    expect(commands.deepmimicConvert).toHaveBeenCalledWith({
      ...request,
      interval_id: "recording-a-53",
      end_s: 2,
      playback: "cyclic",
    });
    pending.resolve(job());
    await flushPromises();
    await wrapper.get("form").trigger("submit");
    expect(commands.deepmimicConvert).toHaveBeenCalledOnce();
    expect(wrapper.text()).toContain("转换中");
  });

  it("rejects too few samples, displays conversion errors, and permits a retry after cancellation", async () => {
    const wrapper = await panel();
    const inputs = wrapper.findAll("input");
    await inputs[2].setValue("0.02");
    await wrapper.get("form").trigger("submit");
    expect(wrapper.get('[role="alert"]').text()).toContain("至少 3 帧");
    expect(commands.deepmimicConvert).not.toHaveBeenCalled();
    await inputs[2].setValue("1.04");
    vi.mocked(commands.deepmimicConvert).mockRejectedValueOnce(
      "Pinned training project lock mismatch",
    );
    await wrapper.get("form").trigger("submit");
    await flushPromises();
    expect(wrapper.get('[role="alert"]').text()).toContain("lock mismatch");
    await wrapper.get("form").trigger("submit");
    await flushPromises();
    await wrapper
      .findAll("button")
      .find((button) => button.text() === "取消转换")!
      .trigger("click");
    await flushPromises();
    expect(commands.deepmimicCancel).toHaveBeenCalledWith("job-a");
    expect(wrapper.text()).toContain("已取消");
    expect(wrapper.get("fieldset").element.disabled).toBe(false);
    await wrapper.get("form").trigger("submit");
    expect(commands.deepmimicConvert).toHaveBeenCalledTimes(3);
  });

  it("keeps the rounded full-clip endpoint inside native form bounds", async () => {
    const report = scan();
    report.intervals[0].last_simulation_s = 7.039999999999872;
    report.intervals[0].frames = 352;
    vi.mocked(commands.deepmimicIntervals).mockResolvedValue(report);
    const wrapper = await panel();
    const end = wrapper.findAll("input")[2];
    expect(end.element.value).toBe("7.02");
    expect(Number(end.attributes("max"))).toBeGreaterThanOrEqual(Number(end.element.value));
    expect(end.element.checkValidity()).toBe(true);
    await wrapper.get("form").trigger("submit");
    expect(commands.deepmimicConvert).toHaveBeenCalledWith(
      expect.objectContaining({ end_s: 7.02 }),
    );
  });

  it("polls by job ID, displays published bundles, and restores them when reopened", async () => {
    vi.useFakeTimers();
    vi.mocked(commands.deepmimicJobs).mockResolvedValue([job()]);
    const wrapper = await panel();
    vi.mocked(commands.deepmimicStatus).mockResolvedValue(
      job({ status: "succeeded", stage: "saving", processed_frames: 53 }),
    );
    vi.mocked(commands.deepmimicBundles).mockResolvedValue([bundle]);
    await vi.advanceTimersByTimeAsync(1000);
    await flushPromises();
    expect(commands.deepmimicStatus).toHaveBeenCalledWith("job-a");
    expect(wrapper.text()).toContain("53 帧 · 1.04 秒 · finite");
    await wrapper
      .findAll("button")
      .find((button) => button.text() === "打开动作包")!
      .trigger("click");
    expect(commands.deepmimicOpenBundle).toHaveBeenCalledWith("bundle-a");
    wrapper.unmount();
    mounted.splice(mounted.indexOf(wrapper), 1);
    expect(vi.getTimerCount()).toBe(0);
    vi.mocked(commands.deepmimicJobs).mockResolvedValue([job({ status: "succeeded" })]);
    const reopened = await panel();
    expect(reopened.text()).toContain("已完成");
    expect(reopened.text()).toContain("打开动作包");
  });

  it("ignores the previous recording's late scan result", async () => {
    const old = deferred<Awaited<ReturnType<typeof commands.deepmimicIntervals>>>();
    vi.mocked(commands.deepmimicIntervals).mockReturnValueOnce(old.promise);
    const wrapper = await panel();
    await wrapper.setProps({ recordingId: "recording-b", recordingName: "New walk" });
    await flushPromises();
    await wrapper.findAll("select")[0].setValue("recording-b-53");
    old.resolve(scan());
    await flushPromises();
    expect(wrapper.findAll("select")[0].element.value).toBe("recording-b-53");
    expect(wrapper.findAll("input")[0].element.value).toBe("New walk DeepMimic");
    expect(wrapper.findAll("input")[2].element.value).toBe("2");
  });

  it("ignores an in-flight progress response after cancellation and stops polling when disposed", async () => {
    vi.useFakeTimers();
    vi.mocked(commands.deepmimicJobs).mockResolvedValue([job()]);
    const old = deferred<Awaited<ReturnType<typeof commands.deepmimicStatus>>>();
    vi.mocked(commands.deepmimicStatus).mockReturnValue(old.promise);
    const wrapper = await panel();
    await vi.advanceTimersByTimeAsync(1000);
    await wrapper
      .findAll("button")
      .find((button) => button.text() === "取消转换")!
      .trigger("click");
    await flushPromises();
    old.resolve(job());
    await flushPromises();
    expect(wrapper.text()).toContain("已取消");
    expect(wrapper.text()).not.toContain("取消转换");
    wrapper.unmount();
    mounted.splice(mounted.indexOf(wrapper), 1);
    await vi.advanceTimersByTimeAsync(3000);
    expect(commands.deepmimicStatus).toHaveBeenCalledOnce();
    expect(vi.getTimerCount()).toBe(0);
  });

  it("shows truncated-source diagnostics and persistent failed-job details without enabling conversion", async () => {
    vi.mocked(commands.deepmimicIntervals).mockResolvedValue({
      ...scan(),
      read_failure: { bytes_read: 123, last_reliable_frame: 42, reason: "Unexpected EOF" },
    });
    vi.mocked(commands.deepmimicJobs).mockResolvedValue([
      job({ status: "failed", error: "Cyclic seam exceeds threshold" }),
    ]);
    const wrapper = await panel();
    expect(wrapper.text()).toContain("最后可靠帧：42");
    expect(wrapper.text()).toContain("Unexpected EOF");
    expect(wrapper.text()).toContain("Cyclic seam exceeds threshold");
    expect(wrapper.get('button[type="submit"]').attributes("disabled")).toBeDefined();
    await wrapper.get("form").trigger("submit");
    expect(commands.deepmimicConvert).not.toHaveBeenCalled();
  });
});
