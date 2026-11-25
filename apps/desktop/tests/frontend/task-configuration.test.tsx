import { mount, flushPromises } from "@vue/test-utils";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as commands from "../../generated/types";
import { TaskConfiguration } from "../../src/features/training/task-configuration";
import { taskConfiguration } from "../fixtures/taskConfiguration.fixture";

describe("selected task configuration", () => {
  afterEach(() => vi.restoreAllMocks());

  it("reads the selected project's task defaults and shows the training, play and RL configurations", async () => {
    const query = vi
      .spyOn(commands, "trainingTaskConfiguration")
      .mockResolvedValue(taskConfiguration("task-a"));
    const wrapper = mount(TaskConfiguration, {
      props: { taskId: "task-a" },
    });
    expect(wrapper.text()).toContain("正在读取");
    await flushPromises();
    expect(query).toHaveBeenCalledWith("task-a");
    expect(wrapper.text()).toContain("64 / 1");
    expect(wrapper.text()).toContain("50 / 50 Hz");
    expect(wrapper.text()).toContain("20 / 40 s");
    expect(wrapper.text()).toContain("fixture.Runner");
    expect(wrapper.text()).toContain("下方训练参数会覆盖");
    expect(wrapper.findAll("details").map((section) => section.get("pre").text())).toEqual([
      '{"mode":"play"}',
      '{"mode":"training"}',
      '{"algorithm":"PPO"}',
    ]);
    wrapper.unmount();
  });

  it("discards an older response after changing task and reuses a loaded task", async () => {
    let resolve!: (value: Awaited<ReturnType<typeof commands.trainingTaskConfiguration>>) => void;
    const query = vi
      .spyOn(commands, "trainingTaskConfiguration")
      .mockReturnValueOnce(
        new Promise((done) => {
          resolve = done;
        }),
      )
      .mockResolvedValue({ ...taskConfiguration("task-b"), algorithm: "B" });
    const wrapper = mount(TaskConfiguration, {
      props: { taskId: "task-a" },
    });
    await wrapper.setProps({ taskId: "task-b" });
    await flushPromises();
    resolve({ ...taskConfiguration("task-a"), algorithm: "STALE" });
    await flushPromises();
    expect(wrapper.text()).not.toContain("STALE");
    await wrapper.setProps({ taskId: "task-a" });
    await flushPromises();
    expect(wrapper.text()).toContain("STALE");
    expect(query).toHaveBeenCalledTimes(2);
    wrapper.unmount();
  });

  it("clears old details on a task change and displays a Python error", async () => {
    const query = vi
      .spyOn(commands, "trainingTaskConfiguration")
      .mockResolvedValue(taskConfiguration("task-a"));
    const wrapper = mount(TaskConfiguration, {
      props: { taskId: "task-a" },
    });
    await flushPromises();
    query.mockRejectedValue(new Error("unknown task"));
    await wrapper.setProps({ taskId: "missing-task" });
    await flushPromises();
    expect(query).toHaveBeenLastCalledWith("missing-task");
    expect(wrapper.findAll("details")).toHaveLength(0);
    expect(wrapper.text()).toContain("配置读取失败");
    expect(wrapper.text()).toContain("unknown task");
    wrapper.unmount();
  });
});
