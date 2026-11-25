import { flushPromises, type VueWrapper } from "@vue/test-utils";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as commands from "../../generated/types";
import { ScrollArea } from "../../src/components/scroll-area";
import { TestBackend } from "../bridge/test-backend";
import { memoryServoCatalog } from "../fixtures/servoCatalog.fixture";
import { buttonByText, mountApp, setupAppTests } from "./app-harness";

function taskField(wrapper: VueWrapper) {
  const row = wrapper.findAll("label").find((label) => label.find("span").text() === "任务");
  if (!row) throw new Error("missing task selector");
  return row;
}

describe("registered training tasks", () => {
  setupAppTests();
  afterEach(() => vi.restoreAllMocks());

  it("queries at startup, displays the Python list and reuses it across page visits", async () => {
    let resolve!: (tasks: string[]) => void;
    const query = vi.spyOn(commands, "trainingTasks").mockReturnValue(
      new Promise((done) => {
        resolve = done;
      }),
    );
    const { wrapper, host, store } = await mountApp(new TestBackend(memoryServoCatalog));
    expect(query).toHaveBeenCalledOnce();
    expect(store.state.tab).toBe("home");
    await buttonByText(wrapper, "训练").trigger("click");
    expect(taskField(wrapper).get("select").attributes("disabled")).toBeDefined();
    expect(taskField(wrapper).text()).toContain("正在读取任务");
    const deepmimic = "Microduck-DeepMimic-Velstand-Flat-v1";
    resolve(["Task-A", deepmimic]);
    await flushPromises();
    const field = taskField(wrapper);
    expect(field.get("select").attributes("disabled")).toBeUndefined();
    expect(field.findAll("option").map((option) => option.attributes("value"))).toEqual([
      "",
      "Task-A",
      deepmimic,
    ]);
    expect(field.get('option[value="Task-A"]').attributes("disabled")).toBeDefined();
    await field.get("select").setValue("");
    expect(field.get("select").element).toHaveProperty("value", deepmimic);
    await buttonByText(wrapper, "训练").trigger("click");
    await buttonByText(wrapper, "训练").trigger("click");
    expect(query).toHaveBeenCalledOnce();
    wrapper.unmount();
    host.remove();
  });

  it.each(["无法启动 uv", "ModuleNotFoundError: mjlab", "empty"])(
    "disables the selector and shows an environment hint for %s",
    async (reason) => {
      const query = vi.spyOn(commands, "trainingTasks");
      if (reason === "empty") query.mockResolvedValue([]);
      else query.mockRejectedValue(new Error(reason));
      const { wrapper, host } = await mountApp(new TestBackend(memoryServoCatalog));
      await buttonByText(wrapper, "训练").trigger("click");
      const field = taskField(wrapper);
      expect(field.get("select").attributes("disabled")).toBeDefined();
      expect(field.attributes("title")).toContain("请排查 Python 环境");
      expect(field.attributes("title")).toContain(
        reason === "empty" ? "未返回已注册的任务" : reason,
      );
      expect(buttonByText(wrapper, "开始训练").attributes("disabled")).toBeDefined();
      wrapper.unmount();
      host.remove();
    },
  );

  it("uses the task as the only entry point and keeps the entire form inside ScrollArea", async () => {
    const { wrapper, host } = await mountApp(new TestBackend(memoryServoCatalog));
    await buttonByText(wrapper, "训练").trigger("click");
    const page = wrapper.get('section[aria-label="本地训练"]');
    const scroll = page.getComponent(ScrollArea);
    expect(scroll.find("header").exists()).toBe(true);
    expect(scroll.findAll("label")[0].find("span").text()).toBe("任务");
    expect(page.text()).not.toContain("训练方式");
    await flushPromises();
    expect(taskField(wrapper).get("select").attributes("disabled")).toBeUndefined();
    expect(page.text()).toContain("奖励来源");
    expect(
      taskField(wrapper)
        .get('option[value="Mjlab-Velocity-Flat-MicroDuck"]')
        .attributes("disabled"),
    ).toBeDefined();
    const form = scroll.text();
    expect(form).toContain("开始训练");
    expect(form.indexOf("任务配置")).toBeGreaterThan(form.indexOf("开始训练"));
    wrapper.unmount();
    host.remove();
  });
});
