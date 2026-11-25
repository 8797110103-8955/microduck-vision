import * as commands from "../../../generated/types";
import { defineComponent, ref, shallowRef, watch } from "vue";
import style from "./style.module.scss";

export const TaskConfiguration = defineComponent({
  props: {
    taskId: { type: String, required: true },
  },
  setup(props) {
    const configuration = shallowRef<Awaited<
      ReturnType<typeof commands.trainingTaskConfiguration>
    > | null>(null);
    const loading = ref(false);
    const error = ref<string | null>(null);
    const cache = new Map<string, NonNullable<typeof configuration.value>>();
    watch(
      () => props.taskId,
      async (taskId, _, onCleanup) => {
        let current = true;
        onCleanup(() => {
          current = false;
        });
        configuration.value = null;
        error.value = null;
        loading.value = false;
        if (!taskId) return;
        const key = taskId;
        const cached = cache.get(key);
        if (cached) {
          configuration.value = cached;
          return;
        }
        loading.value = true;
        try {
          const result = await commands.trainingTaskConfiguration(taskId);
          cache.set(key, result);
          if (current) configuration.value = result;
        } catch (reason) {
          if (current) error.value = reason instanceof Error ? reason.message : String(reason);
        } finally {
          if (current) loading.value = false;
        }
      },
      { immediate: true },
    );
    return () =>
      props.taskId && (
        <section
          class={style.taskConfiguration}
          aria-label="注册任务配置"
          aria-busy={loading.value}
        >
          <strong>任务配置</strong>
          {loading.value && <p role="status">正在读取所选任务的 Python 配置…</p>}
          {error.value && (
            <p class={style.error}>配置读取失败，请检查 Python 环境和训练项目：{error.value}</p>
          )}
          {configuration.value && (
            <>
              <p>
                注册任务的默认配置；下方训练参数会覆盖对应默认值。play
                是演示环境配置，不代表已通过验证。
              </p>
              <dl>
                <dt>算法</dt>
                <dd>{configuration.value.algorithm}</dd>
                <dt>迭代数</dt>
                <dd>{configuration.value.maxIterations}</dd>
                <dt>环境数（训练 / play）</dt>
                <dd>
                  {configuration.value.training.numEnvs} / {configuration.value.play.numEnvs}
                </dd>
                <dt>控制频率（训练 / play）</dt>
                <dd>
                  {configuration.value.training.controlHz} / {configuration.value.play.controlHz} Hz
                </dd>
                <dt>回合时长（训练 / play）</dt>
                <dd>
                  {configuration.value.training.episodeLengthS} /{" "}
                  {configuration.value.play.episodeLengthS} s
                </dd>
                <dt>地形（训练 / play）</dt>
                <dd>
                  {configuration.value.training.terrain ?? "无"} /{" "}
                  {configuration.value.play.terrain ?? "无"}
                </dd>
                <dt>Runner</dt>
                <dd>{configuration.value.runner}</dd>
              </dl>
              {[
                ["演示环境（play）", configuration.value.play.json],
                ["训练环境", configuration.value.training.json],
                ["RL 参数", configuration.value.rlJson],
              ].map(([label, json]) => (
                <details key={`${props.taskId}:${label}`}>
                  <summary>{label}</summary>
                  <pre>{json}</pre>
                </details>
              ))}
            </>
          )}
        </section>
      );
  },
});
