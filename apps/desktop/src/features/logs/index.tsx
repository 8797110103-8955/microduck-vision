import { defineComponent, onMounted, onScopeDispose, ref } from "vue";
import { useIntervalFn } from "@vueuse/core";
import * as commands from "../../../generated/types";
import style from "./style.module.scss";

const stateLabels: Record<string, string> = {
  recording: "采集中",
  complete: "已完成",
  incomplete: "有缺帧",
  interrupted: "已中断",
  failed: "失败",
};

export const LogsPage = defineComponent(() => {
  const records = ref<Awaited<ReturnType<typeof commands.recordingList>>>([]);
  const loaded = ref(false);
  const error = ref("");
  let disposed = false;
  let refreshing = false;

  const run = async (operation: () => Promise<unknown>) => {
    error.value = "";
    try {
      await operation();
    } catch (reason) {
      if (!disposed) error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const refresh = async () => {
    if (refreshing) return;
    refreshing = true;
    try {
      const list = await commands.recordingList();
      if (!disposed) {
        records.value = list;
        loaded.value = true;
      }
    } catch (reason) {
      if (!disposed) error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      refreshing = false;
    }
  };

  onMounted(() => void refresh());
  useIntervalFn(() => void refresh(), 2000);
  onScopeDispose(() => {
    disposed = true;
  });

  return () => (
    <div class={style.page}>
      <header>
        <h1>日志</h1>
        <button type="button" onClick={() => void run(() => commands.openLogs())}>
          打开日志文件夹
        </button>
      </header>
      {error.value && (
        <p role="alert" class={style.error}>
          {error.value}
        </p>
      )}
      {loaded.value && records.value.length === 0 && <p>暂无采集记录</p>}
      <ul class={style.records}>
        {records.value.map((record) => (
          <li key={record.id} class={style.record}>
            <div class={style.details}>
              <strong>{record.name}</strong>
              <span>
                {record.startedAtMs === null
                  ? "时间未知"
                  : new Date(record.startedAtMs).toLocaleString()}{" "}
                · {stateLabels[record.state] ?? record.state} · {record.frames} 帧
                {record.droppedFrames > 0 && ` · 丢帧 ${record.droppedFrames}`}
              </span>
              {record.error && <span class={style.error}>{record.error}</span>}
            </div>
            <div class={style.files}>
              <button
                type="button"
                onClick={() => void run(() => commands.recordingReveal(record.id, false))}
              >
                原始日志
              </button>
              <button
                type="button"
                onClick={() => void run(() => commands.recordingReveal(record.id, true))}
              >
                动作轨迹数据
              </button>
            </div>
          </li>
        ))}
      </ul>
    </div>
  );
});
