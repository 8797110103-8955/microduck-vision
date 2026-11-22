import { computed, defineComponent, onMounted, onScopeDispose, ref } from "vue";
import { useIntervalFn } from "@vueuse/core";
import * as commands from "../../../generated/types";
import { useDesktopStore } from "../../store";
import { OfficialDuckStage } from "../home";
import style from "./style.module.scss";

export const LogsPage = defineComponent(() => {
  const store = useDesktopStore();
  const records = ref<Awaited<ReturnType<typeof commands.recordingList>>>([]);
  const diagnostics = ref<string[]>([]);
  const frames = ref<Awaited<ReturnType<typeof commands.recordingFrames>>>([]);
  const referenceFrames = ref<typeof frames.value>([]);
  const left = ref("");
  const right = ref("");
  const segment = ref(0);
  const referenceSegment = ref(0);
  const start = ref(0);
  const end = ref(86400);
  const referenceStart = ref(0);
  const name = ref("");
  const index = ref(0);
  const playing = ref(false);
  const busy = ref(false);
  const error = ref("");
  const result = ref("");
  const dataset = ref("");
  const category = ref("");
  const level = ref("");
  const query = ref("");
  const plotField = ref("positions");
  const plotIndex = ref(0);
  const comparison = ref<Awaited<ReturnType<typeof commands.recordingCompare>>>();
  const current = computed(() => records.value.find((item) => item.state === "recording"));
  const selected = computed(() => records.value.find((item) => item.id === left.value));
  const sample = computed(() => frames.value[index.value]);
  const segments = computed(() => [...new Set(frames.value.map((frame) => frame.segmentId))]);
  const shownDiagnostics = computed(() =>
    diagnostics.value.filter((line) => {
      if (query.value && !line.toLowerCase().includes(query.value.toLowerCase())) return false;
      try {
        const event: unknown = JSON.parse(line);
        if (!event || typeof event !== "object") return false;
        if (category.value && (!("category" in event) || event.category !== category.value))
          return false;
        if (level.value && (!("level" in event) || event.level !== level.value)) return false;
        return true;
      } catch {
        return !category.value && !level.value;
      }
    }),
  );
  const window = () => ({
    id: left.value,
    segmentId: segment.value,
    startS: start.value,
    endS: end.value,
  });
  let alive = true;
  const run = async (operation: () => Promise<void>) => {
    if (busy.value) return;
    busy.value = true;
    error.value = "";
    try {
      await operation();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      busy.value = false;
    }
  };
  const refresh = async () => {
    const [list, tail] = await Promise.all([commands.recordingList(), commands.diagnosticTail()]);
    if (!alive) return;
    records.value = list;
    diagnostics.value = tail;
  };
  const load = async () => {
    playing.value = false;
    const loaded = await commands.recordingFrames(window());
    if (!alive) return;
    frames.value = loaded;
    index.value = 0;
    comparison.value = undefined;
    referenceFrames.value = [];
    store.state.recordingPlayback = null;
    if (sample.value) store.state.recordingPlayback = sample.value;
  };
  const playSample = () => {
    if (sample.value) store.state.recordingPlayback = sample.value;
  };
  const { pause } = useIntervalFn(() => {
    if (!playing.value || !frames.value.length) return;
    if (index.value >= frames.value.length - 1) {
      playing.value = false;
      return;
    }
    const now = performance.now();
    const target = playbackTime + (now - playbackStart) / 1000;
    while (
      index.value < frames.value.length - 1 &&
      (frames.value[index.value + 1].elapsedS ?? 0) <= target
    ) {
      index.value += 1;
    }
    playSample();
  }, 20);
  let playbackStart = 0;
  let playbackTime = 0;
  const startPlayback = () => {
    playbackStart = performance.now();
    playbackTime = sample.value?.elapsedS ?? 0;
  };
  const points = (items: typeof frames.value, offset: number) => {
    if (!items.length) return "";
    const first = frames.value[0]?.elapsedS ?? 0;
    const duration = Math.max(0.02, (frames.value.at(-1)?.elapsedS ?? first) - first);
    return items
      .map((frame) => {
        const x = (((frame.elapsedS ?? 0) - offset - first) / duration) * 700;
        const value =
          plotField.value === "targets"
            ? frame.status.targets[plotIndex.value]
            : plotField.value === "action"
              ? frame.policy?.action[plotIndex.value]
              : plotField.value === "observation"
                ? frame.policy?.observation[plotIndex.value]
                : frame.status.positions[plotIndex.value];
        if (value == null) return "";
        const y = 75 - value * 30;
        return `${x},${y}`;
      })
      .join(" ");
  };
  onMounted(() => void run(refresh));
  useIntervalFn(() => {
    if (!busy.value)
      void refresh().catch((reason: unknown) => {
        if (alive) error.value = reason instanceof Error ? reason.message : String(reason);
      });
  }, 2000);
  onScopeDispose(() => {
    alive = false;
    pause();
    store.state.recordingPlayback = null;
  });

  return () => (
    <div class={style.page}>
      <header>
        <h1>日志</h1>
        <button onClick={() => void commands.openLogs()}>打开日志文件夹</button>
      </header>
      <section>
        <h2>运行采集</h2>
        <div class={style.controls}>
          <input aria-label="采集名称" placeholder="采集名称" v-model={name.value} />
          <button
            disabled={busy.value || !!current.value || !store.state.loop?.activeBody}
            onClick={() =>
              void run(async () => {
                await commands.recordingStart(name.value);
                await refresh();
              })
            }
          >
            开始采集
          </button>
          <button
            disabled={busy.value || !current.value}
            onClick={() =>
              void run(async () => {
                await commands.recordingStop();
                await refresh();
              })
            }
          >
            停止采集
          </button>
          <button disabled={busy.value} onClick={() => void run(refresh)}>
            刷新
          </button>
        </div>
        {current.value && (
          <p role="status">
            正在采集 {current.value.name} · {current.value.frames} 帧 · 丢帧{" "}
            {current.value.droppedFrames}
          </p>
        )}
        {records.value.length === 0 ? (
          <p>启动控制环后，可以采集当前鸭子的运行数据。</p>
        ) : (
          <div class={style.table}>
            <table>
              <thead>
                <tr>
                  <th>记录</th>
                  <th>鸭子</th>
                  <th>状态</th>
                  <th>帧／丢帧</th>
                  <th>时间</th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {records.value.map((record) => (
                  <tr key={record.id}>
                    <td title={record.path}>{record.name}</td>
                    <td>{record.duckId}</td>
                    <td>
                      {{
                        recording: "采集中",
                        complete: "已完成",
                        incomplete: "有缺帧",
                        interrupted: "已中断",
                        failed: "失败",
                      }[record.state] ?? record.state}
                      {record.error && <p>{record.error}</p>}
                    </td>
                    <td>
                      {record.frames}／{record.droppedFrames}
                    </td>
                    <td>{new Date(record.startedAtMs ?? 0).toLocaleString()}</td>
                    <td>
                      <button
                        disabled={busy.value || record.state === "recording"}
                        onClick={() =>
                          void run(async () => {
                            left.value = record.id;
                            segment.value = 0;
                            start.value = 0;
                            end.value = 86400;
                            await load();
                          })
                        }
                      >
                        查看
                      </button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </section>
      {selected.value && (
        <section>
          <h2>{selected.value.name}</h2>
          <p class={style.path}>{selected.value.path}</p>
          {store.state.recordingPlayback && (
            <div class={style.viewer}>
              <OfficialDuckStage />
            </div>
          )}
          <div class={style.controls}>
            <label>
              片段{" "}
              <select v-model={segment.value}>
                <option value={0}>全部</option>
                {segments.value.map((id) => (
                  <option value={id}>{id}</option>
                ))}
              </select>
            </label>
            <label>
              开始秒 <input type="number" min="0" step="0.02" v-model={start.value} />
            </label>
            <label>
              结束秒 <input type="number" min="0" step="0.02" v-model={end.value} />
            </label>
            <button disabled={busy.value} onClick={() => void run(load)}>
              读取片段
            </button>
          </div>
          <div class={style.controls}>
            <label>
              曲线{" "}
              <select v-model={plotField.value}>
                <option value="positions">关节反馈 rad</option>
                <option value="targets">关节目标 rad</option>
                <option value="action">策略 action</option>
                <option value="observation">策略 observation</option>
              </select>
            </label>
            <label>
              分量（从 0 开始）
              <input type="number" min="0" max="60" v-model={plotIndex.value} />
            </label>
          </div>
          <svg
            viewBox="0 0 700 150"
            class={style.chart}
            role="img"
            aria-label="所选运行字段随时间变化"
          >
            <line x1="0" y1="75" x2="700" y2="75" stroke="#c6c0ad" />
            <polyline
              points={points(frames.value, 0)}
              fill="none"
              stroke="#997000"
              stroke-width="2"
            />
            <polyline
              points={points(referenceFrames.value, referenceStart.value - start.value)}
              fill="none"
              stroke="#258a86"
              stroke-width="2"
            />
          </svg>
          <p>黄线为当前记录，绿线为对比记录 · 回放最多显示 2000 帧，比较与导出使用完整数据。</p>
          <div class={style.controls}>
            <button
              disabled={!frames.value.length}
              onClick={() => {
                playing.value = !playing.value;
                startPlayback();
                playSample();
              }}
            >
              {playing.value ? "暂停" : "播放"}
            </button>
            <input
              aria-label="回放位置"
              type="range"
              min="0"
              max={Math.max(0, frames.value.length - 1)}
              value={index.value}
              onInput={(event) => {
                if (event.target instanceof HTMLInputElement) {
                  index.value = Number(event.target.value);
                  playing.value = false;
                  playSample();
                }
              }}
            />
            {sample.value && (
              <span>
                {(sample.value.elapsedS ?? 0).toFixed(2)} s · 片段 {sample.value.segmentId} · tick{" "}
                {sample.value.status.tick}
              </span>
            )}
            <button
              disabled={!store.state.recordingPlayback}
              onClick={() => {
                playing.value = false;
                store.state.recordingPlayback = null;
              }}
            >
              结束回放
            </button>
          </div>
          {sample.value && (
            <div class={style.table}>
              <table>
                <thead>
                  <tr>
                    <th>关节序号</th>
                    <th>反馈 rad</th>
                    <th>目标 rad</th>
                  </tr>
                </thead>
                <tbody>
                  {sample.value.status.positions.map((value, id) => (
                    <tr>
                      <td>{id + 1}</td>
                      <td>{value.toFixed(4)}</td>
                      <td>{sample.value?.status.targets[id]?.toFixed(4) ?? "—"}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          <h3>对比另一条记录</h3>
          <div class={style.controls}>
            <select aria-label="对比记录" v-model={right.value}>
              <option value="">选择记录</option>
              {records.value
                .filter((record) => record.state !== "recording")
                .map((record) => (
                  <option value={record.id}>{record.name}</option>
                ))}
            </select>
            <label>
              对比片段 <input type="number" min="0" v-model={referenceSegment.value} />
            </label>
            <label>
              对比开始秒 <input type="number" min="0" step="0.02" v-model={referenceStart.value} />
            </label>
            <button
              disabled={busy.value || !right.value}
              onClick={() =>
                void run(async () => {
                  const reference = {
                    id: right.value,
                    segmentId: referenceSegment.value,
                    startS: referenceStart.value,
                    endS: referenceStart.value + end.value - start.value,
                  };
                  comparison.value = await commands.recordingCompare(window(), reference);
                  referenceFrames.value = await commands.recordingFrames(reference);
                })
              }
            >
              比较
            </button>
          </div>
          {comparison.value && (
            <>
              {comparison.value.warnings.map((warning) => (
                <p>{warning}</p>
              ))}
              <p>
                匹配 {comparison.value.matchedFrames} 帧，未匹配 {comparison.value.unmatchedFrames}{" "}
                帧
              </p>
              <table>
                <thead>
                  <tr>
                    <th>字段</th>
                    <th>RMSE</th>
                    <th>最大偏差</th>
                  </tr>
                </thead>
                <tbody>
                  {comparison.value.metrics.map((metric) => (
                    <tr>
                      <td>{metric.field}</td>
                      <td>{(metric.rmse ?? 0).toPrecision(4)}</td>
                      <td>{(metric.maxError ?? 0).toPrecision(4)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
              <p class={style.path}>{comparison.value.path}</p>
            </>
          )}
          <h3>导出教师数据</h3>
          <div class={style.controls}>
            {(["policy", "motion"] as const).map((kind) => (
              <button
                disabled={busy.value}
                onClick={() =>
                  void run(async () => {
                    const exported = await commands.recordingExport(window(), kind);
                    if (!alive) return;
                    dataset.value = kind === "policy" && exported.samples >= 2 ? exported.path : "";
                    result.value = `已导出 ${exported.samples} 个样本，排除 ${exported.excludedFrames} 帧。保存到 ${exported.path}`;
                  })
                }
              >
                {kind === "policy" ? "策略模仿数据" : "动作轨迹数据"}
              </button>
            ))}
          </div>
          {result.value && (
            <p role="status" class={style.path}>
              {result.value}
            </p>
          )}
          {dataset.value && (
            <button
              onClick={() => {
                store.state.teacherDatasetPath = dataset.value;
                store.state.recordingPlayback = null;
                store.state.tab = "training";
              }}
            >
              用于学生训练
            </button>
          )}
        </section>
      )}
      <section>
        <h2>诊断事件</h2>
        <div class={style.controls}>
          <input
            aria-label="事件分类"
            placeholder="分类，如 recording、servo"
            v-model={category.value}
          />
          <select aria-label="日志等级" v-model={level.value}>
            <option value="">全部等级</option>
            {["info", "warn", "error"].map((value) => (
              <option>{value}</option>
            ))}
          </select>
          <input
            aria-label="搜索日志"
            placeholder="搜索 recording ID、鸭子或事件"
            v-model={query.value}
          />
        </div>
        <pre class={style.events}>{shownDiagnostics.value.join("\n") || "暂无匹配的诊断事件"}</pre>
      </section>
      {error.value && (
        <p role="alert" class={style.error}>
          {error.value}
        </p>
      )}
    </div>
  );
});
