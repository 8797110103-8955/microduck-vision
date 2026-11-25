import { computed, defineComponent, onScopeDispose, ref, watch } from "vue";
import { useIntervalFn } from "@vueuse/core";
import * as commands from "../../../generated/types";
import { Select } from "../../components/select";
import style from "./style.module.scss";

const timeToleranceS = 1e-8;

const statusLabels = {
  queued: "等待中",
  running: "转换中",
  succeeded: "已完成",
  failed: "失败",
  cancelled: "已取消",
  interrupted: "已中断",
};
const stageLabels = {
  reading: "读取录制",
  validating: "校验数据与环境",
  converting: "转换动作",
  saving: "保存动作包",
};

function valueOf(event: Event) {
  const target = event.currentTarget;
  return target instanceof HTMLInputElement || target instanceof HTMLSelectElement
    ? target.value
    : "";
}

function message(reason: unknown) {
  return reason instanceof Error ? reason.message : String(reason);
}

export const DeepMimicConversion = defineComponent({
  props: {
    recordingId: { type: String, required: true },
    recordingName: { type: String, required: true },
  },
  setup(props) {
    const report = ref<Awaited<ReturnType<typeof commands.deepmimicIntervals>> | null>(null);
    const jobs = ref<Awaited<ReturnType<typeof commands.deepmimicJobs>>>([]);
    const bundles = ref<Awaited<ReturnType<typeof commands.deepmimicBundles>>>([]);
    const intervalId = ref("");
    const name = ref("");
    const start = ref("0");
    const end = ref("");
    const playback = ref<Parameters<typeof commands.deepmimicConvert>[0]["playback"]>("finite");
    const loading = ref(false);
    const busy = ref(false);
    const polling = ref(false);
    const error = ref("");
    let epoch = 0;
    let disposed = false;
    const current = (token: number) => !disposed && token === epoch;
    const active = computed(() =>
      jobs.value.filter((job) => job.status === "queued" || job.status === "running"),
    );
    const interval = computed(() =>
      report.value?.intervals.find((item) => item.id === intervalId.value),
    );
    const duration = computed(() => {
      const selected = interval.value;
      return selected?.first_simulation_s != null && selected.last_simulation_s != null
        ? selected.last_simulation_s - selected.first_simulation_s
        : 0;
    });
    const locked = computed(() => loading.value || busy.value || active.value.length > 0);

    const selectInterval = (id: string) => {
      intervalId.value = id;
      start.value = "0";
      end.value = String(Number(duration.value.toFixed(8)));
    };

    const load = async () => {
      const token = ++epoch;
      loading.value = true;
      error.value = "";
      const results = await Promise.allSettled([
        commands.deepmimicIntervals(props.recordingId),
        commands.deepmimicJobs(props.recordingId),
        commands.deepmimicBundles(props.recordingId),
      ]);
      if (!current(token)) return;
      const [scan, history, published] = results;
      if (scan.status === "fulfilled") {
        report.value = scan.value;
        selectInterval(scan.value.intervals.find((item) => item.frames >= 3)?.id ?? "");
      }
      if (history.status === "fulfilled") jobs.value = history.value;
      if (published.status === "fulfilled") bundles.value = published.value;
      error.value = results
        .filter((result) => result.status === "rejected")
        .map((result) => message(result.reason))
        .join("；");
      loading.value = false;
    };

    const refresh = async () => {
      if (loading.value || busy.value || polling.value || active.value.length === 0) return;
      const token = epoch;
      const ids = active.value.map((job) => job.id);
      polling.value = true;
      try {
        const updates = await Promise.all(ids.map((id) => commands.deepmimicStatus(id)));
        const published = await commands.deepmimicBundles(props.recordingId);
        if (current(token)) {
          jobs.value = jobs.value.map((job) => updates.find((value) => value.id === job.id) ?? job);
          bundles.value = published;
        }
      } catch (reason) {
        if (current(token)) error.value = message(reason);
      } finally {
        if (current(token)) polling.value = false;
      }
    };

    const saveJob = (job: (typeof jobs.value)[number]) => {
      jobs.value = [job, ...jobs.value.filter((item) => item.id !== job.id)];
    };

    const submit = async (event: Event) => {
      event.preventDefault();
      if (locked.value) return;
      const selected = interval.value;
      const from = Number(start.value);
      const to = Number(end.value);
      error.value = "";
      if (
        !selected ||
        report.value?.read_failure ||
        !name.value.trim() ||
        !start.value.trim() ||
        !end.value.trim() ||
        !Number.isFinite(from) ||
        !Number.isFinite(to) ||
        from < 0 ||
        to > duration.value + timeToleranceS ||
        to <= from ||
        Math.floor((to + timeToleranceS) / (selected.control_s ?? 0.02)) -
          Math.ceil((from - timeToleranceS) / (selected.control_s ?? 0.02)) <
          2
      ) {
        error.value = "请选择连续区间，填写名称，并保留至少 3 帧的有效时间范围。";
        return;
      }
      const token = ++epoch;
      polling.value = false;
      busy.value = true;
      try {
        const job = await commands.deepmimicConvert({
          recording_id: props.recordingId,
          interval_id: selected.id,
          start_s: from,
          end_s: to,
          name: name.value.trim(),
          playback: playback.value,
        });
        if (current(token)) saveJob(job);
      } catch (reason) {
        if (current(token)) error.value = message(reason);
      } finally {
        if (current(token)) busy.value = false;
      }
    };

    const cancel = async (id: string) => {
      if (busy.value) return;
      const token = ++epoch;
      polling.value = false;
      busy.value = true;
      error.value = "";
      try {
        const job = await commands.deepmimicCancel(id);
        if (current(token)) saveJob(job);
        const published = await commands.deepmimicBundles(props.recordingId);
        if (current(token)) bundles.value = published;
      } catch (reason) {
        if (current(token)) error.value = message(reason);
      } finally {
        if (current(token)) busy.value = false;
      }
    };

    const openBundle = async (id: string) => {
      const token = epoch;
      try {
        await commands.deepmimicOpenBundle(id);
      } catch (reason) {
        if (current(token)) error.value = message(reason);
      }
    };

    watch(
      () => props.recordingId,
      () => {
        report.value = null;
        jobs.value = [];
        bundles.value = [];
        busy.value = false;
        polling.value = false;
        name.value = `${props.recordingName} DeepMimic`;
        playback.value = "finite";
        void load();
      },
      { immediate: true },
    );
    useIntervalFn(() => void refresh(), 1000);
    onScopeDispose(() => {
      disposed = true;
      ++epoch;
    });

    return () => (
      <section class={style.conversion} aria-label="DeepMimic 转换配置">
        <h2>DeepMimic 转换配置</h2>
        {loading.value && <p role="status">正在读取连续区间与转换记录…</p>}
        {error.value && (
          <p role="alert" class={style.error}>
            {error.value}
          </p>
        )}
        {report.value?.read_failure && (
          <p role="alert" class={style.error}>
            录制读取失败，最后可靠帧：{report.value.read_failure.last_reliable_frame ?? "无"}。
            {report.value.read_failure.reason}
          </p>
        )}
        {!loading.value && !interval.value && <p>没有至少包含 3 帧的可转换连续区间。</p>}
        <form onSubmit={(event) => void submit(event)}>
          <fieldset disabled={locked.value} class={style.conversionFields}>
            <label>
              名称
              <input
                value={name.value}
                required
                onInput={(e) => {
                  name.value = valueOf(e);
                }}
              />
            </label>
            <label>
              连续区间
              <Select value={intervalId.value} onChange={(e: Event) => selectInterval(valueOf(e))}>
                {report.value?.intervals.map((item) => (
                  <option value={item.id} disabled={item.frames < 3}>
                    帧 {item.first_frame}–{item.last_frame} · {item.frames} 帧
                  </option>
                ))}
              </Select>
            </label>
            <label>
              起始时间（秒）
              <input
                type="number"
                min="0"
                max={duration.value + timeToleranceS}
                step="any"
                required
                value={start.value}
                onInput={(e) => {
                  start.value = valueOf(e);
                }}
              />
            </label>
            <label>
              结束时间（秒）
              <input
                type="number"
                min="0"
                max={duration.value + timeToleranceS}
                step="any"
                required
                value={end.value}
                onInput={(e) => {
                  end.value = valueOf(e);
                }}
              />
            </label>
            <label>
              播放方式
              <Select
                value={playback.value}
                onChange={(e: Event) => {
                  playback.value = valueOf(e) === "cyclic" ? "cyclic" : "finite";
                }}
              >
                <option value="finite">有限（finite）</option>
                <option value="cyclic">循环（cyclic，需通过接缝校验）</option>
              </Select>
            </label>
            {interval.value && (
              <p class={style.sourceInfo}>
                源身体：{interval.value.body_id} · {interval.value.model_kind} · 控制周期{" "}
                {(interval.value.control_s ?? 0) * 1000} ms。 时间相对于所选区间首帧，范围 0–
                {Number(duration.value.toFixed(8))} 秒。
              </p>
            )}
            <button type="submit" disabled={!interval.value || Boolean(report.value?.read_failure)}>
              开始 DeepMimic 转换
            </button>
          </fieldset>
        </form>
        <h3>DeepMimic 转换结果</h3>
        <ul class={style.conversionResults}>
          {jobs.value.map((job) => (
            <li key={job.id}>
              <span>
                {job.request.name} · {statusLabels[job.status]} · {stageLabels[job.stage]} ·{" "}
                {job.processed_frames}
                {job.total_frames !== null && `/${job.total_frames}`} 帧
              </span>
              {(job.status === "queued" || job.status === "running") && (
                <button type="button" disabled={busy.value} onClick={() => void cancel(job.id)}>
                  取消转换
                </button>
              )}
              {job.error && <p class={style.error}>{job.error}</p>}
            </li>
          ))}
          {bundles.value.map((bundle) => (
            <li key={bundle.id}>
              <span>
                {bundle.name} · {bundle.body_id} · {bundle.frames} 帧 ·{" "}
                {bundle.duration_s?.toFixed(2)} 秒 · {bundle.playback}
              </span>
              <button type="button" onClick={() => void openBundle(bundle.id)}>
                打开动作包
              </button>
            </li>
          ))}
        </ul>
        {!loading.value && jobs.value.length === 0 && bundles.value.length === 0 && (
          <p>暂无转换结果</p>
        )}
      </section>
    );
  },
});
