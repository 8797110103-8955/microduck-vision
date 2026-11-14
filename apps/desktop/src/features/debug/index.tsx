import { computed, defineComponent, onMounted, ref, watch } from "vue";
import type { ActionRecord, LoopAction, PhysicalMachine, VirtualMachine } from "../../bridge/types";
import { ScrollArea } from "../../components/scroll-area";
import { desktopBackend, useDesktopStore } from "../../store";
import style from "./style.module.scss";

export { actionKind, servoModel, valueOf, SectionHeading } from "./helpers";

const debugServoIds = Array.from({ length: 15 }, (_, index) => index + 1);
const formatParameter = (value: number | null, suffix = "") =>
  value === null ? "—" : `${value.toFixed(3)}${suffix}`;

function unboundVirtualMachines(
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  const boundVirtualMachineIds = new Set(
    physicalMachines.map((machine) => machine.virtualMachineId).filter((id): id is string => !!id),
  );
  return virtualMachines.filter((machine) => !boundVirtualMachineIds.has(machine.id));
}

export function debugPhysicalMachineFor(
  virtualMachineId: string | null,
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  if (!virtualMachineId) return null;
  const explicitlyBound =
    physicalMachines.find(
      (machine) => machine.virtualMachineId === virtualMachineId && machine.connected,
    ) ?? physicalMachines.find((machine) => machine.virtualMachineId === virtualMachineId);
  if (explicitlyBound) return explicitlyBound;

  const unboundMachines = unboundVirtualMachines(virtualMachines, physicalMachines);
  const unboundPhysicalMachines = physicalMachines.filter(
    (machine) => machine.connected && !machine.virtualMachineId,
  );
  const index = unboundMachines.findIndex((machine) => machine.id === virtualMachineId);
  return index >= 0 ? (unboundPhysicalMachines[index] ?? null) : null;
}

export function defaultDebugVirtualMachineId(
  virtualMachines: VirtualMachine[],
  physicalMachines: PhysicalMachine[],
) {
  if (!physicalMachines.some((machine) => machine.connected && !machine.virtualMachineId))
    return null;
  return unboundVirtualMachines(virtualMachines, physicalMachines)[0]?.id ?? null;
}

export const DebugPage = defineComponent(() => {
  const store = useDesktopStore();
  const selectedId = ref(1);
  const targetDraft = ref("");
  const selectedIndex = computed(() => Math.max(0, selectedId.value - 1));
  const selectedPosition = computed(() => store.state.loop?.positions[selectedIndex.value] ?? null);
  const selectedTarget = computed(() => store.state.loop?.targets[selectedIndex.value] ?? null);
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const selectedPhysicalMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const applyDefaultSelection = () => {
    if (store.state.selectedVirtualMachineManual) return;
    const defaultId = defaultDebugVirtualMachineId(
      store.state.virtualMachines,
      store.state.physicalMachines,
    );
    if (defaultId) store.state.selectedVirtualMachine = defaultId;
  };
  onMounted(async () => {
    if (!store.state.catalog.length && !store.state.loading) await load();
    applyDefaultSelection();
  });
  watch(selectedTarget, (value) => {
    targetDraft.value = value == null ? "" : String(value);
  });
  const send = () => sendTargets((store.state.loop?.targets ?? []).map((v) => v + 0.01));
  const move = () => {
    const value = Number(targetDraft.value);
    if (!Number.isFinite(value) || !store.state.loop) return;
    const targets = [...store.state.loop.targets];
    targets[selectedIndex.value] = value;
    void sendTargets(targets);
  };
  const refresh = async () => {
    store.state.loop = await desktopBackend().loopStatus();
  };
  const toggleLoop = () => {
    if (store.state.loop) {
      void closeLoop();
      return;
    }
    void openLoop(
      Array.from({ length: 15 }, (_, index) => index + 1),
      selectedPhysicalMachine.value?.connected ? selectedPhysicalMachine.value.id : undefined,
    );
  };

  return () => (
    <ScrollArea className={[style.page, style.debugPage].filter(Boolean).join(" ")}>
      <div class={style.debugHeader}>
        <span>{selectedMachine.value?.name ?? "机器"}</span>
        {selectedPhysicalMachine.value && <small>· {selectedPhysicalMachine.value.name}</small>}
      </div>
      <div class={style.debugLegend} aria-label="舵机状态图例">
        <span>
          <i class={[style.legendDot, style.online]} /> 在线
        </span>
        <span>
          <i class={[style.legendDot, style.warn]} /> 告警
        </span>
        <span>
          <i class={[style.legendDot, style.missing]} /> 缺失
        </span>
        <span>
          <i class={[style.legendDot, style.unmapped]} /> 未映射
        </span>
      </div>
      <div class={style.debugGrid}>
        <article class={style.panelCard}>
          <div class={style.cardTopline}>
            <h2>ID {selectedId.value}</h2>
            <span class={[style.phase, style[store.state.loop?.phase ?? "limp"]]}>
              {store.state.loop?.phase ?? "未启动"}
            </span>
          </div>
          <div class={style.servoIdList} aria-label="舵机编号">
            {debugServoIds.map((id) => (
              <button
                class={selectedId.value === id ? style.active : undefined}
                type="button"
                onClick={() => (selectedId.value = id)}
              >
                {id}
              </button>
            ))}
          </div>
          <div class={style.parameterGrid} aria-label="舵机状态">
            <div>
              <span>位置</span>
              <b>{formatParameter(selectedPosition.value, " rad")}</b>
            </div>
            <div>
              <span>速度</span>
              <b>—</b>
            </div>
            <div>
              <span>负载</span>
              <b>—</b>
            </div>
            <div>
              <span>电压</span>
              <b>—</b>
            </div>
            <div>
              <span>温度</span>
              <b>—</b>
            </div>
            <div>
              <span>电流</span>
              <b>—</b>
            </div>
          </div>
          <div class={style.controlGroup}>
            <label class={style.field}>
              目标位置
              <div class={style.targetRow}>
                <input
                  type="range"
                  min="-3.142"
                  max="3.142"
                  step="0.001"
                  value={targetDraft.value}
                  disabled={!store.state.loop}
                  onInput={(event: Event) => {
                    if (event.target instanceof HTMLInputElement)
                      targetDraft.value = event.target.value;
                  }}
                />
                <input
                  class={style.numberInput}
                  type="number"
                  step="0.001"
                  value={targetDraft.value}
                  disabled={!store.state.loop}
                  onChange={(event: Event) => {
                    if (event.target instanceof HTMLInputElement)
                      targetDraft.value = event.target.value;
                  }}
                />
              </div>
            </label>
            <div class={style.actions}>
              <button class={style.primary} disabled={!store.state.loop} onClick={move}>
                移动
              </button>
              <button
                disabled={!store.state.loop}
                onClick={() => void sendAction({ kind: "power", on: true })}
              >
                力矩开
              </button>
              <button
                disabled={!store.state.loop}
                onClick={() => void sendAction({ kind: "power", on: false })}
              >
                力矩关
              </button>
              <button disabled={!store.state.loop} onClick={() => void refresh()}>
                刷新状态
              </button>
            </div>
            <div class={style.actions}>
              <button disabled={!store.state.loop} onClick={() => void send()}>
                发送微调目标
              </button>
              <button disabled={!store.state.loop} onClick={() => void tick()}>
                推进一拍
              </button>
              <button class={store.state.loop ? style.danger : style.primary} onClick={toggleLoop}>
                {store.state.loop ? "停止控制环" : "启动控制环"}
              </button>
            </div>
          </div>
          <p class={style.muted}>速度、加速度和实时 telemetry 会在后端提供单舵机读数后恢复。</p>
        </article>
        <article class={style.panelCard}>
          <div class={style.cardTopline}>
            <h2>安全状态</h2>
            <span>{store.state.loop?.torque ? "力矩开启" : "力矩关闭"}</span>
          </div>
          <ul class={style.diagnosticList}>
            <li>
              <span>控制环 session</span>
              <b>{store.state.loop?.session ?? "—"}</b>
            </li>
            <li>
              <span>目标数量</span>
              <b>{store.state.loop?.targets.length ?? 0}</b>
            </li>
            <li>
              <span>传感器连续失败</span>
              <b>{store.state.loop?.sensorFailures ?? 0}</b>
            </li>
            <li>
              <span>最后错误</span>
              <b>{store.state.loop?.lastError ?? "无"}</b>
            </li>
          </ul>
          <pre class={style.statusJson}>{JSON.stringify(store.state.loop, null, 2)}</pre>
        </article>
      </div>
    </ScrollArea>
  );
});

export async function load() {
  const store = useDesktopStore();
  store.state.loading = true;
  store.state.error = null;
  try {
    [
      store.state.catalog,
      store.state.virtualMachines,
      store.state.physicalMachines,
      store.state.actions,
      store.state.autoReconnectEnabled,
      store.state.policies,
    ] = await Promise.all([
      desktopBackend().servoCatalog(),
      desktopBackend().virtualMachines(),
      desktopBackend().physicalMachines(),
      desktopBackend().actions(),
      desktopBackend().autoReconnectEnabled(),
      desktopBackend().policyList(),
    ]);
    store.state.selectedPolicyId =
      store.state.policies.find((policy) => policy.selected)?.id ?? null;
    store.state.selectedVirtualMachine =
      store.state.virtualMachines.find((machine) => machine.startupSelected)?.id ?? null;
    store.state.selectedVirtualMachineManual = false;
  } catch (error) {
    store.state.error = error instanceof Error ? error.message : String(error);
  } finally {
    store.state.loading = false;
  }
}

export async function createAction(name: string, kind: ActionRecord["kind"] = "pose") {
  const store = useDesktopStore();
  const action = await desktopBackend().createAction({ name, kind });
  store.state.actions.push(action);
  return action;
}

export async function deleteAction(id: string) {
  const store = useDesktopStore();
  await desktopBackend().deleteAction(id);
  store.state.actions = store.state.actions.filter((action) => action.id !== id);
}

export async function openLoop(servoIds: number[], physicalMachineId?: string, policyId?: string) {
  const store = useDesktopStore();
  if (!store.state.selectedVirtualMachine) throw new Error("请选择模拟机");
  store.state.loop = await desktopBackend().loopOpen({
    virtualMachineId: store.state.selectedVirtualMachine,
    servoIds,
    physicalMachineId,
    policyId,
  });
  return store.state.loop;
}

export async function sendAction(action: LoopAction) {
  const store = useDesktopStore();
  store.state.loop = await desktopBackend().loopAction(action);
  return store.state.loop;
}

export async function sendTargets(targets: number[]) {
  const store = useDesktopStore();
  await desktopBackend().loopCommand(targets);
  store.state.loop = await desktopBackend().loopStatus();
}

export async function tick() {
  const store = useDesktopStore();
  store.state.loop = await desktopBackend().loopTick();
  return store.state.loop;
}

export async function closeLoop() {
  const store = useDesktopStore();
  await desktopBackend().loopClose();
  store.state.loop = null;
}

export async function officialSceneModel() {
  return desktopBackend().officialSceneModel();
}

export async function officialSceneFrame(positions: number[]) {
  return desktopBackend().officialSceneFrame(positions);
}
