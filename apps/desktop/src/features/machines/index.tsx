import { defineComponent, onMounted, ref } from "vue";
import type {
  DesktopBackend,
  PhysicalMachineDetectInput,
  PhysicalMachineProbeInput,
  PhysicalMachineReconnectInput,
  PhysicalMachineSaveInput,
} from "../../bridge/types";
import { desktopBackend, useDesktopStore } from "../../store";
import { valueOf } from "../debug/helpers";
import style from "./style.module.scss";

export const PhysicalMachinesPage = defineComponent(() => {
  const store = useDesktopStore();
  const collapsed = ref<string[]>([]);
  const names = ref<Record<string, string>>({});
  const port = ref("");
  const baud = ref("1000000");
  const detecting = ref(false);

  const canSaveDetected = () => {
    const candidate = store.state.detectedPhysicalMachine;
    return !!candidate?.md5 && candidate.matchResult === "new";
  };

  const saveDetectedMachine = () => {
    if (!canSaveDetected()) return;
    const candidate = store.state.detectedPhysicalMachine!;
    void savePhysicalMachine({
      census: candidate.census,
      reconnectHint: candidate.reconnectHint,
      name: null,
    });
  };

  const isExpanded = (id: string) => !collapsed.value.includes(id);
  const toggle = (id: string) => {
    collapsed.value = collapsed.value.includes(id)
      ? collapsed.value.filter((value) => value !== id)
      : [...collapsed.value, id];
  };
  const machineName = (id: string, fallback: string) => names.value[id] ?? fallback;
  const rename = async (id: string, fallback: string) => {
    const name = machineName(id, fallback).trim();
    if (!name || name === fallback) {
      names.value[id] = fallback;
      return;
    }
    await renamePhysicalMachine(id, name);
    delete names.value[id];
  };
  const detect = async () => {
    detecting.value = true;
    try {
      const candidate = store.state.physicalPorts.find((entry) => entry.portName === port.value);
      await probePhysicalMachine({
        portName: port.value,
        baud: Number(baud.value),
        reconnectHint: candidate
          ? {
              portName: candidate.portName,
              vid: candidate.vid,
              pid: candidate.pid,
              adapterSerial: candidate.adapterSerial,
              product: candidate.product,
            }
          : null,
      });
    } finally {
      detecting.value = false;
    }
  };
  onMounted(() => {
    void refreshPhysicalPorts();
  });

  return () => (
    <div class={[style.page, style.physicalMachinesPage]}>
      <section class={[style.physicalConnectionBar, style.panelCard]} aria-label="舵机连接">
        <label>
          USB/串口
          <select
            value={port.value}
            onChange={(event: Event) => (port.value = valueOf(event))}
            disabled={detecting.value}
          >
            <option value="">选择端口</option>
            {store.state.physicalPorts.map((candidate) => (
              <option value={candidate.portName}>
                {candidate.portName}
                {candidate.product ? ` · ${candidate.product}` : ""}
              </option>
            ))}
          </select>
        </label>
        <button
          class={style.smallButton}
          type="button"
          disabled={detecting.value}
          onClick={() => void refreshPhysicalPorts()}
        >
          刷新
        </button>
        <label>
          波特率
          <select
            value={baud.value}
            onChange={(event: Event) => (baud.value = valueOf(event))}
            disabled={detecting.value}
          >
            {(store.state.physicalBaudChoices.length
              ? store.state.physicalBaudChoices
              : [1000000, 57600]
            ).map((value) => (
              <option value={String(value)}>{value.toLocaleString()}</option>
            ))}
          </select>
        </label>
        <button
          class={style.smallButton}
          type="button"
          disabled={!port.value || detecting.value}
          onClick={() => void detect()}
        >
          {detecting.value ? "检测中…" : "连接/检测"}
        </button>
        <span class={style.tag}>
          {detecting.value ? "正在检测" : port.value ? "未连接" : "未选择端口"}
        </span>
        <label class={style.physicalAutoReconnect}>
          自动重连{" "}
          <input
            type="checkbox"
            checked={store.state.autoReconnectEnabled}
            onChange={(event: Event) => {
              if (event.target instanceof HTMLInputElement)
                void setAutoReconnectEnabled(event.target.checked);
            }}
          />
        </label>
      </section>
      {store.state.detectedPhysicalMachine && (
        <section
          class={[style.physicalDetected, style.panelCard, canSaveDetected() && style.clickable]}
          aria-label="检测候选"
          role={canSaveDetected() ? "button" : "presentation"}
          tabindex={canSaveDetected() ? "0" : "-1"}
          onClick={saveDetectedMachine}
          onKeydown={(event: KeyboardEvent) => {
            if (event.key === "Enter" || event.key === " ") saveDetectedMachine();
          }}
        >
          <div class={style.physicalDetectedHeader}>
            <div class={style.physicalDetectedTitle}>
              <strong>检测候选</strong>
              <span class={style.tag}>
                {store.state.detectedPhysicalMachine.md5 ?? "无有效 MD5"}
              </span>
            </div>
            {canSaveDetected() && (
              <span class={style.candidateAdd} aria-hidden="true">
                ＋
              </span>
            )}
          </div>
          <small class={style.physicalDetectedStatus}>
            {store.state.detectedPhysicalMachine.diagnostics.join("、") || "完整 census"}
          </small>
          <div class={style.physicalDetectedCensus}>
            <span>电机 ID：</span>
            <code>
              {store.state.detectedPhysicalMachine.census.length
                ? store.state.detectedPhysicalMachine.census
                    .map((servo) => `ID ${servo.servoId}`)
                    .join(" · ")
                : "—"}
            </code>
          </div>
        </section>
      )}
      <div class={style.physicalMachineList}>
        {store.state.physicalMachines.length === 0 ? (
          <div class={[style.panelCard, style.emptyState]}>未检测到实机连接 &amp; 无历史缓存</div>
        ) : (
          store.state.physicalMachines.map((machine) => (
            <article class={[style.physicalMachineRow, machine.connected && style.connected]}>
              <div class={style.physicalMachineSummary}>
                <button
                  class={[style.physicalExpand, isExpanded(machine.id) && style.expanded]}
                  type="button"
                  aria-label={isExpanded(machine.id) ? "收起实机详情" : "展开实机详情"}
                  onClick={() => toggle(machine.id)}
                >
                  <span class={style.physicalExpandIcon} aria-hidden="true" />
                </button>
                <label class={style.physicalNameField}>
                  <span>Name：</span>
                  <input
                    value={machineName(machine.id, machine.name)}
                    onInput={(event: Event) => {
                      names.value[machine.id] = valueOf(event);
                    }}
                    onBlur={() => void rename(machine.id, machine.name)}
                    onKeydown={(event: KeyboardEvent) => {
                      const target = event.target;
                      if (!(target instanceof HTMLInputElement)) return;
                      if (event.key === "Enter") target.blur();
                      if (event.key === "Escape") {
                        names.value[machine.id] = machine.name;
                        target.blur();
                      }
                    }}
                  />
                </label>
                <div class={style.physicalFact}>
                  <span>硬件码：</span>
                  <code>{machine.machineCode || "—"}</code>
                </div>
                <label class={[style.physicalFact, style.physicalVirtualMachine]}>
                  <span>模拟机：</span>
                  <select
                    value={machine.virtualMachineId ?? ""}
                    onChange={(event: Event) => {
                      const id = valueOf(event);
                      if (id) void mountPhysicalMachine(machine.id, id);
                    }}
                  >
                    <option value="">未绑定</option>
                    {store.state.virtualMachines.map((vm) => (
                      <option value={vm.id}>{vm.name}</option>
                    ))}
                  </select>
                </label>
                <span class={machine.connected ? [style.tag, style.ok] : style.tag}>
                  {machine.connected ? "已连接" : "未连接"}
                </span>
              </div>
              {isExpanded(machine.id) && (
                <div class={style.physicalMachineDetails}>
                  <div class={style.physicalMachineActions}>
                    {!machine.connected && (
                      <button
                        class={style.smallButton}
                        type="button"
                        onClick={() =>
                          void (store.state.autoReconnectEnabled
                            ? autoReconnectPhysicalMachine(machine.id, Number(baud.value))
                            : connectPhysicalMachine(machine.id))
                        }
                      >
                        {store.state.autoReconnectEnabled ? "自动重连" : "连接"}
                      </button>
                    )}
                    <button
                      class={style.smallButton}
                      type="button"
                      onClick={() => void scanPhysicalMachine(machine.id)}
                      disabled={!machine.connected}
                    >
                      扫描
                    </button>
                    <span class={style.tag}>在线 {machine.onlineServoIds.length}</span>
                    <button
                      class={[style.smallButton, style.danger, style.physicalDelete]}
                      type="button"
                      onClick={() => void deletePhysicalMachine(machine.id)}
                    >
                      删除
                    </button>
                  </div>
                  <div class={style.physicalCensusLine}>
                    <span>Census：</span>
                    <code>
                      {machine.census?.length
                        ? machine.census
                            .map((servo) => `${servo.servoId}:${servo.model}`)
                            .join(" · ")
                        : "—"}
                    </code>
                  </div>
                </div>
              )}
            </article>
          ))
        )}
      </div>
    </div>
  );
});

const replaceMachine = (machine: Awaited<ReturnType<DesktopBackend["connectPhysicalMachine"]>>) => {
  const store = useDesktopStore();
  const index = store.state.physicalMachines.findIndex((entry) => entry.id === machine.id);
  if (index >= 0) store.state.physicalMachines[index] = machine;
  return machine;
};

export async function createPhysicalMachine(name: string, usbPort: string, machineCode: string) {
  const store = useDesktopStore();
  const machine = await desktopBackend().createPhysicalMachine({ name, usbPort, machineCode });
  store.state.physicalMachines.push(machine);
  return machine;
}
export async function detectPhysicalMachine(input: PhysicalMachineDetectInput) {
  const store = useDesktopStore();
  store.state.detectedPhysicalMachine = await desktopBackend().detectPhysicalMachine(input);
  return store.state.detectedPhysicalMachine;
}
export async function probePhysicalMachine(input: PhysicalMachineProbeInput) {
  const store = useDesktopStore();
  store.state.detectedPhysicalMachine = await desktopBackend().probePhysicalMachine(input);
  return store.state.detectedPhysicalMachine;
}
export async function refreshPhysicalPorts() {
  const store = useDesktopStore();
  [store.state.physicalPorts, store.state.physicalBaudChoices] = await Promise.all([
    desktopBackend().physicalPorts(),
    desktopBackend().physicalBaudChoices(),
  ]);
  return store.state.physicalPorts;
}
export async function setAutoReconnectEnabled(enabled: boolean) {
  const store = useDesktopStore();
  store.state.autoReconnectEnabled = await desktopBackend().setAutoReconnectEnabled(enabled);
  return store.state.autoReconnectEnabled;
}
export async function savePhysicalMachine(input: PhysicalMachineSaveInput) {
  const store = useDesktopStore();
  const machine = await desktopBackend().savePhysicalMachine(input);
  if (!store.state.physicalMachines.some((entry) => entry.id === machine.id))
    store.state.physicalMachines.push(machine);
  store.state.detectedPhysicalMachine = null;
  return machine;
}
export async function disconnectPhysicalMachine(id: string) {
  return replaceMachine(await desktopBackend().disconnectPhysicalMachine(id));
}
export async function reconnectPhysicalMachine(input: PhysicalMachineReconnectInput) {
  return replaceMachine(await desktopBackend().reconnectPhysicalMachine(input));
}
export async function autoReconnectPhysicalMachine(id: string, baud: number) {
  return replaceMachine(await desktopBackend().autoReconnectPhysicalMachine(id, baud));
}
export async function connectPhysicalMachine(id: string) {
  return replaceMachine(await desktopBackend().connectPhysicalMachine(id));
}
export async function scanPhysicalMachine(id: string) {
  return replaceMachine(await desktopBackend().scanPhysicalMachine(id));
}
export async function mountPhysicalMachine(id: string, virtualMachineId: string) {
  const store = useDesktopStore();
  const machine = await desktopBackend().mountPhysicalMachine(id, virtualMachineId);
  replaceMachine(machine);
  const virtualMachine = store.state.virtualMachines.find((entry) => entry.id === virtualMachineId);
  if (virtualMachine && !virtualMachine.referencedBy.includes(id))
    virtualMachine.referencedBy.push(id);
  return machine;
}
export async function deletePhysicalMachine(id: string) {
  const store = useDesktopStore();
  await desktopBackend().deletePhysicalMachine(id);
  store.state.physicalMachines = store.state.physicalMachines.filter(
    (machine) => machine.id !== id,
  );
  store.state.virtualMachines.forEach((machine) => {
    machine.referencedBy = machine.referencedBy.filter((reference) => reference !== id);
  });
}
export async function renamePhysicalMachine(id: string, name: string) {
  return replaceMachine(await desktopBackend().renamePhysicalMachine(id, name));
}
