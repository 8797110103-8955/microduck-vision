import { defineComponent, onMounted, ref, shallowRef, watch } from "vue";
import type {
  DesktopBackend,
  PhysicalMachineDetectInput,
  PhysicalMachineProbeInput,
  PhysicalMachineReconnectInput,
  PhysicalMachineSaveInput,
  PhysicalServoRegister,
} from "../../bridge/types";
import { Slider } from "../../components/slider";
import { desktopBackend, useDesktopStore } from "../../store";
import style from "./style.module.scss";

const registerRowKey = (id: string, row: PhysicalServoRegister) =>
  `${id}:${row.servoId}:${row.address}`;

const registerTranslations: Record<string, string> = {
  firmware_major_version: "固件主版本号",
  firmware_minor_version: "固件次版本号",
  model: "舵机主版本号",
  model_number: "舵机次版本号",
  id: "舵机 ID",
  baudrate: "波特率",
  baud_rate: "波特率",
  return_delay_time: "保留地址",
  response_status_level: "应答状态级别",
  status_return_level: "应答状态级别",
  min_angle_limit: "最小角度限制",
  min_position_limit: "最小角度限制",
  max_angle_limit: "最大角度限制",
  max_position_limit: "最大角度限制",
  max_temperature_limit: "最高温度上限",
  temperature_limit: "最高温度上限",
  max_voltage_limit: "最高输入电压",
  min_voltage_limit: "最低输入电压",
  max_torque_limit: "最大扭矩",
  phase: "相位",
  unloading_condition: "卸载条件",
  led_alarm_condition: "LED 报警条件",
  p_coefficient: "P 比例系数",
  d_coefficient: "D 微分系数",
  i_coefficient: "I 积分系数",
  minimum_startup_force: "最小启动力",
  integral_limit: "积分限制",
  cw_dead_zone: "顺时针不灵敏区",
  ccw_dead_zone: "逆时针不灵敏区",
  protection_current: "保护电流",
  angular_resolution: "角度分辨率",
  offset: "位置偏移",
  homing_offset: "位置偏移",
  mode: "运行模式",
  operating_mode: "运行模式",
  protective_torque: "保护扭矩",
  protection_time: "保护时间",
  overload_torque: "过载扭矩",
  speed_closed_loop_p_coefficient: "速度闭环 P 比例系数",
  over_current_protection_time: "过流保护时间",
  velocity_closed_loop_i_coefficient: "速度闭环 I 积分系数",
  torque_enable: "扭矩开关",
  acceleration: "加速度",
  acceleration_limit: "加速度",
  goal_position: "目标位置",
  goal_time: "运行时间",
  pwm_open_speed: "PWM 开环速度",
  goal_speed: "运行速度",
  moving_speed: "运行速度",
  torque_limit: "转矩限制",
  lock: "锁标志",
  present_position: "当前位置",
  present_speed: "当前速度",
  present_velocity: "当前速度",
  present_load: "当前负载",
  present_voltage: "当前电压",
  present_input_voltage: "当前电压",
  present_temperature: "当前温度",
  status: "舵机状态",
  moving: "移动标志",
  moving_status: "移动标志",
  present_current: "当前电流",
  current_bias: "电流偏置",
  v_fk: "vFk（×10）",
  v_kgi: "vKgI",
  p_fk: "pFk（×10）",
  e_fk: "eFk（×10）",
  vk: "Vk（毫秒）",
  model_information: "型号信息",
  firmware_version: "固件版本号",
  drive_mode: "驱动模式",
  secondary_id: "次级 ID",
  protocol_type: "协议类型",
  moving_threshold: "移动阈值",
  moving_velocity_threshold: "移动速度阈值",
  dts: "DTS（毫秒）",
  velocity_unit_factor: "速度单位系数",
  hts: "最小速度限制",
  maximum_velocity_limit: "最大速度限制",
  maximum_acceleration: "最大加速度",
  acceleration_multiplier: "加速度倍数",
  pwm_limit: "PWM 限制",
  current_limit: "电流限制",
  velocity_limit: "速度限制",
  startup_configuration: "启动配置",
  pwm_slope: "PWM 斜率",
  shutdown: "关机条件",
  led: "LED",
  registered_instruction: "注册指令",
  hardware_error_status: "硬件错误状态",
  velocity_i_gain: "速度 I 增益",
  velocity_p_gain: "速度 P 增益",
  position_d_gain: "位置 D 增益",
  position_i_gain: "位置 I 增益",
  position_p_gain: "位置 P 增益",
  feedforward_2nd_gain: "前馈二阶增益",
  feedforward_1st_gain: "前馈一阶增益",
  bus_watchdog: "总线看门狗",
  goal_pwm: "目标 PWM",
  goal_current: "目标电流",
  goal_velocity: "目标速度",
  profile_acceleration: "轨迹加速度",
  profile_velocity: "轨迹速度",
  realtime_tick: "实时计时",
  present_pwm: "当前 PWM",
  velocity_trajectory: "速度轨迹",
  position_trajectory: "位置轨迹",
  backup_ready: "备份状态",
  indirect_address_1: "间接地址 1",
  indirect_address_2: "间接地址 2",
  indirect_address_3: "间接地址 3",
  indirect_address_4: "间接地址 4",
  indirect_address_5: "间接地址 5",
  indirect_address_6: "间接地址 6",
  indirect_data_1: "间接数据 1",
  indirect_data_2: "间接数据 2",
  indirect_data_3: "间接数据 3",
  indirect_data_4: "间接数据 4",
  indirect_data_5: "间接数据 5",
  indirect_data_6: "间接数据 6",
};

const registerLabel = (name: string) => {
  const translation = registerTranslations[name];
  return translation ? `${name}（${translation}）` : name;
};

const registerAddress = (address: number) =>
  `0x${address.toString(16).padStart(2, "0").toUpperCase()}`;

export const PhysicalMachinesPage = defineComponent(() => {
  const store = useDesktopStore();
  const collapsed = ref<string[]>([]);
  const names = ref<Record<string, string>>({});
  const port = ref("");
  const baud = ref("1000000");
  const detecting = ref(false);
  const registerRows = ref<Record<string, PhysicalServoRegister[]>>({});
  const registerLoading = ref<Record<string, boolean>>({});
  const registerDrafts = ref<Record<string, string>>({});
  const registerErrors = ref<Record<string, string>>({});
  const selectedServoIds = ref<Record<string, number>>({});
  const portSelect = shallowRef<HTMLSelectElement>();
  const baudSelect = shallowRef<HTMLSelectElement>();
  const autoReconnectInput = shallowRef<HTMLInputElement>();
  const nameInputs = new Map<string, HTMLInputElement>();
  const virtualMachineSelects = new Map<string, HTMLSelectElement>();

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
  const loadRegisters = async (id: string) => {
    if (registerLoading.value[id]) return;
    registerLoading.value[id] = true;
    delete registerErrors.value[id];
    try {
      registerRows.value[id] = await desktopBackend().physicalMachineRegisters(id);
    } catch (error) {
      registerErrors.value[id] = error instanceof Error ? error.message : String(error);
    } finally {
      registerLoading.value[id] = false;
    }
  };
  const toggle = (id: string) => {
    const expanding = collapsed.value.includes(id);
    collapsed.value = expanding
      ? collapsed.value.filter((value) => value !== id)
      : [...collapsed.value, id];
    if (expanding) void loadRegisters(id);
  };
  const writeRegister = async (machineId: string, row: PhysicalServoRegister) => {
    const key = registerRowKey(machineId, row);
    const value = Number(registerDrafts.value[key] ?? row.value);
    if (!Number.isFinite(value)) return;
    try {
      const updated = await desktopBackend().writePhysicalMachineRegister({
        physicalMachineId: machineId,
        servoId: row.servoId,
        name: row.name,
        value,
      });
      if (row.address === 5) {
        // Changing a servo ID changes the machine census and any virtual
        // machine binding that points at that ID. Reload both caches and the
        // register rows so no stale row key survives the rename.
        [store.state.physicalMachines, store.state.virtualMachines] = await Promise.all([
          desktopBackend().physicalMachines(),
          desktopBackend().virtualMachines(),
        ]);
        registerRows.value[machineId] = await desktopBackend().physicalMachineRegisters(machineId);
      } else {
        registerRows.value[machineId] = (registerRows.value[machineId] ?? []).map((entry) =>
          registerRowKey(machineId, entry) === key ? updated : entry,
        );
      }
      delete registerDrafts.value[key];
    } catch (error) {
      registerErrors.value[machineId] = error instanceof Error ? error.message : String(error);
    }
  };
  const machineName = (id: string, fallback: string) => names.value[id] ?? fallback;
  const activeServoIdForMachine = (machine: (typeof store.state.physicalMachines)[number]) => {
    const ids = (machine.census ?? []).map((servo) => servo.servoId);
    const selected = selectedServoIds.value[machine.id];
    return selected !== undefined && ids.includes(selected) ? selected : ids[0];
  };
  const servoModelLabel = (machine: (typeof store.state.physicalMachines)[number]) => {
    const model = machine.census?.find(
      (servo) => servo.servoId === activeServoIdForMachine(machine),
    )?.model;
    if (!model) return "未知型号";
    const normalized = model.trim().toLowerCase();
    return normalized === "xl330" ? "XL330" : normalized === "hd1910" ? "HD1910" : model;
  };
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
    void Promise.all(store.state.physicalMachines.map((machine) => loadRegisters(machine.id)));
  });
  watch(
    () =>
      store.state.physicalMachines
        .map((machine) => `${machine.id}:${machine.connected}:${machine.onlineServoIds.join(".")}`)
        .join(","),
    () =>
      void Promise.all(store.state.physicalMachines.map((machine) => loadRegisters(machine.id))),
  );

  return () => (
    <div class={[style.page, style.physicalMachinesPage]}>
      <section class={[style.physicalConnectionBar, style.panelCard]} aria-label="舵机连接">
        <label>
          USB/串口
          <select
            ref={portSelect}
            value={port.value}
            onChange={() => (port.value = portSelect.value?.value ?? "")}
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
            ref={baudSelect}
            value={baud.value}
            onChange={() => (baud.value = baudSelect.value?.value ?? "")}
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
            ref={autoReconnectInput}
            type="checkbox"
            checked={store.state.autoReconnectEnabled}
            onChange={() =>
              void setAutoReconnectEnabled(autoReconnectInput.value?.checked === true)
            }
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
                  <span
                    class={[
                      style.physicalExpandIcon,
                      style.settingsChevron,
                      isExpanded(machine.id) && style.expanded,
                    ]}
                    aria-hidden="true"
                  />
                </button>
                <label class={style.physicalNameField}>
                  <span>Name：</span>
                  <input
                    ref={(element) => {
                      // oxlint-disable-next-line no-unsafe-type-assertion
                      if (element) nameInputs.set(machine.id, element as HTMLInputElement);
                      else nameInputs.delete(machine.id);
                    }}
                    value={machineName(machine.id, machine.name)}
                    onInput={() => {
                      names.value[machine.id] = nameInputs.get(machine.id)?.value ?? "";
                    }}
                    onBlur={() => void rename(machine.id, machine.name)}
                    onKeydown={(event: KeyboardEvent) => {
                      if (event.key === "Enter") nameInputs.get(machine.id)?.blur();
                      if (event.key === "Escape") {
                        names.value[machine.id] = machine.name;
                        nameInputs.get(machine.id)?.blur();
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
                    ref={(element) => {
                      if (element) {
                        // oxlint-disable-next-line no-unsafe-type-assertion
                        virtualMachineSelects.set(machine.id, element as HTMLSelectElement);
                      } else virtualMachineSelects.delete(machine.id);
                    }}
                    value={machine.virtualMachineId ?? ""}
                    onChange={() => {
                      const id = virtualMachineSelects.get(machine.id)?.value ?? "";
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
                    <span class={style.tag}>舵机型号 {servoModelLabel(machine)}</span>
                    <button
                      class={[style.smallButton, style.danger, style.physicalDelete]}
                      type="button"
                      onClick={() => void deletePhysicalMachine(machine.id)}
                    >
                      删除
                    </button>
                  </div>
                  <div class={style.physicalRegisterPanel}>
                    {registerErrors.value[machine.id] && (
                      <small class={style.physicalRegisterError}>
                        {registerErrors.value[machine.id]}
                      </small>
                    )}
                    {(registerRows.value[machine.id] ?? []).length > 0 ? (
                      Array.from(
                        (registerRows.value[machine.id] ?? []).reduce((groups, row) => {
                          const group = groups.get(row.servoId) ?? [];
                          group.push(row);
                          groups.set(row.servoId, group);
                          return groups;
                        }, new Map<number, PhysicalServoRegister[]>()),
                      ).map(([, rows], index, groups) => {
                        if (index !== 0) return null;
                        const selectedGroup =
                          groups.find(([id]) => id === activeServoIdForMachine(machine)) ??
                          groups[0];
                        const selectedServoId = selectedGroup[0];
                        const selectedRows = selectedGroup[1] ?? rows;
                        return (
                          <>
                            <div class={style.physicalServoTabs} role="tablist">
                              {groups.map(([id]) => (
                                <button
                                  class={[
                                    style.physicalServoTab,
                                    id === selectedServoId && style.active,
                                  ]}
                                  type="button"
                                  role="tab"
                                  aria-selected={id === selectedServoId}
                                  onClick={() => (selectedServoIds.value[machine.id] = id)}
                                >
                                  舵机 {id}
                                </button>
                              ))}
                            </div>
                            <section class={style.physicalRegisterGroup}>
                              <table>
                                <thead>
                                  <tr>
                                    <th>地址</th>
                                    <th>Size</th>
                                    <th>寄存器</th>
                                    <th>值</th>
                                    <th>读写</th>
                                  </tr>
                                </thead>
                                <tbody>
                                  {selectedRows.map((row) => {
                                    const key = registerRowKey(machine.id, row);
                                    const displayedValue =
                                      registerDrafts.value[key] ??
                                      (row.value === null || row.value === undefined
                                        ? "—"
                                        : String(row.value));
                                    return (
                                      <tr>
                                        <td>
                                          <code>{registerAddress(row.address)}</code>
                                        </td>
                                        <td>{row.size}</td>
                                        <td>{registerLabel(row.name)}</td>
                                        <td>
                                          {row.writable ? (
                                            <Slider
                                              class={style.physicalRegisterInput}
                                              value={
                                                displayedValue === "—" ? 0 : Number(displayedValue)
                                              }
                                              min={0}
                                              max={2 ** (row.size * 8) - 1}
                                              step={1}
                                              ariaLabel={registerLabel(row.name)}
                                              onInput={(event) => {
                                                registerDrafts.value[key] = String(event.value);
                                              }}
                                            />
                                          ) : (
                                            <code>{displayedValue}</code>
                                          )}
                                        </td>
                                        <td>
                                          {row.default ? (
                                            "默认"
                                          ) : row.writable ? (
                                            <button
                                              class={style.smallButton}
                                              type="button"
                                              disabled={
                                                !machine.connected || displayedValue === "—"
                                              }
                                              onClick={() => void writeRegister(machine.id, row)}
                                            >
                                              写入
                                            </button>
                                          ) : (
                                            "只读"
                                          )}
                                        </td>
                                      </tr>
                                    );
                                  })}
                                </tbody>
                              </table>
                            </section>
                          </>
                        );
                      })
                    ) : (
                      <span class={style.physicalRegisterEmpty}>暂无参数数据</span>
                    )}
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
export async function refreshPhysicalMachines() {
  const store = useDesktopStore();
  store.state.physicalMachines = await desktopBackend().physicalMachines();
  return store.state.physicalMachines;
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
