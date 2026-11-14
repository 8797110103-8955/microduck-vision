import { computed, defineComponent, onMounted, onUnmounted, ref } from "vue";
import type { ServoCatalogEntry, ServoModelKey, SimulationParameters } from "../../bridge/types";
import { desktopBackend, useDesktopStore } from "../../store";
import { servoModel, valueOf } from "../debug/helpers";
import style from "./style.module.scss";

const randomFactor = (spread: number) => 1 - spread + Math.random() * spread * 2;

export const SimulationPage = defineComponent(() => {
  const store = useDesktopStore();
  const expanded = ref<string[]>([]);
  const editing = ref<string | null>(null);
  const draft = ref("");
  const editingWasExpanded = ref(false);
  const createOpen = ref(false);
  const newName = ref("");
  const newServo = ref<ServoModelKey>(store.state.selectedServo);
  const newSimulation = ref<SimulationParameters>(simulationFor(newServo.value));

  const simulationFields: Array<
    [keyof SimulationParameters, string, string, keyof ServoCatalogEntry]
  > = [
    ["damping", "damping", "阻尼", "simDamping"],
    ["frictionLoss", "frictionloss", "摩擦损失", "simFrictionLoss"],
    ["armature", "armature", "转子惯量", "simArmature"],
    ["kp", "kp", "刚度", "simKp"],
    ["forceRange", "forcerange", "力范围", "simForceRange"],
  ];

  function simulationFor(model: ServoModelKey) {
    const profile = store.state.catalog.find((entry) => entry.key === model);
    if (!profile) throw new Error(`Missing servo profile for ${model}`);
    return {
      damping: profile.simDamping,
      frictionLoss: profile.simFrictionLoss,
      armature: profile.simArmature,
      kp: profile.simKp,
      forceRange: profile.simForceRange,
    };
  }

  function simulationKey(simulation: SimulationParameters) {
    return simulationFields.map(([key]) => (simulation[key] ?? 0).toFixed(6)).join("/");
  }

  function fieldsFor(model: ServoModelKey) {
    const profile = store.state.catalog.find((entry) => entry.key === model);
    return simulationFields.filter(([, , , profileKey]) => profile?.[profileKey] != null);
  }

  function randomSimulation(model: ServoModelKey) {
    const base = simulationFor(model);
    const used = new Set(
      store.state.virtualMachines.map((machine) => simulationKey(machine.simulation)),
    );
    for (let attempt = 0; attempt < 100; attempt += 1) {
      const candidate = {
        damping: Number(((base.damping ?? 0) * randomFactor(0.1)).toFixed(6)),
        frictionLoss: Number(((base.frictionLoss ?? 0) * randomFactor(0.1)).toFixed(6)),
        armature: Number(((base.armature ?? 0) * randomFactor(0.1)).toFixed(6)),
        kp: Number(((base.kp ?? 0) * randomFactor(0.15)).toFixed(6)),
        forceRange: Number(((base.forceRange ?? 0) * randomFactor(0.1)).toFixed(6)),
      };
      if (!used.has(simulationKey(candidate))) return candidate;
    }
    return {
      ...base,
      kp: Number(((base.kp ?? 0) + (store.state.virtualMachines.length + 1) * 0.000001).toFixed(6)),
    };
  }
  const physicalNamesByMachine = computed(() => {
    const result = new Map<string, string[]>();
    store.state.physicalMachines.forEach((machine) => {
      if (!machine.virtualMachineId) return;
      const names = result.get(machine.virtualMachineId) ?? [];
      names.push(machine.name);
      result.set(machine.virtualMachineId, names);
    });
    return result;
  });

  const toggle = (id: string) => {
    expanded.value = expanded.value.includes(id)
      ? expanded.value.filter((value) => value !== id)
      : [...expanded.value, id];
  };

  const beginEdit = (id: string, name: string) => {
    if (editing.value) return;
    editingWasExpanded.value = expanded.value.includes(id);
    expanded.value = expanded.value.filter((value) => value !== id);
    editing.value = id;
    draft.value = name;
  };

  const finishEdit = async (id: string, save: boolean) => {
    if (editing.value !== id) return;
    const name = draft.value.trim();
    const restoreExpanded = editingWasExpanded.value;
    editing.value = null;
    if (save && name) await renameVirtualMachine(id, name);
    if (restoreExpanded) expanded.value = [...expanded.value, id];
  };

  const create = async () => {
    const name = newName.value.trim() || `机器 ${store.state.virtualMachines.length}`;
    await createVirtualMachine(name, newServo.value, newSimulation.value);
    newName.value = "";
    createOpen.value = false;
  };

  const closeCreatePanel = () => {
    createOpen.value = false;
  };

  const onDocumentPointerDown = (event: PointerEvent) => {
    if (!(event.target instanceof Element)) return;
    if (
      event.target.closest(".simulation-create-panel") ||
      event.target.closest(".simulation-add")
    ) {
      return;
    }
    closeCreatePanel();
  };

  onMounted(() => {
    document.addEventListener("pointerdown", onDocumentPointerDown);
    window.addEventListener("blur", closeCreatePanel);
  });

  onUnmounted(() => {
    document.removeEventListener("pointerdown", onDocumentPointerDown);
    window.removeEventListener("blur", closeCreatePanel);
  });

  return () => (
    <div class={[style.page, style.simulationPage]}>
      <div class={style.simulationToolbar}>
        <span class={style.muted}>所有模拟机将显示在首页</span>
        <button
          class={style.simulationAdd}
          type="button"
          aria-label="新建模拟机"
          title="新建模拟机"
          onClick={() => {
            newServo.value = store.state.selectedServo;
            newSimulation.value = randomSimulation(newServo.value);
            createOpen.value = !createOpen.value;
          }}
        >
          <span aria-hidden="true">+</span>
        </button>
        {createOpen.value && (
          <div class={style.simulationCreatePanel}>
            <label>
              名称
              <input
                autofocus
                placeholder="模拟机名称"
                value={newName.value}
                onInput={(event: Event) => (newName.value = valueOf(event))}
                onKeydown={(event: KeyboardEvent) => {
                  if (event.key === "Enter") void create();
                  if (event.key === "Escape") closeCreatePanel();
                }}
              />
            </label>
            <label>
              舵机
              <select
                value={newServo.value}
                onChange={(event: Event) => {
                  newServo.value = servoModel(valueOf(event));
                  newSimulation.value = randomSimulation(newServo.value);
                }}
              >
                {store.state.catalog.map((entry) => (
                  <option value={entry.key}>{entry.label}</option>
                ))}
              </select>
            </label>
            <div class={style.simulationCreateParameters}>
              <strong>仿真参数</strong>
              {fieldsFor(newServo.value).map(([key, label, description]) => (
                <label class={style.simulationParameterInput}>
                  <span>
                    {label}
                    <small>{description}</small>
                  </span>
                  <input
                    type="number"
                    step="any"
                    value={newSimulation.value[key] ?? ""}
                    onInput={(event: Event) => {
                      newSimulation.value = {
                        ...newSimulation.value,
                        [key]: Number(valueOf(event)),
                      };
                    }}
                  />
                </label>
              ))}
            </div>
            <div class={style.simulationCreateActions}>
              <button type="button" onClick={closeCreatePanel}>
                取消
              </button>
              <button type="button" class={style.primary} onClick={() => void create()}>
                创建
              </button>
            </div>
          </div>
        )}
      </div>
      <div class={[style.panelCard, style.simulationCard]}>
        <div class={style.simulationList}>
          {store.state.virtualMachines.map((machine) => {
            const physicalNames = physicalNamesByMachine.value.get(machine.id) ?? [];
            const isExpanded = expanded.value.includes(machine.id);
            return (
              <article
                class={
                  store.state.selectedVirtualMachine === machine.id
                    ? [style.simulationRow, style.selected]
                    : style.simulationRow
                }
                onClick={(event: MouseEvent) => {
                  if (
                    event.target instanceof Element &&
                    event.target.closest("button, input, select")
                  )
                    return;
                  store.state.selectedVirtualMachine = machine.id;
                  store.state.selectedVirtualMachineManual = true;
                  toggle(machine.id);
                }}
              >
                <span
                  class={isExpanded ? [style.expandButton, style.expanded] : style.expandButton}
                  aria-hidden="true"
                >
                  <span class={style.simulationExpandIcon} />
                </span>

                {editing.value === machine.id ? (
                  <input
                    class={style.simulationNameInput}
                    autofocus
                    value={draft.value}
                    onClick={(event: MouseEvent) => event.stopPropagation()}
                    onKeydown={(event: KeyboardEvent) => {
                      if (event.key === "Enter") void finishEdit(machine.id, true);
                      if (event.key === "Escape") void finishEdit(machine.id, false);
                    }}
                    onInput={(event: Event) => (draft.value = valueOf(event))}
                    onBlur={() => void finishEdit(machine.id, true)}
                  />
                ) : (
                  <div class={[style.machineMain, style.simulationMachineName]}>
                    <div
                      class={style.machineName}
                      onClick={(event: MouseEvent) => event.stopPropagation()}
                    >
                      <strong
                        title="双击改名"
                        onDblclick={(event: MouseEvent) => {
                          event.stopPropagation();
                          beginEdit(machine.id, machine.name);
                        }}
                      >
                        {machine.name}
                      </strong>
                      <span class={style.simulationSeparator} aria-hidden="true">
                        |
                      </span>
                      <span class={style.simulationPhysical}>
                        {physicalNames.length ? physicalNames.join("、") : "实机"}
                      </span>
                    </div>
                    <span class={style.machineModel}>
                      {store.state.catalog.find((entry) => entry.key === machine.servoModel)
                        ?.label ?? machine.servoModel}
                    </span>
                  </div>
                )}
                {isExpanded && editing.value !== machine.id && (
                  <div class={style.simulationExpanded}>
                    <button
                      class={
                        machine.startupSelected ? [style.startup, style.active] : style.startup
                      }
                      type="button"
                      onClick={() => void setStartupVirtualMachine(machine.id)}
                    >
                      {machine.startupSelected ? "启动时选中" : "设为启动项"}
                    </button>
                    <button
                      class={[style.smallButton, style.danger]}
                      type="button"
                      onClick={() => void deleteVirtualMachine(machine.id)}
                      disabled={machine.startupSelected || machine.referencedBy.length > 0}
                    >
                      删除实例
                    </button>
                    <div class={style.simulationParameterList}>
                      <strong>仿真参数</strong>
                      {fieldsFor(machine.servoModel).map(([key, label, description]) => (
                        <span>
                          <small>
                            {label}（{description}）
                          </small>
                          <b>{machine.simulation[key] ?? "—"}</b>
                        </span>
                      ))}
                    </div>
                  </div>
                )}
              </article>
            );
          })}
        </div>
      </div>
    </div>
  );
});

export async function createVirtualMachine(
  name: string,
  model?: ServoModelKey,
  simulation?: SimulationParameters,
) {
  const store = useDesktopStore();
  const selectedServo = model ?? store.state.selectedServo;
  const profile =
    store.state.catalog.find((entry) => entry.key === selectedServo) ?? store.state.catalog[0];
  if (!simulation && !profile) throw new Error(`Missing servo profile for ${selectedServo}`);
  const resolvedSimulation = simulation ?? {
    damping: profile.simDamping,
    frictionLoss: profile.simFrictionLoss,
    armature: profile.simArmature,
    kp: profile.simKp,
    forceRange: profile.simForceRange,
  };
  const machine = await desktopBackend().createVirtualMachine({
    name,
    bodyId: "official",
    servoModel: selectedServo,
    simulation: resolvedSimulation,
  });
  store.state.virtualMachines.push(machine);
  return machine;
}
export async function setStartupVirtualMachine(id: string) {
  const store = useDesktopStore();
  await desktopBackend().setStartupVirtualMachine(id);
  store.state.virtualMachines.forEach((machine) => (machine.startupSelected = machine.id === id));
  store.state.selectedVirtualMachine = id;
  store.state.selectedVirtualMachineManual = true;
}
export async function deleteVirtualMachine(id: string) {
  const store = useDesktopStore();
  const wasSelected = store.state.selectedVirtualMachine === id;
  await desktopBackend().deleteVirtualMachine(id);
  store.state.virtualMachines = store.state.virtualMachines.filter((machine) => machine.id !== id);
  if (wasSelected) {
    store.state.selectedVirtualMachine = null;
    store.state.selectedVirtualMachineManual = false;
  }
}
export async function renameVirtualMachine(id: string, name: string) {
  const store = useDesktopStore();
  const machine = await desktopBackend().renameVirtualMachine(id, name);
  const index = store.state.virtualMachines.findIndex((entry) => entry.id === id);
  if (index >= 0) store.state.virtualMachines[index] = machine;
  return machine;
}
