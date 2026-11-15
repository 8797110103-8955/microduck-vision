import type {
  ActionRecord,
  DetectedPhysicalMachine,
  LoopAction,
  LoopStatus,
  PhysicalMachine,
  PhysicalServoRegister,
  PolicyEntry,
  ReconnectHint,
  SceneFrame,
  SceneInfo,
  ServoCatalogEntry,
  ServoIdentity,
  ServoModelKey,
  SimulationParameters,
  VirtualMachine,
} from "../../src/bridge/types";

type InvokePayload = unknown;

type Call = {
  name: string;
  payload?: InvokePayload;
};

type PreviewTestInput = {
  twist: [number, number, number];
  head: [number, number, number, number];
  body: [number, number, number];
  targetDistanceM: number | null;
};

const isRecord = (value: unknown): value is Record<string, unknown> =>
  !!value && typeof value === "object" && !Array.isArray(value);

const payloadRecord = (value: InvokePayload) => {
  if (!isRecord(value)) throw new Error("expected an IPC object payload");
  return value;
};

const stringValue = (value: unknown, field: string) => {
  if (typeof value !== "string") throw new Error(`expected ${field} to be a string`);
  return value;
};

const numberValue = (value: unknown) =>
  typeof value === "number" && Number.isFinite(value) ? value : 0;

const reconnectHint = (value: unknown): ReconnectHint | null => {
  if (!isRecord(value)) return null;
  return {
    portName: stringValue(value.portName, "reconnectHint.portName"),
    vid: typeof value.vid === "number" ? value.vid : null,
    pid: typeof value.pid === "number" ? value.pid : null,
    adapterSerial: typeof value.adapterSerial === "string" ? value.adapterSerial : null,
    product: typeof value.product === "string" ? value.product : null,
  };
};

const census = (value: unknown): ServoIdentity[] => {
  if (!Array.isArray(value)) return [];
  return value.flatMap((entry) => {
    if (!isRecord(entry) || typeof entry.servoId !== "number" || typeof entry.model !== "string")
      return [];
    return [{ servoId: entry.servoId, model: entry.model.trim().toLowerCase() }];
  });
};

const censusCounts = (items: ServoIdentity[]) => {
  const values = new Map<string, number>();
  items.forEach((servo) => {
    const key = `${servo.servoId}:${servo.model}`;
    values.set(key, (values.get(key) ?? 0) + 1);
  });
  return values;
};

const sameCensus = (left: ServoIdentity[] | undefined, right: ServoIdentity[]) => {
  const a = censusCounts(left ?? []);
  const b = censusCounts(right);
  return a.size > 0 && a.size === b.size && [...a].every(([key, value]) => b.get(key) === value);
};

const cloneMachine = (machine: PhysicalMachine): PhysicalMachine => ({
  ...machine,
  census: machine.census?.map((servo) => ({ ...servo })),
  reconnectHint: machine.reconnectHint ? { ...machine.reconnectHint } : machine.reconnectHint,
  onlineServoIds: [...machine.onlineServoIds],
});

const cloneVirtual = (machine: VirtualMachine): VirtualMachine => ({
  ...machine,
  jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
  referencedBy: [...machine.referencedBy],
});

const cloneStatus = (status: LoopStatus | null) =>
  status
    ? {
        ...status,
        positions: [...status.positions],
        targets: [...status.targets],
      }
    : null;

const parseLoopAction = (value: Record<string, unknown>): LoopAction | null => {
  if (value.kind === "power" && typeof value.on === "boolean")
    return { kind: "power", on: value.on };
  if (value.kind === "drive" && typeof value.on === "boolean")
    return { kind: "drive", on: value.on };
  if (value.kind === "reconnect" && typeof value.session === "number")
    return { kind: "reconnect", session: value.session };
  if (value.kind === "stop" || value.kind === "estop") return { kind: value.kind };
  return null;
};

function simulationFor(catalog: ServoCatalogEntry[], model: ServoModelKey): SimulationParameters {
  const profile = catalog.find((entry) => entry.key === model);
  if (!profile) throw new Error(`missing servo profile: ${model}`);
  return {
    damping: profile.simDamping,
    frictionLoss: profile.simFrictionLoss,
    armature: profile.simArmature,
    kp: profile.simKp,
    forceRange: profile.simForceRange,
    worldPosition: [0, 0, 0],
  };
}

export class TestBackend {
  readonly calls: Call[] = [];
  failCatalog = false;

  private readonly catalog: ServoCatalogEntry[];
  private virtual: VirtualMachine[];
  private physical: PhysicalMachine[] = [];
  private readonly actionsList: ActionRecord[] = [
    { id: "stand", name: "站立", kind: "pose", updatedAt: "内置", source: "builtin" },
    { id: "sit", name: "坐下", kind: "pose", updatedAt: "内置", source: "builtin" },
  ];
  private readonly detectedCensus: ServoIdentity[] = [
    { servoId: 1, model: "hd1910" },
    { servoId: 2, model: "hd1910" },
  ];
  private status: LoopStatus | null = null;
  private previewStatus: LoopStatus | null = null;
  private previewInput: PreviewTestInput = {
    twist: [0, 0, 0],
    head: [0, 0, 0, 0],
    body: [0, 0, 0],
    targetDistanceM: null,
  };
  private autoReconnect = true;
  private policies: PolicyEntry[] = [
    {
      id: "alpha-walking",
      name: "Alpha Walking",
      file: "alpha_walking.onnx",
      sourceUrl:
        "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/alpha_walking.onnx",
      cached: false,
      selected: false,
      size: 0,
    },
    {
      id: "alpha-stand",
      name: "Alpha Stand",
      file: "alpha_stand.onnx",
      sourceUrl:
        "https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/alpha_stand.onnx",
      cached: false,
      selected: false,
      size: 0,
    },
  ];

  constructor(catalog: ServoCatalogEntry[]) {
    this.catalog = catalog.map((entry) => ({ ...entry, capabilities: [...entry.capabilities] }));
    this.virtual = [
      {
        id: "official-hd1910",
        name: "机器 0",
        bodyId: "official",
        servoModel: "hd1910",
        simulation: simulationFor(this.catalog, "hd1910"),
        startupSelected: true,
        referencedBy: [],
      },
    ];
  }

  async invoke(command: string, payload?: InvokePayload): Promise<unknown> {
    this.calls.push({ name: command, payload });
    if (command.startsWith("plugin:window|"))
      return command === "plugin:window|is_maximized" ? false : null;

    switch (command) {
      case "servo_catalog":
        if (this.failCatalog) throw new Error("模拟配置读取失败");
        return this.catalog.map((entry) => ({ ...entry, capabilities: [...entry.capabilities] }));
      case "official_joint_slots":
        return [
          "left_hip_yaw",
          "left_hip_roll",
          "left_hip_pitch",
          "left_knee",
          "left_ankle",
          "neck_pitch",
          "head_pitch",
          "head_yaw",
          "head_roll",
          "mouth",
          "right_hip_yaw",
          "right_hip_roll",
          "right_hip_pitch",
          "right_knee",
          "right_ankle",
        ];
      case "official_joint_ranges":
        return [
          "left_hip_yaw",
          "left_hip_roll",
          "left_hip_pitch",
          "left_knee",
          "left_ankle",
          "neck_pitch",
          "head_pitch",
          "head_yaw",
          "head_roll",
          "mouth",
          "right_hip_yaw",
          "right_hip_roll",
          "right_hip_pitch",
          "right_knee",
          "right_ankle",
        ].map((slotId) => ({ slotId, minRad: -Math.PI, maxRad: Math.PI }));
      case "official_scene_model":
        return this.sceneModel();
      case "official_scene_frame":
        return this.sceneFrame(payloadRecord(payload).positions);
      case "virtual_machines":
        return this.virtual.map((machine) => ({
          ...machine,
          referencedBy: [...machine.referencedBy],
        }));
      case "virtual_machine_create":
        return this.createVirtualMachine(payloadRecord(payload).input);
      case "physical_machines":
        return this.physical.map(cloneMachine);
      case "auto_reconnect_enabled":
        return this.autoReconnect;
      case "set_auto_reconnect_enabled":
        this.autoReconnect = payloadRecord(payload).enabled === true;
        return this.autoReconnect;
      case "physical_ports":
        return [
          { portName: "COM1", vid: null, pid: null, adapterSerial: null, product: "Test port" },
        ];
      case "physical_baud_choices":
        return [1_000_000, 576_000, 115_200, 57_600];
      case "physical_machine_probe":
        return this.probe(payloadRecord(payload).input);
      case "physical_machine_detect":
        return this.detected(payloadRecord(payload).input);
      case "physical_machine_save":
        return this.save(payloadRecord(payload).input);
      case "physical_machine_disconnect":
        return this.updateConnection(stringValue(payloadRecord(payload).id, "id"), false);
      case "physical_machine_reconnect":
        return this.reconnect(payloadRecord(payload).input);
      case "physical_machine_auto_reconnect":
        return this.updateConnection(stringValue(payloadRecord(payload).id, "id"), true);
      case "physical_machine_connect":
        return this.updateConnection(stringValue(payloadRecord(payload).id, "id"), true);
      case "physical_machine_scan":
        return this.scan(stringValue(payloadRecord(payload).id, "id"));
      case "physical_machine_registers":
        return this.registers(stringValue(payloadRecord(payload).id, "id"));
      case "physical_machine_read_register": {
        const input = payloadRecord(payload);
        const machine = this.machine(stringValue(input.id, "id"));
        const servoId = numberValue(input.servoId);
        const row = this.registers(machine.id).find(
          (entry) => entry.servoId === servoId && entry.name === stringValue(input.name, "name"),
        );
        if (!row) throw new Error("unknown register");
        return row;
      }
      case "physical_machine_write_register": {
        const input = payloadRecord(payloadRecord(payload).input);
        const machine = this.machine(stringValue(input.physicalMachineId, "physicalMachineId"));
        const servoId = numberValue(input.servoId);
        if (stringValue(input.name, "name") !== "torque_enable")
          throw new Error("register is read-only");
        machine.torqueEnabled = numberValue(input.value) !== 0;
        return {
          servoId,
          address: 40,
          name: "torque_enable",
          size: 1,
          default: false,
          writable: true,
          value: numberValue(input.value),
        };
      }
      case "policy_list":
        return this.policies.map((policy) => ({ ...policy }));
      case "policy_download":
        return this.downloadPolicy(stringValue(payloadRecord(payload).id, "id"));
      case "policy_select":
        return this.selectPolicy(stringValue(payloadRecord(payload).id, "id"));
      case "policy_open_folder":
        return null;
      case "open_logs":
        return null;
      case "physical_machine_mount":
        return this.mount(payloadRecord(payload));
      case "physical_machine_delete":
        this.physical = this.physical.filter(
          (machine) => machine.id !== stringValue(payloadRecord(payload).id, "id"),
        );
        return null;
      case "physical_machine_rename":
        return this.rename(payloadRecord(payload));
      case "physical_machine_bind": {
        const input = payloadRecord(payloadRecord(payload).input);
        const machine = this.machine(stringValue(input.physicalMachineId, "physicalMachineId"));
        const virtual = this.virtual.find(
          (entry) => entry.id === (input.virtualMachineId ?? machine.virtualMachineId),
        );
        if (!virtual) throw new Error("physical machine is not associated with a virtual machine");
        const bindings = (virtual.jointBindings ?? []).filter(
          (binding) => binding.slotId !== input.slotId,
        );
        if (input.physicalId != null)
          bindings.push({
            slotId: String(input.slotId),
            physicalId: Number(input.physicalId),
            direction: 1,
            zeroOffsetRad: 0,
          });
        virtual.jointBindings = bindings;
        return cloneMachine(machine);
      }
      case "virtual_machine_bind": {
        const input = payloadRecord(payloadRecord(payload).input);
        const machine = this.virtual.find(
          (entry) => entry.id === stringValue(input.virtualMachineId, "virtualMachineId"),
        );
        if (!machine) throw new Error("unknown virtual machine");
        const bindings = (machine.jointBindings ?? []).filter(
          (binding) => binding.slotId !== input.slotId,
        );
        if (input.physicalId != null)
          bindings.push({
            slotId: String(input.slotId),
            physicalId: Number(input.physicalId),
            direction: 1,
            zeroOffsetRad: 0,
          });
        machine.jointBindings = bindings;
        return cloneVirtual(machine);
      }
      case "virtual_machine_validate_bindings": {
        const id = stringValue(payloadRecord(payload).id, "id");
        const machine = this.virtual.find((entry) => entry.id === id);
        if (!machine) throw new Error("unknown virtual machine");
        const bindings = machine.jointBindings ?? [];
        const conflictingIds = bindings
          .filter((binding, index) =>
            bindings.slice(index + 1).some((other) => other.physicalId === binding.physicalId),
          )
          .map((binding) => binding.physicalId);
        machine.jointBindings = bindings.filter(
          (binding) => !conflictingIds.includes(binding.physicalId),
        );
        return cloneVirtual(machine);
      }
      case "physical_machine_validate_bindings": {
        const id = stringValue(payloadRecord(payload).id, "id");
        const machine = this.machine(id);
        const virtual = this.virtual.find((entry) => entry.id === machine.virtualMachineId);
        const bindings = virtual?.jointBindings ?? [];
        const conflictingIds = bindings
          .filter((binding, index) =>
            bindings.slice(index + 1).some((other) => other.physicalId === binding.physicalId),
          )
          .map((binding) => binding.physicalId);
        const validBindings = bindings.filter(
          (binding) => !conflictingIds.includes(binding.physicalId),
        );
        if (virtual) virtual.jointBindings = validBindings;
        return cloneMachine(machine);
      }
      case "physical_machine_set_pose_zero": {
        const machine = this.machine(stringValue(payloadRecord(payload).id, "id"));
        const virtual = this.virtual.find((entry) => entry.id === machine.virtualMachineId);
        if (!virtual) throw new Error("physical machine is not associated with a virtual machine");
        virtual.jointBindings = (virtual.jointBindings ?? []).map((binding) => ({
          ...binding,
          zeroOffsetRad: 0,
        }));
        return cloneMachine(machine);
      }
      case "physical_machine_pose":
        this.machine(stringValue(payloadRecord(payload).id, "id"));
        return Array.from({ length: 15 }, () => 0);
      case "physical_machine_set_torque": {
        const id = stringValue(payloadRecord(payload).id, "id");
        const machine = this.machine(id);
        machine.torqueEnabled = payloadRecord(payload).enabled === true;
        return cloneMachine(machine);
      }
      case "physical_machine_write_position": {
        const input = payloadRecord(payload);
        const machine = this.machine(stringValue(input.id, "id"));
        if (!machine.connected) throw new Error("physical machine is not connected");
        return cloneMachine(machine);
      }
      case "actions":
        return this.actionsList.map((action) => ({ ...action }));
      case "loop_open":
        return this.openLoop(payloadRecord(payload).input);
      case "loop_command":
        return this.command(payloadRecord(payload).targets);
      case "loop_policy_command":
        return null;
      case "loop_action":
        return this.action(payloadRecord(payload).action);
      case "loop_tick":
        return this.tick();
      case "loop_status":
        return cloneStatus(this.status);
      case "loop_close":
        this.status = null;
        return null;
      case "preview_loop_open":
        return this.openPreviewLoop(
          stringValue(payloadRecord(payload).virtualMachineId, "virtualMachineId"),
        );
      case "preview_loop_policy":
        return this.previewPolicy(stringValue(payloadRecord(payload).policyId, "policyId"));
      case "preview_loop_command":
        return this.previewCommand(payloadRecord(payload).input);
      case "preview_loop_reset_world":
        if (!this.previewStatus) throw new Error("preview loop is closed");
        this.previewStatus = {
          ...this.previewStatus,
          world: { position: [0, 0], yaw: 0, distanceM: 0 },
        };
        return cloneStatus(this.previewStatus);
      case "preview_loop_tick":
        return this.previewTick();
      case "preview_loop_close":
        this.previewStatus = null;
        return null;
      default:
        throw new Error(`unhandled test IPC command: ${command}`);
    }
  }

  private sceneModel(): SceneInfo {
    return {
      meshDir: "",
      model: {
        bodies: [{ name: "official-duck", geoms: [] }],
        joints: [],
        trunkHeight: 0.2,
        headCamera: null,
      },
    };
  }

  private sceneFrame(value: unknown): SceneFrame {
    const positions = Array.isArray(value)
      ? value.map((entry) => (typeof entry === "number" ? entry : 0))
      : [];
    return {
      bodies: [[0, 0, 0, 1, 0, 0, 0]],
      markers: positions.slice(0, 15).map(() => [0, 0, 0]),
    };
  }

  private detected(value: unknown): DetectedPhysicalMachine {
    const input = isRecord(value) ? value : {};
    const found = census(input.census);
    const valid =
      found.length > 0 &&
      found.every((servo, index) => {
        const knownModel = servo.model === "hd1910" || servo.model === "xl330";
        const uniqueId =
          found.findIndex((candidate) => candidate.servoId === servo.servoId) === index;
        return knownModel && uniqueId;
      });
    return {
      census: found,
      md5: valid ? "test-census-1-2" : null,
      reconnectHint: reconnectHint(input.reconnectHint),
      matchResult: valid ? "new" : "incompleteCensus",
      diagnostics: valid ? [] : ["IncompleteCensus"],
    };
  }

  private probe(value: unknown) {
    const input = isRecord(value) ? value : {};
    return this.detected({ census: this.detectedCensus, reconnectHint: input.reconnectHint });
  }

  private save(value: unknown) {
    const input = isRecord(value) ? value : {};
    const found = census(input.census);
    if (!sameCensus(this.detectedCensus, found))
      throw new Error("physical machine identity is invalid");
    const hint = reconnectHint(input.reconnectHint);
    const machine: PhysicalMachine = {
      id: `physical-${this.physical.length + 1}`,
      name:
        typeof input.name === "string" && input.name.trim() ? input.name.trim() : "test-census-1-2",
      usbPort: hint?.portName ?? "",
      machineCode: "test-census-1-2",
      census: found,
      reconnectHint: hint,
      virtualMachineId: null,
      onlineServoIds: found.map((servo) => servo.servoId),
      connected: true,
      lastSeen: "刚刚",
    };
    this.physical.push(machine);
    return cloneMachine(machine);
  }

  private machine(id: string) {
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    return machine;
  }

  private registers(id: string): PhysicalServoRegister[] {
    const machine = this.machine(id);
    return (machine.census ?? []).flatMap((servo) => [
      {
        servoId: servo.servoId,
        address: 0,
        name: "model_number",
        size: 2,
        default: false,
        writable: false,
        value: 0,
      },
      {
        servoId: servo.servoId,
        address: 40,
        name: "torque_enable",
        size: 1,
        default: false,
        writable: true,
        value: machine.torqueEnabled ? 1 : 0,
      },
      {
        servoId: servo.servoId,
        address: 56,
        name: "present_position",
        size: 2,
        default: false,
        writable: false,
        value: 0,
      },
    ]);
  }

  private createVirtualMachine(value: unknown): VirtualMachine {
    const input = isRecord(value) ? value : {};
    const machine: VirtualMachine = {
      id: `virtual-${this.virtual.length + 1}`,
      name: stringValue(input.name, "name"),
      bodyId: stringValue(input.bodyId, "bodyId"),
      servoModel: input.servoModel === "xl330" ? "xl330" : "hd1910",
      simulation: {
        damping: numberValue(
          input.simulation && isRecord(input.simulation) ? input.simulation.damping : 0,
        ),
        frictionLoss: numberValue(
          input.simulation && isRecord(input.simulation) ? input.simulation.frictionLoss : 0,
        ),
        armature: numberValue(
          input.simulation && isRecord(input.simulation) ? input.simulation.armature : 0,
        ),
        kp: numberValue(input.simulation && isRecord(input.simulation) ? input.simulation.kp : 0),
        forceRange: numberValue(
          input.simulation && isRecord(input.simulation) ? input.simulation.forceRange : 0,
        ),
        worldPosition:
          input.simulation &&
          isRecord(input.simulation) &&
          Array.isArray(input.simulation.worldPosition)
            ? (() => {
                const position = input.simulation.worldPosition.slice(0, 3).map(numberValue);
                return [position[0] ?? 0, position[1] ?? 0, position[2] ?? 0] satisfies [
                  number,
                  number,
                  number,
                ];
              })()
            : [0, 0, 0],
      },
      startupSelected: false,
      referencedBy: [],
    };
    this.virtual.push(machine);
    return { ...machine, referencedBy: [] };
  }

  private updateConnection(id: string, connected: boolean) {
    const machine = this.machine(id);
    machine.connected = connected;
    machine.lastSeen = connected ? "刚刚" : machine.lastSeen;
    return cloneMachine(machine);
  }

  private reconnect(value: unknown) {
    const input = isRecord(value) ? value : {};
    const machine = this.machine(stringValue(input.id, "input.id"));
    const found = census(input.census);
    if (!sameCensus(machine.census, found)) {
      machine.connected = false;
      throw new Error("reconnect census does not match saved physical machine");
    }
    machine.connected = true;
    machine.reconnectHint = reconnectHint(input.reconnectHint);
    return cloneMachine(machine);
  }

  private scan(id: string) {
    const machine = this.machine(id);
    machine.onlineServoIds = machine.census?.map((servo) => servo.servoId) ?? [1, 2];
    return cloneMachine(machine);
  }

  private downloadPolicy(id: string) {
    const policy = this.policies.find((entry) => entry.id === id);
    if (!policy) throw new Error(`unknown policy: ${id}`);
    this.policies.forEach((entry) => (entry.selected = entry.id === id));
    policy.cached = true;
    policy.size = 42;
    return { ...policy };
  }

  private selectPolicy(id: string) {
    const policy = this.policies.find((entry) => entry.id === id);
    if (!policy?.cached) throw new Error("请先下载这个官方策略");
    this.policies.forEach((entry) => (entry.selected = entry.id === id));
    return { ...policy, selected: true };
  }

  private mount(value: Record<string, unknown>) {
    const machine = this.machine(stringValue(value.id, "id"));
    const virtualMachineId = stringValue(value.virtualMachineId, "virtualMachineId");
    const virtualMachine = this.virtual.find((entry) => entry.id === virtualMachineId);
    if (!virtualMachine) throw new Error("unknown virtual machine");
    machine.virtualMachineId = virtualMachineId;
    if (!virtualMachine.referencedBy.includes(machine.id))
      virtualMachine.referencedBy.push(machine.id);
    return cloneMachine(machine);
  }

  private rename(value: Record<string, unknown>) {
    const machine = this.machine(stringValue(value.id, "id"));
    machine.name = stringValue(value.name, "name");
    return cloneMachine(machine);
  }

  private openLoop(value: unknown) {
    const input = isRecord(value) ? value : {};
    const virtualMachine = this.virtual.find(
      (entry) =>
        entry.id === (typeof input.virtualMachineId === "string" ? input.virtualMachineId : ""),
    );
    const seen = new Set<number>();
    const duplicate = virtualMachine?.jointBindings?.find((binding) => {
      if (seen.has(binding.physicalId)) return true;
      seen.add(binding.physicalId);
      return false;
    });
    if (duplicate) throw new Error(`绑定冲突：舵机 ${duplicate.physicalId} 被重复绑定`);
    const ids = Array.isArray(input.servoIds)
      ? input.servoIds.filter((id): id is number => typeof id === "number")
      : [];
    this.status = {
      phase: "limp",
      session: 0,
      tick: 0,
      torque: false,
      positions: ids.map(() => 0),
      targets: ids.map(() => 0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
    };
    return cloneStatus(this.status);
  }

  private command(value: unknown) {
    if (!this.status) throw new Error("loop is closed");
    const targets = Array.isArray(value)
      ? value.map((target) => (typeof target === "number" ? target : 0))
      : [];
    this.status.targets = targets;
    return null;
  }

  private action(value: unknown) {
    if (!this.status || !isRecord(value)) throw new Error("loop is closed");
    const action = parseLoopAction(value);
    if (!action) throw new Error("invalid loop action");
    if (action.kind === "power") {
      this.status = { ...this.status, phase: action.on ? "homing" : "limp", torque: action.on };
    } else if (
      action.kind === "drive" &&
      action.on &&
      ["ready", "holding"].includes(this.status.phase)
    ) {
      this.status = { ...this.status, phase: "driving" };
    } else if (action.kind === "drive" && !action.on && this.status.phase === "driving") {
      this.status = { ...this.status, phase: "holding" };
    } else if (action.kind === "stop" || action.kind === "estop") {
      this.status = {
        ...this.status,
        phase: action.kind === "stop" ? "stopped" : "estop",
        torque: false,
      };
    }
    return cloneStatus(this.status);
  }

  private tick() {
    if (!this.status) throw new Error("loop is closed");
    const phase =
      this.status.phase === "homing" && this.status.tick >= 4 ? "ready" : this.status.phase;
    this.status = {
      ...this.status,
      phase,
      tick: this.status.tick + 1,
      positions: [...this.status.targets],
    };
    return cloneStatus(this.status);
  }

  private openPreviewLoop(virtualMachineId: string) {
    if (!this.virtual.some((machine) => machine.id === virtualMachineId))
      throw new Error(`unknown virtual machine: ${virtualMachineId}`);
    this.previewStatus = {
      phase: "ready",
      session: 1,
      tick: 0,
      torque: false,
      positions: Array(15).fill(0),
      targets: Array(15).fill(0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
    };
    return cloneStatus(this.previewStatus);
  }

  private previewPolicy(id: string) {
    if (!this.previewStatus) throw new Error("preview loop is closed");
    const policy = this.policies.find((entry) => entry.id === id);
    if (!policy?.cached) throw new Error("请先下载这个官方策略");
    this.previewStatus = { ...this.previewStatus, torque: true, lastError: null };
    return cloneStatus(this.previewStatus);
  }

  private previewCommand(value: unknown) {
    if (!this.previewStatus || !isRecord(value)) throw new Error("preview loop is closed");
    const axis = (name: "twist" | "head" | "body", size: number) => {
      const values = value[name];
      if (!Array.isArray(values)) return Array.from({ length: size }, () => 0);
      return Array.from({ length: size }, (_, index) => numberValue(values[index]));
    };
    const twistValues = axis("twist", 3);
    const headValues = axis("head", 4);
    const bodyValues = axis("body", 3);
    const twist: [number, number, number] = [twistValues[0], twistValues[1], twistValues[2]];
    const head: [number, number, number, number] = [
      headValues[0],
      headValues[1],
      headValues[2],
      headValues[3],
    ];
    const body: [number, number, number] = [bodyValues[0], bodyValues[1], bodyValues[2]];
    this.previewInput = { twist, head, body, targetDistanceM: numberValue(value.targetDistanceM) };
    return cloneStatus(this.previewStatus);
  }

  private previewTick() {
    if (!this.previewStatus) throw new Error("preview loop is closed");
    const tick = this.previewStatus.tick + 1;
    const targets = this.previewStatus.torque
      ? Array.from({ length: 15 }, (_, index) => Math.sin(tick / 12 + index * 0.18) * 0.08)
      : this.previewStatus.targets;
    this.previewStatus = {
      ...this.previewStatus,
      phase:
        this.previewStatus.torque && this.previewStatus.phase === "ready"
          ? "driving"
          : this.previewStatus.phase,
      tick,
      targets: [...targets],
      positions: [...targets],
      world: this.previewStatus.torque
        ? {
            position: [
              (this.previewStatus.world.position[0] ?? 0) + this.previewInput.twist[0] * 0.02,
              (this.previewStatus.world.position[1] ?? 0) + this.previewInput.twist[1] * 0.02,
            ],
            yaw: (this.previewStatus.world.yaw ?? 0) + this.previewInput.twist[2] * 0.02,
            distanceM:
              (this.previewStatus.world.distanceM ?? 0) +
              Math.hypot(this.previewInput.twist[0], this.previewInput.twist[1]) * 0.02,
          }
        : this.previewStatus.world,
    };
    return cloneStatus(this.previewStatus);
  }
}
