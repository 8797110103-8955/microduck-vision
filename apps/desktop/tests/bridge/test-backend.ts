import type {
  ActionRecord,
  DetectedPhysicalMachine,
  LoopAction,
  LoopStatus,
  PhysicalMachine,
  PhysicalServoRegister,
  PolicyEntry,
  ReconnectHint,
  RecordingInfo,
  RecordingFrame,
  SceneFrame,
  SceneInfo,
  ServoCatalogEntry,
  ServoIdentity,
  ServoModelKey,
  SimulationParameters,
  VirtualMachine,
} from "../../generated/types";

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
  jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
  jointDirections: machine.jointDirections?.map((binding) => ({ ...binding })),
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
        jointRangeStates: status.jointRangeStates.map((state) => ({ ...state })),
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
  readonly servoPositions = new Map<string, Map<number, number>>();
  private sourceGeneration = 0;
  private sourceVirtualId: string | null = null;
  private status: LoopStatus | null = null;
  private previewStatus: LoopStatus | null = null;
  private recordings: RecordingInfo[] = [];
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
      case "recording_list":
        return this.recordings.map((record) => ({ ...record }));
      case "diagnostic_tail":
        return this.recordings.map((record) =>
          JSON.stringify({
            category: "recording",
            level: "info",
            recording_id: record.id,
            event: record.state,
          }),
        );
      case "frontend_diagnostic":
        return null;
      case "recording_start": {
        const status = this.previewStatus ?? this.status;
        if (!status?.activeBody) throw new Error("请先启动控制环");
        const id = `recording-test-${this.recordings.length + 1}`;
        const record: RecordingInfo = {
          id,
          appSessionId: "test",
          duckId: status.activeBody.physicalMachineId ?? status.activeBody.virtualMachineId ?? "",
          name: stringValue(payloadRecord(payload).name, "name") || id,
          path: `logs/recordings/${id}`,
          startedAtMs: Date.now(),
          endedAtMs: null,
          state: "recording",
          frames: 0,
          droppedFrames: 0,
          error: null,
        };
        this.recordings.push(record);
        return { ...record };
      }
      case "recording_stop": {
        const record = this.recordings.find((candidate) => candidate.state === "recording");
        if (!record) return null;
        record.state = "complete";
        record.frames = 2;
        record.endedAtMs = Date.now();
        return { ...record };
      }
      case "recording_frames": {
        const status = this.previewStatus ?? this.status;
        if (!status) return [];
        const window = payloadRecord(payloadRecord(payload).window);
        const rows: RecordingFrame[] = [0, 1].map((tick) => ({
          recordingId: String(window.id),
          duckId: "official-hd1910",
          segmentId: 1,
          episodeId: 1,
          elapsedS: tick * 0.02,
          controlStartedS: tick * 0.02,
          status: { ...status, tick },
          policy: null,
        }));
        return rows;
      }
      case "recording_compare":
        return {
          metrics: [],
          matchedFrames: 2,
          unmatchedFrames: 0,
          path: "logs/recordings/comparison.json",
          warnings: [],
        };
      case "recording_export":
        return { path: "logs/recordings/teacher-motion", samples: 2, excludedFrames: 0 };
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
        const slotId = stringValue(input.slotId, "slotId");
        if ((this.previewStatus ?? this.status)?.activeBody?.physicalMachineId === machine.id)
          throw new Error("请先停止这台实机的控制环，再修改肢体映射或方向");
        if (input.direction != null) {
          machine.jointDirections = [
            ...(machine.jointDirections ?? []).filter((binding) => binding.slotId !== slotId),
            { slotId, direction: Number(input.direction) },
          ];
        }
        return cloneMachine(machine);
      }
      case "virtual_machine_bindings": {
        const value = payloadRecord(payload);
        const machine = this.virtual.find((entry) => entry.id === value.virtualMachineId);
        if (!machine) throw new Error("unknown virtual machine");
        const updates = Array.isArray(value.bindings) ? value.bindings.map(payloadRecord) : [];
        const bindings = new Map(
          (machine.jointBindings ?? []).map((binding) => [binding.slotId, binding]),
        );
        for (const update of updates) {
          const slotId = String(update.slotId);
          if (update.physicalId == null) bindings.delete(slotId);
          else bindings.set(slotId, { slotId, physicalId: Number(update.physicalId) });
        }
        const ids = [...bindings.values()].map((binding) => binding.physicalId);
        if (new Set(ids).size !== ids.length) throw new Error("duplicate servo ID");
        machine.jointBindings = [...bindings.values()];
        return cloneVirtual(machine);
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
          });
        if (new Set(bindings.map((binding) => binding.physicalId)).size !== bindings.length)
          throw new Error("duplicate servo ID");
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
        return cloneMachine(machine);
      }
      case "physical_machine_center_encoders": {
        const input = payloadRecord(payload);
        const machine = this.machine(stringValue(input.id, "id"));
        if (!machine.connected) throw new Error("实机没有可用连接");
        const virtual = this.virtual.find((entry) => entry.id === input.virtualMachineId);
        if (!virtual) throw new Error("unknown virtual machine");
        const positions = this.servoPositions.get(machine.id) ?? new Map<number, number>();
        const calibratedServoIds: number[] = [];
        for (const binding of virtual.jointBindings ?? []) {
          if (!machine.onlineServoIds.includes(binding.physicalId)) continue;
          const direction =
            machine.jointDirections?.find((entry) => entry.slotId === binding.slotId)?.direction ??
            1;
          const joints = (machine.jointBindings ??= []);
          let calibration = joints.find((entry) => entry.slotId === binding.slotId);
          if (!calibration) {
            calibration = { slotId: binding.slotId, zeroOffsetRad: 0, minRad: null, maxRad: null };
            joints.push(calibration);
          }
          const angle =
            ((positions.get(binding.physicalId) ?? 0) - (calibration.zeroOffsetRad ?? 0)) *
            direction;
          const bodyAngle =
            ((((angle + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
          calibration.zeroOffsetRad = -direction * bodyAngle;
          positions.set(binding.physicalId, 0);
          calibratedServoIds.push(binding.physicalId);
        }
        machine.torqueEnabled = false;
        this.servoPositions.set(machine.id, positions);
        return { machine: cloneMachine(machine), calibratedServoIds, error: null };
      }
      case "physical_machine_set_pose_zero": {
        const machine = this.machine(stringValue(payloadRecord(payload).id, "id"));
        const virtual = this.virtual.find(
          (entry) => entry.id === payloadRecord(payload).virtualMachineId,
        );
        machine.jointBindings = (virtual?.jointBindings ?? [])
          .filter((binding) => machine.onlineServoIds.includes(binding.physicalId))
          .map((binding) => ({
            slotId: binding.slotId,
            zeroOffsetRad: this.servoPositions.get(machine.id)?.get(binding.physicalId) ?? 0,
            minRad: null,
            maxRad: null,
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
      case "runtime_reconfigure":
        this.status = null;
        this.previewStatus = null;
      // The test adapter shares construction after the atomic replacement boundary.
      case "runtime_open": {
        const input = payloadRecord(payload).input;
        if (isRecord(input) && typeof input.physicalMachineId === "string") {
          const existing = this.previewStatus ?? this.status;
          if (
            input.access === "read_only" &&
            existing?.activeBody?.physicalMachineId === input.physicalMachineId
          )
            return cloneStatus(existing);
          const physical = this.openLoop(input);
          if (this.previewStatus && input.access === "read_only") {
            this.previewStatus = {
              ...this.previewStatus,
              readSources: this.status?.readSources ?? [],
              configRevision: this.previewStatus.configRevision + 1,
            };
            return cloneStatus(this.previewStatus);
          }
          return physical;
        }
        if (!isRecord(input)) throw new Error("invalid runtime input");
        return this.openPreviewLoop(stringValue(input.virtualMachineId, "virtualMachineId"));
      }
      case "runtime_select_body": {
        const input = payloadRecord(payload);
        const current = this.previewStatus;
        if (
          !current ||
          current.runtimeGeneration !== input.generation ||
          current.executionGeneration !== input.executionGeneration
        )
          throw new Error("stale execution generation");
        const virtualMachineId = stringValue(input.virtualMachineId, "virtualMachineId");
        const physicalMachineId =
          typeof input.physicalMachineId === "string" ? input.physicalMachineId : null;
        if (physicalMachineId && !this.machine(physicalMachineId).connected)
          throw new Error("真机离线");
        const positions = physicalMachineId
          ? (current.readSources.find((source) => source.id === physicalMachineId)?.positions ??
            Array(15).fill(0.25))
          : Array(15).fill(0);
        return cloneStatus(
          (this.previewStatus = {
            ...current,
            activeBody: { virtualMachineId, physicalMachineId },
            executionGeneration: current.executionGeneration + 1,
            torque: !!physicalMachineId,
            phase: physicalMachineId ? "holding" : "ready",
            readSources: [],
            configRevision: current.configRevision + 1,
            positions: [...positions],
            targets: [...positions],
            world: { position: [0, 0], yaw: 0, distanceM: 0 },
          }),
        );
      }
      case "runtime_command": {
        const request = payloadRecord(payload).request;
        if (!isRecord(request)) throw new Error("invalid runtime command");
        if (request.kind === "input") return this.previewCommand(request.input);
        if (request.kind === "policy") {
          this.previewPolicy(stringValue(request.policy_id, "policy_id"));
          return this.previewCommand(request.input);
        }
        if (request.kind === "resetWorld") {
          if (!this.previewStatus) throw new Error("preview loop is closed");
          this.previewStatus = {
            ...this.previewStatus,
            world: { position: [0, 0], yaw: 0, distanceM: 0 },
          };
          return cloneStatus(this.previewStatus);
        }
        if (request.kind === "action" && this.previewStatus) {
          const action = parseLoopAction(payloadRecord(request.action));
          if (!action) throw new Error("invalid loop action");
          if (action.kind === "drive") {
            this.previewStatus = {
              ...this.previewStatus,
              phase: action.on ? "driving" : "holding",
            };
          }
          return cloneStatus(this.previewStatus);
        }
        if (request.kind === "action") return this.action(request.action);
        return this.command(request.targets);
      }
      case "loop_command":
        return this.command(payloadRecord(payload).targets);
      case "loop_command_at":
        return this.command(payloadRecord(payload).targets);
      case "loop_policy_command":
        return null;
      case "loop_policy_command_at":
        return null;
      case "loop_action":
        return this.action(payloadRecord(payload).action);
      case "loop_action_at":
        return this.action(payloadRecord(payload).action);
      case "loop_tick":
        return this.tick();
      case "loop_tick_at":
        return this.tick();
      case "loop_status":
        this.sampleSource();
        return cloneStatus(this.previewStatus ?? this.status);
      case "loop_close":
        this.status = null;
        return null;
      case "runtime_close":
        this.status = null;
        if (payloadRecord(payload).scope === "readOnlySource" && this.previewStatus) {
          this.previewStatus = {
            ...this.previewStatus,
            readSources: [],
            configRevision: this.previewStatus.configRevision + 1,
          };
          return null;
        }
        this.previewStatus = null;
        return null;
      case "preview_loop_open":
        return this.openPreviewLoop(
          stringValue(payloadRecord(payload).virtualMachineId, "virtualMachineId"),
        );
      case "preview_machine_select":
        return null;
      case "preview_loop_policy":
        return this.previewPolicy(stringValue(payloadRecord(payload).policyId, "policyId"));
      case "preview_loop_policy_at":
        return this.previewPolicy(stringValue(payloadRecord(payload).policyId, "policyId"));
      case "preview_loop_command":
        return this.previewCommand(payloadRecord(payload).input);
      case "preview_loop_command_at":
        return this.previewCommand(payloadRecord(payload).input);
      case "preview_loop_reset_world":
        if (!this.previewStatus) throw new Error("preview loop is closed");
        this.previewStatus = {
          ...this.previewStatus,
          world: { position: [0, 0], yaw: 0, distanceM: 0 },
        };
        return cloneStatus(this.previewStatus);
      case "preview_loop_reset_world_at":
        if (!this.previewStatus) throw new Error("preview loop is closed");
        this.previewStatus = {
          ...this.previewStatus,
          world: { position: [0, 0], yaw: 0, distanceM: 0 },
        };
        return cloneStatus(this.previewStatus);
      case "preview_loop_tick":
        return this.previewTick();
      case "preview_loop_tick_at":
        return this.previewTick();
      case "preview_loop_close":
        this.previewStatus = null;
        return null;
      case "preview_loop_close_at":
        this.previewStatus = null;
        return null;
      default:
        throw new Error(`unhandled test IPC command: ${command}`);
    }
  }

  private sceneModel(): SceneInfo {
    return {
      meshDir: "",
      defaultFrame: { bodies: [[0, 0, 0, 1, 0, 0, 0]], markers: [] },
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
      jointBindings: [],
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

  private sampleSource() {
    const source = this.status?.readSources[0];
    if (!source) return;
    const physicalMachine = this.machine(source.id);
    const virtual = this.virtual.find((machine) => machine.id === this.sourceVirtualId);
    const positions = [
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
    ].map((slotId) => {
      const binding = virtual?.jointBindings?.find((entry) => entry.slotId === slotId);
      if (!binding || !physicalMachine.onlineServoIds.includes(binding.physicalId)) return 0;
      const calibration = physicalMachine.jointBindings?.find((entry) => entry.slotId === slotId);
      const raw =
        this.servoPositions.get(physicalMachine.id)?.get(binding.physicalId) ??
        calibration?.zeroOffsetRad ??
        0;
      const direction =
        physicalMachine.jointDirections?.find((entry) => entry.slotId === slotId)?.direction ?? 1;
      return (raw - (calibration?.zeroOffsetRad ?? 0)) * direction;
    });
    source.positions = positions;
    source.sequence += 1;
    this.status!.positions = positions;
  }

  private openLoop(value: unknown) {
    const input = isRecord(value) ? value : {};
    const physicalMachine = this.machine(stringValue(input.physicalMachineId, "physicalMachineId"));
    if (!physicalMachine.connected) throw new Error("真机离线");
    const virtual = this.virtual.find((machine) => machine.id === input.virtualMachineId);
    this.sourceVirtualId = virtual?.id ?? null;
    const seen = new Set<number>();
    const duplicate = virtual?.jointBindings?.find((binding) => {
      if (seen.has(binding.physicalId)) return true;
      seen.add(binding.physicalId);
      return false;
    });
    if (duplicate) throw new Error(`绑定冲突：舵机 ${duplicate.physicalId} 被重复绑定`);
    const ids = Array.from({ length: 15 }, (_, index) => index + 1);
    const positions = [
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
    ].map((slotId) => {
      const binding = virtual?.jointBindings?.find((entry) => entry.slotId === slotId);
      if (!binding || !physicalMachine.onlineServoIds.includes(binding.physicalId)) return 0;
      const calibration = physicalMachine.jointBindings?.find((entry) => entry.slotId === slotId);
      const raw =
        this.servoPositions.get(physicalMachine.id)?.get(binding.physicalId) ??
        calibration?.zeroOffsetRad ??
        0;
      const direction =
        physicalMachine.jointDirections?.find((entry) => entry.slotId === slotId)?.direction ?? 1;
      return (raw - (calibration?.zeroOffsetRad ?? 0)) * direction;
    });
    this.sourceGeneration += 1;
    this.status = {
      runtimeState: "running",
      hasExecution: input.access === "control",
      activeBody:
        input.access === "control"
          ? { virtualMachineId: null, physicalMachineId: String(input.physicalMachineId) }
          : null,
      simulation: null,
      readSources:
        input.access === "control"
          ? []
          : [
              {
                id: String(input.physicalMachineId),
                generation: this.sourceGeneration,
                sequence: 1,
                sampledAt: 0,
                ageMs: 0,
                health: "fresh",
                positions,
                velocities: ids.map(() => 0),
                validPositions: ids.map(() => true),
                validVelocities: ids.map(() => true),
                lastError: null,
              },
            ],
      runtimeGeneration: 1,
      executionGeneration: 1,
      configRevision: 1,
      phase: "limp",
      session: 0,
      tick: 0,
      torque: false,
      positions,
      targets: ids.map(() => 0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
      jointRangeStates: [],
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
      runtimeState: "running",
      hasExecution: true,
      activeBody: { virtualMachineId, physicalMachineId: null },
      simulation: null,
      readSources: [],
      runtimeGeneration: 1,
      executionGeneration: 1,
      configRevision: 1,
      phase: "ready",
      session: 1,
      tick: 0,
      torque: false,
      positions: Array(15).fill(0),
      targets: Array(15).fill(0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
      jointRangeStates: [],
    };
    return cloneStatus(this.previewStatus);
  }

  private previewPolicy(id: string) {
    if (!this.previewStatus) throw new Error("preview loop is closed");
    const policy = this.policies.find((entry) => entry.id === id);
    if (!policy?.cached) throw new Error("请先下载这个官方策略");
    this.previewStatus = { ...this.previewStatus, phase: "driving", torque: true, lastError: null };
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
