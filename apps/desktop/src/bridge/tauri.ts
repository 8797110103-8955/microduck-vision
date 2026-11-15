import { commands } from "../../generated/types";
import type { DesktopBackend } from "./types";

const discard = (promise: Promise<unknown>): Promise<void> => promise.then(() => undefined);
export function createTauriBackend(): DesktopBackend {
  return {
    servoCatalog: () => commands.servoCatalog(),
    officialJointSlots: () => commands.officialJointSlots(),
    officialJointRanges: () => commands.officialJointRanges(),
    virtualMachines: () => commands.virtualMachines(),
    physicalMachines: () => commands.physicalMachines(),
    autoReconnectEnabled: () => commands.autoReconnectEnabled(),
    setAutoReconnectEnabled: (enabled) => commands.setAutoReconnectEnabled(enabled),
    physicalPorts: () => commands.physicalPorts(),
    physicalBaudChoices: () => commands.physicalBaudChoices(),
    probePhysicalMachine: (input) => commands.physicalMachineProbe(input),
    detectPhysicalMachine: (input) => commands.physicalMachineDetect(input),
    savePhysicalMachine: (input) => commands.physicalMachineSave(input),
    disconnectPhysicalMachine: (id) => commands.physicalMachineDisconnect(id),
    reconnectPhysicalMachine: (input) => commands.physicalMachineReconnect(input),
    autoReconnectPhysicalMachine: (id, baud) => commands.physicalMachineAutoReconnect(id, baud),
    createPhysicalMachine: (input) => commands.physicalMachineCreate(input),
    connectPhysicalMachine: (id) => commands.physicalMachineConnect(id),
    mountPhysicalMachine: (id, virtualMachineId) =>
      commands.physicalMachineMount(id, virtualMachineId),
    deletePhysicalMachine: (id) => discard(commands.physicalMachineDelete(id)),
    renamePhysicalMachine: (id, name) => commands.physicalMachineRename(id, name),
    bindPhysicalMachine: (input) => commands.physicalMachineBind(input),
    bindVirtualMachine: (input) => commands.virtualMachineBind(input),
    validateVirtualMachineBindings: (id) => commands.virtualMachineValidateBindings(id),
    validatePhysicalMachineBindings: (id) => commands.physicalMachineValidateBindings(id),
    setPhysicalMachinePoseZero: (id, virtualMachineId) =>
      commands.physicalMachineSetPoseZero(id, virtualMachineId ?? null),
    physicalMachinePose: (id, virtualMachineId) =>
      commands
        .physicalMachinePose(id, virtualMachineId ?? null)
        .then((values) => values.map((value) => value ?? 0)),
    setPhysicalMachineTorque: (id, enabled, virtualMachineId) =>
      commands.physicalMachineSetTorque(id, enabled, virtualMachineId ?? null),
    writePhysicalMachinePosition: (input) =>
      commands.physicalMachineWritePosition(
        input.physicalMachineId,
        input.servoId,
        input.positionRad,
      ),
    scanPhysicalMachine: (id) => commands.physicalMachineScan(id),
    physicalMachineRegisters: (id) => commands.physicalMachineRegisters(id),
    readPhysicalMachineRegister: (physicalMachineId, servoId, name) =>
      commands.physicalMachineReadRegister(physicalMachineId, servoId, name),
    writePhysicalMachineRegister: (input) => commands.physicalMachineWriteRegister(input),
    policyList: () => commands.policyList(),
    policyDownload: (id) => commands.policyDownload(id),
    policySelect: (id) => commands.policySelect(id),
    policyOpenFolder: () => discard(commands.policyOpenFolder()),
    openLogs: () => discard(commands.openLogs()),
    createVirtualMachine: (input) => commands.virtualMachineCreate(input),
    setStartupVirtualMachine: (id) => discard(commands.virtualMachineSetStartup(id)),
    setVirtualMachineWorldPosition: (id, position) =>
      commands.virtualMachineSetWorldPosition(id, position),
    deleteVirtualMachine: (id) => discard(commands.virtualMachineDelete(id)),
    renameVirtualMachine: (id, name) => commands.virtualMachineRename(id, name),
    actions: () => commands.actions(),
    createAction: (input) => commands.actionCreate(input),
    deleteAction: (id) => discard(commands.actionDelete(id)),
    loopOpen: (input) => commands.loopOpen(input),
    loopCommand: (targets) => discard(commands.loopCommand(targets)),
    loopPolicyCommand: (input) => discard(commands.loopPolicyCommand(input)),
    loopAction: (action) => commands.loopAction(action),
    loopTick: () => commands.loopTick(),
    loopStatus: () => commands.loopStatus(),
    loopClose: () => discard(commands.loopClose()),
    previewLoopOpen: (virtualMachineId) => commands.previewLoopOpen(virtualMachineId),
    previewLoopPolicy: (policyId) => commands.previewLoopPolicy(policyId),
    previewLoopCommand: (input) => commands.previewLoopCommand(input),
    previewLoopResetWorld: () => commands.previewLoopResetWorld(),
    previewLoopTick: () => commands.previewLoopTick(),
    previewLoopClose: () => discard(commands.previewLoopClose()),
    officialSceneModel: () => commands.officialSceneModel(),
    officialSceneFrame: (positions) => commands.officialSceneFrame(positions),
  };
}
