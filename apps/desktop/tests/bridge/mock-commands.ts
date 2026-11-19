// UI fixtures mock generated command functions; production has no injectable backend state.
import { vi } from "vitest";
import * as commands from "../../generated/types";
import { MemoryBackend } from "./memory";

export function mockMemoryCommands(backend: MemoryBackend) {
  vi.spyOn(commands, "servoCatalog").mockImplementation(() => backend.servoCatalog());
  vi.spyOn(commands, "virtualMachines").mockImplementation(() => backend.virtualMachines());
  vi.spyOn(commands, "physicalMachines").mockImplementation(() => backend.physicalMachines());
  vi.spyOn(commands, "actions").mockImplementation(() => backend.actions());
  vi.spyOn(commands, "autoReconnectEnabled").mockImplementation(() =>
    backend.autoReconnectEnabled(),
  );
  vi.spyOn(commands, "policyList").mockImplementation(() => backend.policyList());
  vi.spyOn(commands, "physicalMachineProbe").mockImplementation((input) =>
    backend.probePhysicalMachine(input),
  );
  vi.spyOn(commands, "physicalMachineSave").mockImplementation((input) =>
    backend.savePhysicalMachine(input),
  );
  vi.spyOn(commands, "physicalMachineCreate").mockImplementation((input) =>
    backend.createPhysicalMachine(input),
  );
  vi.spyOn(commands, "physicalMachineConnect").mockImplementation((id) =>
    backend.connectPhysicalMachine(id),
  );
  vi.spyOn(commands, "physicalMachineScan").mockImplementation((id) =>
    backend.scanPhysicalMachine(id),
  );
  vi.spyOn(commands, "physicalMachineMount").mockImplementation((id, virtualId) =>
    backend.mountPhysicalMachine(id, virtualId),
  );
  vi.spyOn(commands, "physicalMachineRename").mockImplementation((id, name) =>
    backend.renamePhysicalMachine(id, name),
  );
  vi.spyOn(commands, "virtualMachineRename").mockImplementation((id, name) =>
    backend.renameVirtualMachine(id, name),
  );
  vi.spyOn(commands, "physicalMachineDelete").mockImplementation(async (id) => {
    await backend.deletePhysicalMachine(id);
    return null;
  });
  vi.spyOn(commands, "virtualMachineDelete").mockImplementation(async (id) => {
    await backend.deleteVirtualMachine(id);
    return null;
  });
  vi.spyOn(commands, "virtualMachineSetStartup").mockImplementation(async (id) => {
    await backend.setStartupVirtualMachine(id);
    return null;
  });
  vi.spyOn(commands, "actionDelete").mockImplementation(async (id) => {
    await backend.deleteAction(id);
    return null;
  });
  vi.spyOn(commands, "virtualMachineCreate").mockImplementation((input) => {
    if (input.servoModel !== "hd1910" && input.servoModel !== "xl330")
      throw new Error("unknown servo model");
    return backend.createVirtualMachine({
      ...input,
      servoModel: input.servoModel,
      simulation: input.simulation ?? {},
    });
  });
  vi.spyOn(commands, "actionCreate").mockImplementation((input) => {
    if (input.kind !== "pose" && input.kind !== "sequence" && input.kind !== "policy")
      throw new Error("unknown action kind");
    return backend.createAction({ ...input, kind: input.kind });
  });
}
