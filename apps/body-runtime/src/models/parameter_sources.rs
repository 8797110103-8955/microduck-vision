use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ParameterClass {
    Fact,
    SimulationApproximation,
    CompatibilityMeasure,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RealMachineMapping {
    Implemented,
    PhysicalEffect,
    SimulationOnly,
    Unverified,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ParameterSource {
    pub parameter: &'static str,
    pub class: ParameterClass,
    pub source: &'static str,
    pub assumption: Option<&'static str>,
    pub real_machine_mapping: RealMachineMapping,
}
