//! Read-only descriptions of configurations supplied by the Python task registry.

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TaskEnvironmentConfiguration {
    pub num_envs: u32,
    pub control_hz: f64,
    pub episode_length_s: f64,
    pub terrain: Option<String>,
    pub json: String,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, specta::Type)]
#[serde(rename_all = "camelCase")]
pub struct TaskConfiguration {
    pub task_id: String,
    pub runner: String,
    pub algorithm: String,
    pub max_iterations: u32,
    pub training: TaskEnvironmentConfiguration,
    pub play: TaskEnvironmentConfiguration,
    pub rl_json: String,
}
