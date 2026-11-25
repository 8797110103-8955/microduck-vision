//! Serialize a resolved simulator profile and its exact assets for Python.

use base64::{Engine as _, engine::general_purpose::STANDARD};
use duck_body::{AssetResolver, BodyProfile};
use serde::Serialize;

use crate::models::assets::{AssetBundle, SourceAsset};
use crate::models::profile::BodyExecutionProfile;

use super::task::{ResolvedTrainingBody, TaskBodyDescription};

#[derive(Serialize)]
struct AssetBytes<'a> {
    path: &'a str,
    base64: String,
}

#[derive(Serialize)]
struct SimulationExport<'a> {
    format_version: u32,
    units: Units,
    profile: &'a BodyExecutionProfile,
    task: Option<&'a TaskBodyDescription>,
    assets: Vec<AssetBytes<'a>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    backend: Option<super::mjlab::MjlabBodyConfig>,
}

#[derive(Serialize)]
struct Units {
    angle: &'static str,
    angular_velocity: &'static str,
    length: &'static str,
    time: &'static str,
    voltage: &'static str,
}

impl ResolvedTrainingBody {
    pub fn export(&self, source: &dyn AssetResolver) -> Result<Vec<u8>, String> {
        export(&self.profile, source, Some(&self.task))
    }

    pub fn display_scene(
        &self,
        body: &BodyProfile,
        source: &dyn AssetResolver,
    ) -> Result<duck_body::OfficialScene, duck_body::SceneError> {
        let mut template = body.template().clone();
        template.assets.display_mjcf = self.task.robot_xml.as_str().into();
        duck_body::OfficialScene::from_body(&template, source)
    }
}

pub fn export_native_simulation(
    profile: &BodyExecutionProfile,
    source: &dyn AssetResolver,
) -> Result<Vec<u8>, String> {
    export(profile, source, None)
}

/// Preserve the recorded effective parameters while verifying every resource against
/// this release. Recorded profiles contain dynamic strings and are kept as JSON.
pub fn export_recorded_native_simulation(
    profile: &serde_json::Value,
    source: &dyn AssetResolver,
) -> Result<Vec<u8>, String> {
    let simulation = &profile["simulation"];
    let scenario = &simulation["scenario"];
    if profile["format_version"] != 1
        || profile["execution_kind"] != "simulation"
        || !scenario["task_id"].is_null()
    {
        return Err("Expected a recorded native simulation profile".into());
    }
    let scene = scenario["scene_xml"]
        .as_str()
        .ok_or("Missing recorded scene XML")?;
    let robot = scenario["robot_xml"]
        .as_str()
        .ok_or("Missing recorded robot XML")?;
    let expected: Vec<SourceAsset> = serde_json::from_value(simulation["source_assets"].clone())
        .map_err(|error| error.to_string())?;
    let roots: Vec<_> = expected
        .iter()
        .filter(|asset| asset.path.ends_with(".xml"))
        .map(|asset| asset.path.as_str())
        .collect();
    if !roots.contains(&scene) || !roots.contains(&robot) {
        return Err("Recorded resources omit scene or robot XML".into());
    }
    let resources = AssetBundle::read(source, &roots).map_err(|error| error.to_string())?;
    if resources.manifest() != expected {
        return Err("Recorded simulation resources differ from this release".into());
    }
    let assets: Vec<_> = resources
        .files()
        .map(|(path, bytes)| AssetBytes {
            path,
            base64: STANDARD.encode(bytes),
        })
        .collect();
    serde_json::to_vec(&serde_json::json!({
        "format_version": 1,
        "units": {"angle":"rad","angular_velocity":"rad/s","length":"m","time":"s","voltage":"V"},
        "profile": profile, "task": null, "assets": assets,
    }))
    .map_err(|error| error.to_string())
}

fn export(
    profile: &BodyExecutionProfile,
    source: &dyn AssetResolver,
    task: Option<&TaskBodyDescription>,
) -> Result<Vec<u8>, String> {
    let simulation = profile
        .simulation
        .as_ref()
        .ok_or("expected a simulation profile")?;
    if simulation.scenario.task_id.as_deref() != task.map(|task| task.task_id.as_str()) {
        return Err("task description does not match the resolved simulation".into());
    }
    let assets = simulation
        .scenario
        .resources(source)
        .map_err(|error| error.to_string())?;
    if assets.manifest() != simulation.source_assets {
        return Err("resolved resources changed before simulation profile export".into());
    }
    serde_json::to_vec(&SimulationExport {
        format_version: if task.is_some() { 2 } else { 1 },
        units: Units {
            angle: "rad",
            angular_velocity: "rad/s",
            length: "m",
            time: "s",
            voltage: "V",
        },
        profile,
        task,
        backend: task
            .map(|_| super::mjlab::resolve_mjlab_body(profile))
            .transpose()?,
        assets: assets
            .files()
            .map(|(path, bytes)| AssetBytes {
                path,
                base64: STANDARD.encode(bytes),
            })
            .collect(),
    })
    .map_err(|error| error.to_string())
}
