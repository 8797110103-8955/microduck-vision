//! Serialize a resolved simulator profile and its exact assets for Python.

use base64::{Engine as _, engine::general_purpose::STANDARD};
use duck_body::{AssetResolver, BodyProfile};
use serde::Serialize;

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
        format_version: 1,
        units: Units {
            angle: "rad",
            angular_velocity: "rad/s",
            length: "m",
            time: "s",
            voltage: "V",
        },
        profile,
        task,
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
