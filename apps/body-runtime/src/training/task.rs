//! Resolve and validate the robot body requested by a training task.

use std::path::Path;

use duck_body::{AssetResolver, BodyProfile};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::models::profile::{
    BodyExecutionProfile, BodyPeriods, ModelKind, Selection, SimulationTuning,
};
use crate::models::resolve::resolve_simulation_profile;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TaskBodyDescription {
    pub task_id: String,
    pub robot_xml: String,
    pub robot_sha256: String,
    pub joint_order: Vec<String>,
    pub robot_joints: Vec<Option<String>>,
    pub home_pose_rad: Vec<f64>,
    pub root_home_pose: [f64; 7],
    pub periods: BodyPeriods,
    pub requires_backlash: bool,
    pub name_servo_collision_geoms: bool,
    pub target_delay_steps: [usize; 2],
    pub policy_layout: serde_json::Value,
    pub assembly: serde_json::Value,
}

impl TaskBodyDescription {
    pub(crate) fn validate(
        &self,
        body: &BodyProfile,
        source: &dyn AssetResolver,
        model: ModelKind,
    ) -> Result<(), String> {
        let template = body.template();
        if self.task_id.trim().is_empty()
            || !self.robot_xml.starts_with("robot_")
            || !self.robot_xml.ends_with(".xml")
            || self.robot_xml.contains(['/', '\\'])
        {
            return Err("training task must identify an official robot XML".into());
        }
        let robot = source
            .read(Path::new(&self.robot_xml))
            .map_err(|error| error.to_string())?;
        if format!("{:x}", Sha256::digest(&robot)) != self.robot_sha256 {
            return Err(format!(
                "task robot source digest changed: {}",
                self.robot_xml
            ));
        }
        if self.joint_order != template.control.joint_order
            || self.robot_joints.len() != template.joints.len()
            || self.robot_joints.iter().map(Option::as_deref).ne(template
                .joints
                .iter()
                .map(|joint| joint.scene_joint.as_deref()))
            || self.home_pose_rad.len() != self.joint_order.len()
            || self.home_pose_rad.iter().any(|value| !value.is_finite())
            || self.root_home_pose.iter().any(|value| !value.is_finite())
        {
            return Err("training task body joint mapping or home pose is invalid".into());
        }
        let text = std::str::from_utf8(&robot).map_err(|error| error.to_string())?;
        let doc = roxmltree::Document::parse(text).map_err(|error| error.to_string())?;
        let named_joints = doc
            .descendants()
            .filter(|node| node.has_tag_name("joint"))
            .filter_map(|node| node.attribute("name"))
            .collect::<std::collections::HashSet<_>>();
        if self
            .robot_joints
            .iter()
            .flatten()
            .any(|name| !named_joints.contains(name.as_str()))
        {
            return Err("task joint mapping does not match the official robot XML".into());
        }
        let passive = self
            .robot_joints
            .iter()
            .flatten()
            .filter(|name| named_joints.contains(format!("passive_{name}_backlash").as_str()))
            .count();
        let expected = self.robot_joints.iter().flatten().count();
        if (self.requires_backlash && passive != expected)
            || (!self.requires_backlash && passive != 0)
        {
            return Err("task backlash requirement does not match the official robot XML".into());
        }
        if self.requires_backlash && model != ModelKind::Xl330 {
            return Err(format!(
                "task {} requires backlash hinges and feedback that {model:?} does not supply",
                self.task_id
            ));
        }
        if self.name_servo_collision_geoms != self.robot_xml.starts_with("robot_allcollisions") {
            return Err("task collision naming does not match the official robot variant".into());
        }
        let (Some(physics_s), Some(physics_substeps)) =
            (self.periods.physics_s, self.periods.physics_substeps)
        else {
            return Err("training task must specify a physics period and decimation".into());
        };
        let control = physics_s * f64::from(physics_substeps);
        if !physics_s.is_finite()
            || physics_s <= 0.0
            || physics_substeps == 0
            || !self.periods.control_s.is_finite()
            || (self.periods.control_s - control).abs() > 1e-12
            || self.target_delay_steps[0] > self.target_delay_steps[1]
            || !self.policy_layout.is_object()
            || !self.assembly.is_object()
        {
            return Err("training task periods, delay, or policy layout are invalid".into());
        }
        Ok(())
    }
}

pub struct ResolvedTrainingBody {
    pub profile: BodyExecutionProfile,
    pub task: TaskBodyDescription,
}

pub fn resolve_training_body(
    selection: Selection,
    body: &BodyProfile,
    source: &dyn AssetResolver,
    tuning: SimulationTuning,
    task: TaskBodyDescription,
) -> Result<ResolvedTrainingBody, String> {
    task.validate(body, source, selection.model_kind)?;
    let profile = resolve_simulation_profile(selection, body, source, tuning, Some(&task))?;
    Ok(ResolvedTrainingBody { profile, task })
}
