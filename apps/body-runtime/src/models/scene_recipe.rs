//! Scene and robot sources are distinct, including when the robot is a scene include.

use duck_body::{AssetResolver, BodyError};
use serde::Serialize;

use crate::models::assets::AssetBundle;

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SceneOperation {
    ConfigureBamActuators,
    MatchXl330PositionStiffness,
    NameServoCollisionGeoms,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SceneRecipe {
    pub scene_xml: Option<&'static str>,
    pub robot_xml: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub task_id: Option<String>,
    pub derived_scene_operations: Vec<SceneOperation>,
}

impl SceneRecipe {
    pub(crate) fn native(scene_xml: &'static str, robot_xml: &'static str) -> Self {
        Self {
            scene_xml: Some(scene_xml),
            robot_xml: robot_xml.to_owned(),
            task_id: None,
            derived_scene_operations: vec![SceneOperation::ConfigureBamActuators],
        }
    }

    pub fn resources<'a>(
        &self,
        source: &'a dyn AssetResolver,
    ) -> Result<AssetBundle<'a>, BodyError> {
        match self.scene_xml {
            Some(scene_xml) => AssetBundle::read(
                source,
                &[scene_xml, &self.robot_xml, "robot_walk.xml", "scene.xml"],
            ),
            None => AssetBundle::read(source, &[&self.robot_xml]),
        }
    }
}
