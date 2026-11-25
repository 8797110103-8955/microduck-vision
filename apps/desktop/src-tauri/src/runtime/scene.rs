//! Read-only scene queries for the desktop application.

use crate::types::{
    JointRange, SceneBody as FrontendSceneBody, SceneFrame as FrontendSceneFrame,
    SceneGeom as FrontendSceneGeom, SceneInfo as FrontendSceneInfo,
    SceneJoint as FrontendSceneJoint, SceneModel as FrontendSceneModel,
    SceneSite as FrontendSceneSite,
};

pub(crate) fn official_joint_slots() -> Vec<String> {
    duck_control::model::JOINT_NAMES
        .iter()
        .map(|name| (*name).to_owned())
        .collect()
}

pub(crate) fn official_joint_ranges() -> Result<Vec<JointRange>, String> {
    let template = crate::runtime::machines::body_catalog::official()?.template();
    Ok(template
        .actuators
        .iter()
        .map(|slot| {
            let range = template
                .joint_range(&slot.joint)
                .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI]);
            JointRange {
                slot_id: slot.id.clone(),
                min_rad: range[0],
                max_rad: range[1],
            }
        })
        .collect())
}

pub(crate) fn scene_model() -> Result<FrontendSceneInfo, String> {
    let scene = crate::runtime::machines::body_catalog::official_scene()?;
    let template = crate::runtime::machines::body_catalog::official()?.template();
    project_scene(scene, &template.control.home_pose)
}

pub(crate) fn project_scene(
    scene: &duck_body::OfficialScene,
    home: &[f64],
) -> Result<FrontendSceneInfo, String> {
    let model = scene.model();
    let default_frame = scene.frame(home).map_err(|error| error.to_string())?;
    Ok(FrontendSceneInfo {
        model: FrontendSceneModel {
            bodies: model
                .bodies
                .iter()
                .map(|body| FrontendSceneBody {
                    name: body.name.clone(),
                    geoms: body
                        .geoms
                        .iter()
                        .map(|geom| FrontendSceneGeom {
                            mesh: geom.mesh.clone(),
                            pos: geom.pos,
                            quat: geom.quat,
                            rgba: geom.rgba,
                        })
                        .collect(),
                })
                .collect(),
            joints: model
                .joints
                .iter()
                .map(|joint| FrontendSceneJoint {
                    name: joint.name.clone(),
                    body: joint.body,
                })
                .collect(),
            trunk_height: model.trunk_height,
            head_camera: model.head_camera.as_ref().map(|site| FrontendSceneSite {
                body: site.body,
                pos: site.pos,
            }),
        },
        mesh_dir: crate::runtime::adapters::body_assets::resource_directory()?
            .join(scene.mesh_dir())
            .to_string_lossy()
            .into_owned(),
        default_frame: FrontendSceneFrame {
            bodies: default_frame.bodies,
            markers: default_frame.markers,
        },
    })
}

pub(crate) fn scene_frame(positions: &[f64]) -> Result<FrontendSceneFrame, String> {
    let scene = crate::runtime::machines::body_catalog::official_scene()?;
    let frame = scene.frame(positions).map_err(|error| error.to_string())?;
    Ok(FrontendSceneFrame {
        bodies: frame.bodies,
        markers: frame.markers,
    })
}

use crate::types::WorldState;

#[derive(Default)]
pub(crate) struct WorldCoordinates {
    previous: Option<[f64; 3]>,
}

impl WorldCoordinates {
    pub(crate) fn observe(
        &mut self,
        world: &mut WorldState,
        [position_x, position_y, _]: [f64; 3],
        [w, x, y, z]: [f64; 4],
        driven: bool,
    ) {
        let pose = [
            position_x,
            position_y,
            (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z)),
        ];
        self.advance(world, pose, driven);
    }

    fn advance(&mut self, world: &mut WorldState, pose: [f64; 3], driven: bool) {
        if let Some(previous) = self.previous.replace(pose)
            && driven
        {
            // Reopening a body starts a new physics origin. Continue in the duck's saved
            // scene heading, including when it resumes after a turn or an idle interval.
            let (sin, cos) = (world.yaw - previous[2]).sin_cos();
            let dx = pose[0] - previous[0];
            let dy = pose[1] - previous[1];
            world.position[0] += cos * dx - sin * dy;
            world.position[1] += sin * dx + cos * dy;
            world.distance_m += dx.hypot(dy);
            world.yaw += angle_delta(pose[2] - previous[2]);
            world.yaw = angle_delta(world.yaw);
        }
    }
}

fn angle_delta(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn scene_projection_serves_deployed_meshes_with_embedded_xml() {
        let scene = scene_model().expect("official display scene");
        let meshes = std::path::Path::new(&scene.mesh_dir);
        for geom in scene.model.bodies.iter().flat_map(|body| &body.geoms) {
            assert!(meshes.join(&geom.mesh).is_file(), "missing {}", geom.mesh);
        }
        let deployed = meshes.parent().expect("deployed model asset directory");
        assert!(!deployed.join("scene.xml").exists());
        assert!(!deployed.join("scene_backlash.xml").exists());
    }

    fn placement() -> WorldState {
        WorldState {
            position: [2.0, 3.0],
            yaw: FRAC_PI_2,
            distance_m: 4.0,
        }
    }

    #[test]
    fn idle_drift_is_excluded_and_a_new_origin_preserves_scene_heading() {
        let mut world = placement();
        let mut coordinates = WorldCoordinates::default();
        coordinates.advance(&mut world, [8.0, -3.0, 0.0], false);
        coordinates.advance(&mut world, [8.2, -3.1, 0.3], false);
        assert_eq!(world.position, [2.0, 3.0]);
        assert_eq!(world.yaw, FRAC_PI_2);
        assert_eq!(world.distance_m, 4.0);

        // Resume motion after idle rotation: one local metre forward is one scene metre
        // along the saved heading. The idle drift must never appear on the first tick.
        coordinates.advance(
            &mut world,
            [8.2 + 0.3_f64.cos(), -3.1 + 0.3_f64.sin(), 0.3],
            true,
        );
        assert!((world.position[0] - 2.0).abs() < 1e-12);
        assert!((world.position[1] - 4.0).abs() < 1e-12);
        assert!((world.distance_m - 5.0).abs() < 1e-12);

        // Reopening a body resets its physics, rather than the duck's scene placement.
        let mut reopened = WorldCoordinates::default();
        reopened.advance(&mut world, [0.0; 3], false);
        reopened.advance(&mut world, [0.5, 0.0, 0.0], true);
        assert!((world.position[0] - 2.0).abs() < 1e-12);
        assert!((world.position[1] - 4.5).abs() < 1e-12);
    }

    #[test]
    fn crossing_the_yaw_boundary_does_not_reverse_or_add_a_full_turn() {
        let mut world = WorldState {
            yaw: PI - 0.01,
            ..placement()
        };
        let mut coordinates = WorldCoordinates::default();
        coordinates.advance(&mut world, [0.0, 0.0, PI - 0.01], false);
        coordinates.advance(&mut world, [0.0, 0.0, -PI + 0.01], true);
        assert!((world.yaw - (-PI + 0.01)).abs() < 1e-12);
        assert_eq!(world.distance_m, 4.0);
    }
}
