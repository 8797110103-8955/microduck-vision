//! Scene placement is independent of the simulation's temporary physics origin.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::types::{SimulationState, WorldState};

#[derive(Default)]
pub(crate) struct WorldCoordinates {
    previous: Option<[f64; 3]>,
}

impl WorldCoordinates {
    pub(crate) fn observe(
        &mut self,
        world: &mut WorldState,
        state: &SimulationState,
        driven: bool,
    ) {
        let [w, x, y, z] = state.trunk_quaternion;
        let pose = [
            state.trunk_position[0],
            state.trunk_position[1],
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

/// Retain each duck's placement across body replacements and loop restarts. Disk writes
/// happen at explicit reset/stop boundaries, never in the control tick.
pub(crate) struct WorldPlacements {
    values: Mutex<HashMap<String, WorldState>>,
    path: Option<PathBuf>,
}

impl WorldPlacements {
    pub(crate) fn load(config: Option<&Path>) -> Result<Self, String> {
        let path = config.map(|config| config.join("world-placements.json"));
        let values = match path.as_ref().map(std::fs::read).transpose() {
            Ok(Some(bytes)) => serde_json::from_slice(&bytes).map_err(|error| error.to_string())?,
            Ok(None) => HashMap::new(),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => HashMap::new(),
            Err(error) => {
                return Err(format!(
                    "read world placements {}: {error}",
                    path.as_ref().expect("read path exists").display()
                ));
            }
        };
        Ok(Self {
            values: Mutex::new(values),
            path,
        })
    }

    pub(crate) fn get(&self, id: &str) -> Option<WorldState> {
        self.values.lock().ok()?.get(id).copied()
    }

    pub(crate) fn remember(&self, id: &str, world: WorldState) {
        if let Ok(mut values) = self.values.lock() {
            values.insert(id.to_owned(), world);
        }
    }

    pub(crate) fn flush(&self) -> Result<(), String> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        let values = self.values.lock().map_err(|error| error.to_string())?;
        let bytes = serde_json::to_vec_pretty(&*values).map_err(|error| error.to_string())?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|error| error.to_string())?;
        }
        let temporary = path.with_extension("json.tmp");
        std::fs::write(&temporary, bytes).map_err(|error| error.to_string())?;
        std::fs::rename(temporary, path).map_err(|error| error.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

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

    #[test]
    fn placements_survive_reload_and_remain_scoped_to_each_duck() {
        let root = crate::helpers::test_directory().join(format!(
            "world-placement-test-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let registry = WorldPlacements::load(Some(&root)).unwrap();
        registry.remember("duck-a", placement());
        registry.remember(
            "duck-b",
            WorldState {
                position: [-1.0, 8.0],
                ..placement()
            },
        );
        registry.flush().unwrap();
        registry.flush().unwrap();
        let reloaded = WorldPlacements::load(Some(&root)).unwrap();
        assert_eq!(reloaded.get("duck-a").unwrap().position, [2.0, 3.0]);
        assert_eq!(reloaded.get("duck-b").unwrap().position, [-1.0, 8.0]);
        assert!(reloaded.get("duck-c").is_none());
        std::fs::remove_dir_all(root).unwrap();
    }
}
