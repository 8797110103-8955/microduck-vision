//! Persist scene placement by virtual machine ID.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use crate::types::WorldState;

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
    use std::f64::consts::FRAC_PI_2;

    fn placement() -> WorldState {
        WorldState {
            position: [2.0, 3.0],
            yaw: FRAC_PI_2,
            distance_m: 4.0,
        }
    }

    #[test]
    fn placements_survive_reload_and_remain_scoped_to_each_duck() {
        let root = crate::platform::helpers::test_directory().join(format!(
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
