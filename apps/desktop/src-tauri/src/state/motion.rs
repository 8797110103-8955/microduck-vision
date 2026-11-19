//! Physical world-state cache. Without an odometry provider the world remains unknown at origin.
//! Simulation owns and publishes its world state directly from MuJoCo.

use crate::types::WorldState;

#[derive(Debug, Clone, Copy)]
pub(crate) struct MotionTracker {
    world: WorldState,
}

impl Default for MotionTracker {
    fn default() -> Self {
        Self {
            world: WorldState {
                position: [0.0, 0.0],
                yaw: 0.0,
                distance_m: 0.0,
            },
        }
    }
}

impl MotionTracker {
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn world(&self) -> WorldState {
        self.world
    }
}
