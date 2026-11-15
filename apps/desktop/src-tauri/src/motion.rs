//! Application-level world motion tracking.
//!
//! The body loop deliberately knows nothing about world coordinates. This tracker consumes the
//! command selected by the client and can later be fed measured odometry from a real robot. The
//! current fallback is a command-integrated estimate, which is suitable for preview and clearly
//! separate from joint control.

use std::time::Duration;

use crate::types::WorldState;

#[derive(Debug, Clone, Copy)]
pub(crate) struct MotionTracker {
    world: WorldState,
    command: [f64; 3],
    estimate_from_command: bool,
}

impl Default for MotionTracker {
    fn default() -> Self {
        Self {
            world: WorldState {
                position: [0.0, 0.0],
                yaw: 0.0,
                distance_m: 0.0,
            },
            command: [0.0; 3],
            estimate_from_command: true,
        }
    }
}

impl MotionTracker {
    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    pub(crate) fn set_command(&mut self, command: [f64; 3]) {
        self.command = command;
    }

    pub(crate) fn set_estimate_from_command(&mut self, enabled: bool) {
        self.estimate_from_command = enabled;
    }

    pub(crate) fn tick(&mut self, driving: bool, dt: Duration) -> WorldState {
        if driving && self.estimate_from_command {
            let seconds = dt.as_secs_f64();
            self.world.position[0] += self.command[0] * seconds;
            self.world.position[1] += self.command[1] * seconds;
            self.world.yaw += self.command[2] * seconds;
            self.world.distance_m += self.command[0].hypot(self.command[1]) * seconds;
        }
        self.world
    }

    pub(crate) fn world(&self) -> WorldState {
        self.world
    }
}

#[cfg(test)]
mod tests {
    use super::MotionTracker;
    use std::time::Duration;

    #[test]
    fn simulation_estimate_accumulates_command_motion() {
        let mut tracker = MotionTracker::default();
        tracker.set_command([0.5, 0.25, 1.0]);
        let world = tracker.tick(true, Duration::from_secs(2));
        assert_eq!(world.position, [1.0, 0.5]);
        assert_eq!(world.yaw, 2.0);
        assert!((world.distance_m - (0.5_f64.hypot(0.25) * 2.0)).abs() < 1e-12);
    }

    #[test]
    fn physical_tracker_does_not_claim_command_motion_as_odometry() {
        let mut tracker = MotionTracker::default();
        tracker.set_estimate_from_command(false);
        tracker.set_command([0.5, 0.25, 1.0]);
        assert_eq!(
            tracker.tick(true, Duration::from_secs(2)).position,
            [0.0, 0.0]
        );
    }
}
