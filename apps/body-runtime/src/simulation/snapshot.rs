//! Native simulation snapshots in the canonical body frame.

use std::sync::{Arc, Mutex};

use duck_body::SceneFrame;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SimulationSnapshot {
    pub physics_step: u64,
    pub time: f64,
    pub trunk_position: [f64; 3],
    /// World quaternion in w, x, y, z order.
    pub trunk_quaternion: [f64; 4],
    pub motor_positions: Vec<f64>,
    pub passive_positions: Vec<f64>,
    /// Freejoint velocity comprises world linear velocity and local angular velocity.
    pub base_velocity: [f64; 6],
    pub contacts: Vec<[i32; 2]>,
    pub motor_torques: Vec<f64>,
    pub friction_budgets: Vec<f64>,
    pub effective_voltage: f64,
    pub frame: SceneFrame,
}

#[derive(Default)]
pub struct SharedSnapshot {
    pub state: Option<SimulationSnapshot>,
}

pub type SnapshotHandle = Arc<Mutex<SharedSnapshot>>;
