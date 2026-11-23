//! Publish runtime data to the desktop window.

use std::sync::Mutex;
use tauri::{AppHandle, Emitter, Wry};

use crate::types::{JointRangeState, LoopStatus};

pub(crate) fn emit_joint_range_states(
    handle: &Mutex<Option<AppHandle<Wry>>>,
    states: &[JointRangeState],
) {
    if let Ok(handle) = handle.lock()
        && let Some(handle) = handle.as_ref()
    {
        let _ = handle.emit("joint-range-state", states);
    }
}

pub(crate) fn emit_loop_status(handle: &Mutex<Option<AppHandle<Wry>>>, status: LoopStatus) {
    if let Ok(handle) = handle.lock()
        && let Some(handle) = handle.as_ref()
    {
        let _ = handle.emit("loop-status", status);
    }
}
