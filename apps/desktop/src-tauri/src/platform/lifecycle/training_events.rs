use tauri::{AppHandle, Emitter, Wry};

use crate::types::{TrainingFrame, TrainingProgress};

pub(crate) fn progress(app: &AppHandle<Wry>, progress: TrainingProgress) {
    let _ = app.emit("training-progress", progress);
}

pub(crate) fn frame(app: &AppHandle<Wry>, frame: TrainingFrame) {
    let _ = app.emit("training-frame", frame);
}
