//! Application startup, background resource registration, and shutdown ownership.
//!
//! Lifecycle code may initialize state and watchers, but it does not run policy ticks or expose
//! frontend command handlers.

pub(crate) mod device_watcher;
pub(crate) use device_watcher::DeviceWatcher;

use crate::control;
use crate::state::DesktopState;
use tauri::{App, AppHandle, Manager, RunEvent, Wry};

/// Installs resources that live for the entire desktop application lifetime. Loop ownership and
/// command dispatch stay in their own modules; lifecycle owns only startup wiring and resource
/// registration.
pub(crate) fn setup(app: &mut App<Wry>) -> Result<(), Box<dyn std::error::Error>> {
    let config_dir = app.path().app_config_dir()?;
    let state = DesktopState::load(config_dir);
    state.set_app_handle(app.handle().clone());
    app.manage(state);
    app.manage(DeviceWatcher::start(app.handle().clone()));
    Ok(())
}

/// Performs the application-level shutdown handoff.  Tauri owns the event loop, while this
/// module owns the order in which control loops and preview resources are released.
pub(crate) fn handle_event(app: &AppHandle<Wry>, event: RunEvent) {
    if !matches!(event, RunEvent::Exit) {
        return;
    }
    let state = app.state::<DesktopState>();
    let _ = control::close(state.inner());
    if let Ok(mut preview) = state.runtime_loop.lock() {
        *preview = None;
    }
}
