//! Body definitions and display resources for the desktop product.

use std::sync::OnceLock;

use duck_body::{BodyProfile, OfficialScene};
use microduck_body_runtime::models::assets::{OfficialAssets, display_scene, load_official_body};

pub(crate) fn official() -> Result<&'static BodyProfile, String> {
    static PROFILE: OnceLock<Result<BodyProfile, String>> = OnceLock::new();
    PROFILE
        .get_or_init(|| {
            load_official_body(&OfficialAssets::Embedded).map_err(|error| error.to_string())
        })
        .as_ref()
        .map_err(Clone::clone)
}

#[cfg(test)]
pub(crate) fn official_template() -> Result<duck_body::BodyTemplate, String> {
    Ok(official()?.template().clone())
}

pub(crate) fn official_scene() -> Result<&'static OfficialScene, String> {
    static SCENE: OnceLock<Result<OfficialScene, String>> = OnceLock::new();
    SCENE
        .get_or_init(|| {
            display_scene(official()?, &OfficialAssets::Embedded).map_err(|error| error.to_string())
        })
        .as_ref()
        .map_err(Clone::clone)
}
