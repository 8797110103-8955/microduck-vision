//! Location of the official display meshes served through the desktop asset protocol.

use std::path::PathBuf;

pub(crate) fn resource_directory() -> Result<PathBuf, String> {
    let executable = std::env::current_exe().map_err(|error| error.to_string())?;
    let parent = executable
        .parent()
        .ok_or_else(|| "missing executable directory".to_owned())?;
    // Test binaries live in the profile's deps directory.
    [Some(parent), parent.parent()]
        .into_iter()
        .flatten()
        .map(|directory| directory.join("resources/microduck"))
        .find(|directory| directory.is_dir())
        .ok_or_else(|| "official MuJoCo model assets are missing beside the application".to_owned())
}

#[cfg(test)]
pub(crate) fn test_profile() -> duck_body::BodyProfile {
    use microduck_body_runtime::models::assets::{OfficialAssets, load_official_body};
    load_official_body(&OfficialAssets::Embedded).unwrap()
}

#[cfg(test)]
pub(crate) fn test_template() -> duck_body::BodyTemplate {
    test_profile().template().clone()
}
