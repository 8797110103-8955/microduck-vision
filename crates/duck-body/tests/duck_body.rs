use duck_body::{BodyAssets, BodyTemplate, FileAssets, ServoParameterSet, official_assets};
use std::path::Path;

mod body_template;
mod dynamics;
mod official_scene;
mod servo_binding;

fn assets() -> BodyAssets {
    official_assets(
        &Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../vendor/microduck_rl/src/mjlab_microduck/robot/microduck"),
    )
}

fn hd_parameters() -> ServoParameterSet {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    ServoParameterSet::all_hd1910(&body)
}
