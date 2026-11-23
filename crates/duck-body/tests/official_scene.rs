use super::assets;
use duck_body::{BodyError, BodyTemplate, FileAssets, official_assets};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

#[test]
fn vendor_kinematics_fixture_has_the_official_sites_and_ranges() {
    let model = kinematics::Model::alpha();
    assert_eq!(model.num_joints(), 14);
    for joint in ["left_hip_yaw", "left_knee", "neck_pitch", "right_ankle"] {
        let index = model.joint_index(joint).expect("vendor joint");
        let (lo, hi) = model.joint_range(index).expect("vendor range");
        assert!(lo < hi);
    }
    for site in ["left_foot", "right_foot", "head_camera", "tof"] {
        assert!(model.site(site).is_some(), "missing vendor site {site}");
    }
}

#[test]
fn display_and_physics_assets_are_vendor_fixtures() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let path = body.assets.display_mjcf;
    assert!(
        path.ends_with("vendor/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml")
    );
}

#[test]
fn official_mjcf_mass_and_inertia_are_loaded_in_grams() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    let leg = body
        .physical
        .links
        .iter()
        .find(|link| link.name == "leg")
        .unwrap();
    assert!((leg.mass_g - 21.5844).abs() < 0.001);
    assert!(leg.inertia.is_some());
    assert!(body.physical.mass_g.unwrap() > 0.0);
}

#[test]
fn official_asset_validation_uses_the_vendor_path() {
    let body = BodyTemplate::official(assets(), &FileAssets).unwrap();
    assert!(body.validate_assets(&FileAssets).is_ok());
}

#[test]
fn official_display_uses_the_same_resources_in_memory() {
    use duck_body::{BodyProfile, OfficialScene};

    let files = assets();
    let root = files.display_mjcf.parent().unwrap();
    let mut memory: BTreeMap<PathBuf, Vec<u8>> =
        ["robot_walk.xml", "robot_groundcontact.xml", "scene.xml"]
            .into_iter()
            .map(|name| (PathBuf::from(name), std::fs::read(root.join(name)).unwrap()))
            .collect();
    for entry in std::fs::read_dir(root.join("assets")).unwrap() {
        let entry = entry.unwrap();
        if entry.file_type().unwrap().is_file() {
            memory.insert(
                Path::new("assets").join(entry.file_name()),
                std::fs::read(entry.path()).unwrap(),
            );
        }
    }
    let ids = official_assets(Path::new(""));
    let on_disk = BodyProfile::official(files, &FileAssets).unwrap();
    let in_memory = BodyProfile::official(ids.clone(), &memory).unwrap();
    assert_eq!(
        on_disk.compatibility_snapshot(),
        in_memory.compatibility_snapshot()
    );
    let file_scene = OfficialScene::for_official_body(on_disk.template(), &FileAssets).unwrap();
    let memory_scene = OfficialScene::for_official_body(in_memory.template(), &memory).unwrap();
    assert_eq!(file_scene.model(), memory_scene.model());
    let pose = &on_disk.template().control.home_pose;
    assert_eq!(
        file_scene.frame(pose).unwrap(),
        memory_scene.frame(pose).unwrap()
    );

    let include = PathBuf::from("robot_groundcontact.xml");
    let robot = memory.remove(&include).unwrap();
    assert_eq!(
        BodyProfile::official(ids.clone(), &memory).unwrap_err(),
        BodyError::MissingAsset(include.clone())
    );
    memory.insert(include, robot);
    let mesh = PathBuf::from("assets/right_shell.stl");
    memory.remove(&mesh).unwrap();
    assert_eq!(
        BodyProfile::official(ids, &memory).unwrap_err(),
        BodyError::MissingAsset(mesh)
    );
}
