use crate::models::scene_recipe::{SceneOperation, SceneRecipe};

pub(crate) fn scene() -> SceneRecipe {
    let mut scene = crate::models::hd1910::scene::scene();
    scene
        .derived_scene_operations
        .push(SceneOperation::MatchXl330PositionStiffness);
    scene
}
