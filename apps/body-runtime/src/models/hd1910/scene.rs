use crate::models::scene_recipe::SceneRecipe;

pub(crate) fn scene() -> SceneRecipe {
    SceneRecipe::native("scene.xml", "robot_groundcontact.xml")
}
