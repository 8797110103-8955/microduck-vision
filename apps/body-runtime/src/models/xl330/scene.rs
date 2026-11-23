use crate::models::scene_recipe::SceneRecipe;

pub(crate) fn scene() -> SceneRecipe {
    SceneRecipe::native("scene_backlash.xml", "robot_groundcontact_backlash.xml")
}
