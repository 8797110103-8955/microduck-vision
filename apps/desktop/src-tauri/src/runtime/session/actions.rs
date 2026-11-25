//! Application service for the user action catalog.

use crate::runtime::session::state::DesktopState;
use crate::types::{ActionInput, ActionKind, ActionRecord, ActionSource};

pub(crate) fn list(state: &DesktopState) -> Result<Vec<ActionRecord>, String> {
    state
        .actions
        .lock()
        .map(|actions| actions.clone())
        .map_err(|error| error.to_string())
}

pub(crate) fn create(state: &DesktopState, input: ActionInput) -> Result<ActionRecord, String> {
    let name = input.name.trim();
    if name.is_empty() {
        return Err("action name cannot be empty".into());
    }
    let kind = match input.kind.as_str() {
        "pose" => ActionKind::Pose,
        "sequence" => ActionKind::Sequence,
        "policy" => ActionKind::Policy,
        _ => return Err("unknown action kind".into()),
    };
    let mut actions = state.actions.lock().map_err(|error| error.to_string())?;
    let action = ActionRecord {
        id: format!("action-{}", actions.len() + 1),
        name: name.to_string(),
        kind,
        updated_at: "刚刚".into(),
        source: ActionSource::User,
    };
    actions.push(action.clone());
    Ok(action)
}

pub(crate) fn delete(state: &DesktopState, id: String) -> Result<(), String> {
    let mut actions = state.actions.lock().map_err(|error| error.to_string())?;
    if actions
        .iter()
        .any(|action| action.id == id && action.source == ActionSource::Builtin)
    {
        return Err("built-in action cannot be deleted".into());
    }
    actions.retain(|action| action.id != id);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{create, delete, list};
    use crate::runtime::session::state::DesktopState;
    use crate::types::ActionInput;

    #[test]
    fn create_trims_names_and_persists_a_user_action() {
        let state = DesktopState::new(None);
        let action = create(
            &state,
            ActionInput {
                name: "  wave  ".into(),
                kind: "sequence".into(),
            },
        )
        .expect("user action should be created");

        assert_eq!(action.id, "action-3");
        assert_eq!(action.name, "wave");
        assert_eq!(
            list(&state)
                .expect("actions should be readable")
                .last()
                .unwrap()
                .id,
            action.id
        );
    }

    #[test]
    fn built_in_actions_are_protected_but_user_actions_can_be_deleted() {
        let state = DesktopState::new(None);
        assert_eq!(
            delete(&state, "stand".into()).unwrap_err(),
            "built-in action cannot be deleted"
        );
        let action = create(
            &state,
            ActionInput {
                name: "wave".into(),
                kind: "pose".into(),
            },
        )
        .expect("user action should be created");
        delete(&state, action.id.clone()).expect("user action should be deletable");
        assert!(
            !list(&state)
                .expect("actions should be readable")
                .iter()
                .any(|entry| entry.id == action.id)
        );
    }
}
