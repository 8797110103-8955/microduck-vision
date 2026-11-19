//! Physical-machine identity is derived from a complete servo census.
//!
//! Serial-port metadata is deliberately kept separate from this module's identity
//! functions.  It is only a hint for finding a port again after a disconnect.

use std::collections::BTreeSet;

use md5::{Digest, Md5};

use crate::types::{
    DetectedPhysicalMachine, DiagnosticCode, MatchResult, PhysicalMachine, ReconnectHint,
    ServoIdentity,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CensusError {
    Empty,
    DuplicateId(u8),
    UnknownModel(String),
}

impl std::fmt::Display for CensusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => write!(f, "census is empty"),
            Self::DuplicateId(id) => write!(f, "census contains duplicate servo id {id}"),
            Self::UnknownModel(model) => write!(f, "unknown servo model {model:?}"),
        }
    }
}

/// Normalize, validate and sort a census.  The returned value is the only form
/// used for persistence and matching.
pub fn canonicalize_census(census: &[ServoIdentity]) -> Result<Vec<ServoIdentity>, CensusError> {
    if census.is_empty() {
        return Err(CensusError::Empty);
    }
    let mut ids = BTreeSet::new();
    let mut result = Vec::with_capacity(census.len());
    for servo in census {
        if !ids.insert(servo.servo_id) {
            return Err(CensusError::DuplicateId(servo.servo_id));
        }
        let model = normalize_model(&servo.model)?;
        result.push(ServoIdentity {
            servo_id: servo.servo_id,
            model,
        });
    }
    result.sort_by_key(|servo| servo.servo_id);
    Ok(result)
}

pub fn normalize_model(model: &str) -> Result<String, CensusError> {
    let normalized = model.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "hd1910" | "xl330" => Ok(normalized),
        _ => Err(CensusError::UnknownModel(model.trim().to_owned())),
    }
}

/// Calculate the lowercase MD5 required by the PRD.  USB data and names never
/// enter this input.
pub fn census_md5(census: &[ServoIdentity]) -> Result<String, CensusError> {
    let canonical = canonicalize_census(census)?;
    let bytes = canonical
        .iter()
        .map(|servo| format!("{}:{}", servo.servo_id, servo.model))
        .collect::<Vec<_>>()
        .join("\0");
    Ok(format!("{:x}", Md5::digest(bytes.as_bytes())))
}

pub fn same_census(a: &[ServoIdentity], b: &[ServoIdentity]) -> bool {
    match (canonicalize_census(a), canonicalize_census(b)) {
        (Ok(a), Ok(b)) => a == b,
        _ => false,
    }
}

/// Match only a complete census.  Duplicate runtime MD5s are an identity
/// conflict and must never be resolved with recency or USB metadata.
pub fn match_saved(
    saved: &[PhysicalMachine],
    census: &[ServoIdentity],
) -> Result<(String, MatchResult), CensusError> {
    let md5 = census_md5(census)?;
    let matches: Vec<&PhysicalMachine> = saved
        .iter()
        .filter(|machine| same_census(&machine.census, census))
        .collect();
    let result = match matches.as_slice() {
        [] => MatchResult::New,
        [machine] => MatchResult::Unique {
            physical_machine_id: machine.id.clone(),
        },
        _ => MatchResult::IdentityConflict,
    };
    Ok((md5, result))
}

pub fn detect(
    saved: &[PhysicalMachine],
    census: Vec<ServoIdentity>,
    reconnect_hint: Option<ReconnectHint>,
) -> DetectedPhysicalMachine {
    let canonical = canonicalize_census(&census);
    match canonical {
        Ok(census) => {
            let (md5, match_result) =
                match_saved(saved, &census).expect("canonical census must hash");
            let diagnostics = matches!(match_result, MatchResult::IdentityConflict)
                .then_some(vec![DiagnosticCode::IdentityConflict])
                .unwrap_or_default();
            DetectedPhysicalMachine {
                census,
                md5: Some(md5),
                reconnect_hint,
                match_result,
                diagnostics,
            }
        }
        Err(error) => DetectedPhysicalMachine {
            census,
            md5: None,
            reconnect_hint,
            match_result: MatchResult::IncompleteCensus,
            diagnostics: vec![match error {
                CensusError::Empty => DiagnosticCode::IncompleteCensus,
                CensusError::DuplicateId(_) | CensusError::UnknownModel(_) => {
                    DiagnosticCode::IncompleteCensus
                }
            }],
        },
    }
}

/// Build a failed probe result without collapsing transport diagnostics into a string error.
/// A failed probe never creates or matches a physical machine.
pub fn failed_probe(
    reconnect_hint: Option<ReconnectHint>,
    mut diagnostics: Vec<DiagnosticCode>,
) -> DetectedPhysicalMachine {
    if !diagnostics.contains(&DiagnosticCode::IncompleteCensus) {
        diagnostics.push(DiagnosticCode::IncompleteCensus);
    }
    DetectedPhysicalMachine {
        census: Vec::new(),
        md5: None,
        reconnect_hint,
        match_result: MatchResult::IncompleteCensus,
        diagnostics,
    }
}

pub fn reconnect_candidates<'a>(
    hint: &ReconnectHint,
    ports: &'a [ReconnectHint],
) -> Vec<&'a ReconnectHint> {
    // Machines created before USB metadata was available may only have a port
    // name.  Prefer that exact name instead of treating every metadata-less
    // serial port as interchangeable.
    if hint.vid.is_none() && hint.pid.is_none() {
        return ports
            .iter()
            .filter(|port| port.port_name == hint.port_name)
            .collect();
    }
    if let Some(serial) = hint.adapter_serial.as_deref() {
        return ports
            .iter()
            .filter(|port| {
                port.vid == hint.vid
                    && port.pid == hint.pid
                    && port.adapter_serial.as_deref() == Some(serial)
            })
            .collect();
    }
    let same_kind: Vec<&ReconnectHint> = ports
        .iter()
        .filter(|port| port.vid == hint.vid && port.pid == hint.pid)
        .collect();
    if let Some(port) = same_kind
        .iter()
        .find(|port| port.port_name == hint.port_name)
    {
        return vec![*port];
    }
    if same_kind.len() == 1 {
        same_kind
    } else {
        Vec::new()
    }
}

/// Whether a unique, already-saved identity is eligible for startup mounting.
/// A newly detected candidate intentionally fails this check until it is saved.
pub fn can_auto_mount(
    detected: &DetectedPhysicalMachine,
    machine: &PhysicalMachine,
    startup_virtual_machine_id: Option<&str>,
    strategy_enabled: bool,
) -> bool {
    strategy_enabled
        && startup_virtual_machine_id.is_some()
        && machine.connected
        && matches!(
            &detected.match_result,
            MatchResult::Unique { physical_machine_id } if physical_machine_id == &machine.id
        )
        && machine
            .virtual_machine_id
            .as_deref()
            .is_none_or(|id| Some(id) == startup_virtual_machine_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn servo(id: u8, model: &str) -> ServoIdentity {
        ServoIdentity {
            servo_id: id,
            model: model.into(),
        }
    }

    #[test]
    fn md5_is_order_and_case_stable() {
        assert_eq!(
            census_md5(&[servo(2, " XL330 "), servo(1, "HD1910")]),
            census_md5(&[servo(1, "hd1910"), servo(2, "xl330")])
        );
    }

    #[test]
    fn md5_changes_for_id_model_or_count() {
        let base = census_md5(&[servo(1, "hd1910")]).unwrap();
        assert_ne!(base, census_md5(&[servo(2, "hd1910")]).unwrap());
        assert_ne!(base, census_md5(&[servo(1, "xl330")]).unwrap());
        assert_ne!(
            base,
            census_md5(&[servo(1, "hd1910"), servo(2, "hd1910")]).unwrap()
        );
    }

    #[test]
    fn invalid_census_has_no_md5() {
        assert_eq!(census_md5(&[]), Err(CensusError::Empty));
        assert!(matches!(
            census_md5(&[servo(1, "unknown")]),
            Err(CensusError::UnknownModel(_))
        ));
        assert_eq!(
            census_md5(&[servo(1, "hd1910"), servo(1, "hd1910")]),
            Err(CensusError::DuplicateId(1))
        );
    }

    #[test]
    fn reconnect_uses_serial_or_a_unique_same_kind_port() {
        let hint = ReconnectHint {
            port_name: "COM3".into(),
            vid: Some(1),
            pid: Some(2),
            adapter_serial: Some("SN".into()),
            product: None,
        };
        let moved = ReconnectHint {
            port_name: "COM7".into(),
            vid: Some(1),
            pid: Some(2),
            adapter_serial: Some("SN".into()),
            product: None,
        };
        assert_eq!(reconnect_candidates(&hint, &[moved]).len(), 1);
        let no_serial = ReconnectHint {
            adapter_serial: None,
            ..hint
        };
        let a = ReconnectHint {
            port_name: "COM8".into(),
            ..no_serial.clone()
        };
        let b = ReconnectHint {
            port_name: "COM9".into(),
            ..no_serial.clone()
        };
        assert!(reconnect_candidates(&no_serial, &[a, b]).is_empty());
    }

    #[test]
    fn reconnect_without_usb_metadata_uses_the_saved_port_name() {
        let hint = ReconnectHint {
            port_name: "COM3".into(),
            vid: None,
            pid: None,
            adapter_serial: None,
            product: None,
        };
        let same = ReconnectHint {
            port_name: "COM3".into(),
            ..hint.clone()
        };
        let other = ReconnectHint {
            port_name: "COM4".into(),
            ..hint.clone()
        };
        assert_eq!(reconnect_candidates(&hint, &[same, other]).len(), 1);
    }

    #[test]
    fn auto_mount_requires_saved_unique_match_and_strategy() {
        let machine = PhysicalMachine {
            id: "physical-1".into(),
            name: "duck".into(),
            usb_port: "COM1".into(),
            machine_code: String::new(),
            census: vec![servo(1, "hd1910")],
            reconnect_hint: None,
            virtual_machine_id: None,
            online_servo_ids: vec![1],
            connected: true,
            last_seen: None,
            torque_enabled: false,
            joint_directions: Vec::new(),
        };
        let detected = DetectedPhysicalMachine {
            census: machine.census.clone(),
            md5: census_md5(&machine.census).ok(),
            reconnect_hint: None,
            match_result: MatchResult::Unique {
                physical_machine_id: machine.id.clone(),
            },
            diagnostics: Vec::new(),
        };
        assert!(can_auto_mount(&detected, &machine, Some("virtual-1"), true));
        assert!(!can_auto_mount(
            &detected,
            &machine,
            Some("virtual-1"),
            false
        ));
        let mut stale = machine.clone();
        stale.connected = false;
        assert!(!can_auto_mount(&detected, &stale, Some("virtual-1"), true));
    }

    #[test]
    fn failed_probe_is_incomplete_and_never_has_md5() {
        let result = failed_probe(None, vec![DiagnosticCode::OpenFailed]);
        assert_eq!(result.match_result, MatchResult::IncompleteCensus);
        assert_eq!(result.md5, None);
        assert!(
            result
                .diagnostics
                .contains(&DiagnosticCode::IncompleteCensus)
        );
    }
}
