//! Phase/lifecycle predicates kept out of adapter and policy code.

use crate::LoopPhase;

pub(crate) fn torque_enabled(phase: LoopPhase) -> bool {
    matches!(
        phase,
        LoopPhase::Homing | LoopPhase::Ready | LoopPhase::Driving | LoopPhase::Holding
    )
}

pub(crate) fn reads_sensors(phase: LoopPhase) -> bool {
    !matches!(
        phase,
        LoopPhase::Limp | LoopPhase::Stopped | LoopPhase::Estop
    )
}
