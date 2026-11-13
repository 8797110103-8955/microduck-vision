use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum ServoError {
    #[error("servo implementation is not wired: {0}")]
    NotImplemented(&'static str),
    #[error("unknown register: {0}")]
    UnknownRegister(String),
    #[error("register {name} cannot encode value {value}")]
    RegisterOutOfRange { name: String, value: i64 },
    #[error("invalid register bytes for {name}")]
    InvalidRegister { name: String },
    #[error("bus error: {0}")]
    Bus(String),
    #[error("{what}: expected {expected} bytes, got {got}")]
    ShortRead {
        what: &'static str,
        expected: usize,
        got: usize,
    },
    #[error("servo {0} is missing")]
    MissingServo(u8),
    #[error("position is not finite: {0}")]
    NonFinitePosition(f64),
    #[error("{what}: expected {expected} values, got {got}")]
    CountMismatch {
        what: &'static str,
        expected: usize,
        got: usize,
    },
}
