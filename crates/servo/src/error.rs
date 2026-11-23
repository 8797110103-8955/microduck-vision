use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum ServoError {
    #[error("backend gain is not configurable")]
    UnsupportedGain,
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
    #[error("不支持的位置配置 mode={mode} resolution={resolution}")]
    UnsupportedPositionMode { mode: u8, resolution: u8 },
    #[error("无效的位置限位 {min}..{max}")]
    InvalidPositionLimits { min: u16, max: u16 },
    #[error(
        "位置 count={count} 超出硬件限位 {min}..{max} (mode={mode})；请核对编码器圈数与校准，不能对目标直接取模"
    )]
    PositionOutsideHardwareLimits {
        min: u16,
        max: u16,
        count: i32,
        mode: u8,
    },
    #[error("{what}: expected {expected} values, got {got}")]
    CountMismatch {
        what: &'static str,
        expected: usize,
        got: usize,
    },
}
