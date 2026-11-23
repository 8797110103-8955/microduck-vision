//! Non-motion HLS calibration transactions. No target position or torque-on writes are allowed.

use super::servo_bus::ConnectedBus;
use serde::{Deserialize, Serialize};
use servo::hd1910::FeetechRawBus;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub(crate) struct EncoderSnapshot {
    pub firmware: [u8; 2],
    pub model: [u8; 2],
    pub mode: u8,
    pub phase: u8,
    pub resolution: u8,
    pub limits: [u16; 2],
    pub offset_raw: u16,
    pub lock: u8,
    pub torque: u8,
    pub position_count: i32,
    pub velocity_count: i16,
}

impl EncoderSnapshot {
    fn from_bytes(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() != 60 {
            return Err("HD1910 calibration snapshot requires 60 bytes".into());
        }
        let word = |index| u16::from_le_bytes([bytes[index], bytes[index + 1]]);
        Ok(Self {
            firmware: [bytes[0], bytes[1]],
            model: [bytes[3], bytes[4]],
            mode: bytes[33],
            phase: bytes[18],
            resolution: bytes[30],
            limits: [word(9), word(11)],
            offset_raw: word(31),
            lock: bytes[55],
            torque: bytes[40],
            position_count: servo::hd1910::decode_register("present_position", &bytes[56..58])
                .map_err(|e| e.to_string())? as i32,
            velocity_count: servo::hd1910::decode_register("present_velocity", &bytes[58..60])
                .map_err(|e| e.to_string())? as i16,
        })
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.model != [10, 31] || self.firmware < [3, 43] {
            return Err("编码器中位校准要求 HD1910 固件 ≥ 3.43".into());
        }
        if !matches!(self.mode, 0 | 4) || self.resolution != 1 || self.limits != [0, 4095] {
            return Err("编码器中位校准要求位置模式、分辨率 1 及完整单圈限位 0..4095；自定义限位或多圈配置需单独标定".into());
        }
        if self.torque != 0 || self.lock > 1 {
            return Err("编码器校准前必须确认力矩关闭及有效的 EEPROM 锁状态".into());
        }
        Ok(())
    }
}

pub(crate) trait EncoderBus {
    fn torque_off(&mut self, ids: &[u8]) -> Result<(), String>;
    fn snapshot(&mut self, id: u8) -> Result<EncoderSnapshot, String>;
    fn center(&mut self, id: u8) -> Result<(), String>;
    fn restore_offset(&mut self, id: u8, offset: u16, lock: u8) -> Result<(), String>;
    fn settle(&mut self) {
        std::thread::sleep(std::time::Duration::from_millis(80));
    }
}

impl EncoderBus for ConnectedBus {
    fn torque_off(&mut self, ids: &[u8]) -> Result<(), String> {
        self.set_torque(servo::ServoModel::Hd1910, ids, false)
    }
    fn snapshot(&mut self, id: u8) -> Result<EncoderSnapshot, String> {
        let Self::Hd1910(bus) = self else {
            return Err("编码器中位校准仅支持 HD1910".into());
        };
        EncoderSnapshot::from_bytes(&bus.read_raw(id, 0, 60).map_err(|e| e.to_string())?)
    }
    fn center(&mut self, id: u8) -> Result<(), String> {
        let Self::Hd1910(bus) = self else {
            return Err("编码器中位校准仅支持 HD1910".into());
        };
        bus.calibrate_position_center(id).map_err(|e| e.to_string())
    }
    fn restore_offset(&mut self, id: u8, offset: u16, lock: u8) -> Result<(), String> {
        let Self::Hd1910(bus) = self else {
            return Err("编码器中位校准仅支持 HD1910".into());
        };
        bus.write_raw(id, 55, &[0]).map_err(|e| e.to_string())?;
        let restore = bus
            .write_raw(id, 31, &offset.to_le_bytes())
            .map_err(|e| e.to_string());
        let relock = bus.write_raw(id, 55, &[lock]).map_err(|e| e.to_string());
        match (restore, relock) {
            (Ok(()), Ok(())) => Ok(()),
            (a, b) => Err(format!("恢复位置偏移：{a:?}；恢复 EEPROM 锁：{b:?}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn snapshot_preserves_full_turn_and_offset_sign_bits_and_checks_hls_capabilities() {
        let mut bytes = [0u8; 60];
        bytes[0] = 3;
        bytes[1] = 46;
        bytes[3] = 10;
        bytes[4] = 31;
        bytes[11..13].copy_from_slice(&4095u16.to_le_bytes());
        bytes[18] = 52;
        bytes[30] = 1;
        bytes[31..33].copy_from_slice(&0x87deu16.to_le_bytes());
        bytes[33] = 4;
        bytes[55] = 1;
        bytes[56..58].copy_from_slice(&4130u16.to_le_bytes());
        bytes[58..60].copy_from_slice(&0x8001u16.to_le_bytes());
        let snapshot = EncoderSnapshot::from_bytes(&bytes).unwrap();
        assert_eq!(snapshot.position_count, 4130);
        assert_eq!(snapshot.offset_raw, 0x87de);
        assert_eq!(snapshot.velocity_count, -1);
        snapshot.validate().unwrap();
        assert!(
            EncoderSnapshot {
                torque: 1,
                ..snapshot.clone()
            }
            .validate()
            .is_err()
        );
        assert!(
            EncoderSnapshot {
                limits: [0, 0],
                ..snapshot.clone()
            }
            .validate()
            .is_err()
        );
        assert!(
            EncoderSnapshot {
                resolution: 2,
                ..snapshot
            }
            .validate()
            .is_err()
        );
        assert!(EncoderSnapshot::from_bytes(&bytes[..59]).is_err());
    }
}
