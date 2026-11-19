//! Backend control boundaries. Transport, register access, and loop drivers are kept in
//! separate modules while this facade preserves the existing crate-local API.

mod driver;
mod registers;
mod transport;

pub(crate) use registers::RegisterDescriptor;
pub(crate) use transport::{
    ConnectedBus, move_pending_bus, open_for_model, try_open_feetech, try_open_xl330,
};

#[cfg(test)]
mod tests {
    use super::registers::is_timeout_error;
    use super::transport::move_pending_bus;
    use std::collections::HashMap;
    use std::sync::Mutex;

    #[test]
    fn moving_pending_bus_releases_the_lock_before_reinserting() {
        let buses = Mutex::new(HashMap::from([("pending:COM7".to_owned(), 7_u8)]));

        move_pending_bus(&buses, Some("pending:COM7"), "physical-1").unwrap();

        let buses = buses.lock().unwrap();
        assert!(!buses.contains_key("pending:COM7"));
        assert_eq!(buses.get("physical-1"), Some(&7_u8));
    }

    #[test]
    fn only_timeout_errors_are_tolerated_for_final_eeprom_lock() {
        assert!(is_timeout_error(
            "EEPROM lock: bus error: Operation timed out"
        ));
        assert!(is_timeout_error("serial timeout"));
        assert!(!is_timeout_error("EEPROM lock: checksum mismatch"));
    }
}
