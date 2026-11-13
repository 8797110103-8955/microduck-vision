pub(crate) mod support {
    use std::collections::VecDeque;
    use std::io::{self, Cursor, Read, Write};
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    pub(crate) struct ScriptedPort {
        answers: VecDeque<Vec<u8>>,
        incoming: Cursor<Vec<u8>>,
        written: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl ScriptedPort {
        pub(crate) fn new(answers: Vec<Vec<u8>>) -> (Self, Arc<Mutex<Vec<Vec<u8>>>>) {
            let written = Arc::new(Mutex::new(Vec::new()));
            (
                Self {
                    answers: answers.into(),
                    incoming: Cursor::new(Vec::new()),
                    written: Arc::clone(&written),
                },
                written,
            )
        }
    }

    impl Read for ScriptedPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.incoming.read(buf)
        }
    }

    impl Write for ScriptedPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.written.lock().unwrap().push(buf.to_vec());
            self.incoming = Cursor::new(self.answers.pop_front().unwrap_or_default());
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl serialport::SerialPort for ScriptedPort {
        fn bytes_to_read(&self) -> serialport::Result<u32> {
            Ok(0)
        }

        fn bytes_to_write(&self) -> serialport::Result<u32> {
            Ok(0)
        }

        fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
            Ok(())
        }

        fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
            Err(serialport::Error::new(
                serialport::ErrorKind::Unknown,
                "scripted port cannot be cloned",
            ))
        }

        fn timeout(&self) -> Duration {
            Duration::from_millis(10)
        }

        fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
            Ok(())
        }

        fn name(&self) -> Option<String> {
            Some("scripted".into())
        }

        fn baud_rate(&self) -> serialport::Result<u32> {
            Ok(1_000_000)
        }

        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
            Ok(())
        }

        fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
            Ok(serialport::DataBits::Eight)
        }

        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
            Ok(serialport::FlowControl::None)
        }

        fn parity(&self) -> serialport::Result<serialport::Parity> {
            Ok(serialport::Parity::None)
        }

        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
            Ok(serialport::StopBits::One)
        }

        fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
            Ok(())
        }

        fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
            Ok(())
        }

        fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
            Ok(())
        }

        fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
            Ok(())
        }

        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
            Ok(())
        }

        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
            Ok(())
        }

        fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
            Ok(true)
        }

        fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
            Ok(true)
        }

        fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
            Ok(false)
        }

        fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
            Ok(true)
        }

        fn set_break(&self) -> serialport::Result<()> {
            Ok(())
        }

        fn clear_break(&self) -> serialport::Result<()> {
            Ok(())
        }
    }
}

#[path = "xl330.rs"]
mod xl330;

#[path = "hd1910.rs"]
mod hd1910;

#[path = "servo_contract.rs"]
mod servo_contract;
