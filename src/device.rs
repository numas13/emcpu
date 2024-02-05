use std::io::{Read, Write};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;

use crate::target::Message;

pub struct DeviceInterrupt {
    tx: mpsc::SyncSender<Message>,
    id: u32,
}

impl DeviceInterrupt {
    pub fn new(tx: mpsc::SyncSender<Message>, id: u32) -> Self {
        Self { tx, id }
    }

    pub fn interrupt(&self) {
        trace!("device-interrupt({}) cpu", self.id);
        self.tx.send(Message::DeviceInterrupt(self.id)).unwrap();
    }
}

pub trait Device {
    #[allow(unused_variables)]
    fn read_u32(&self, offset: u32) -> u32 {
        0
    }

    #[allow(unused_variables)]
    fn write_u32(&self, offset: u32, value: u32) {}
}

struct Buffer {
    pos: usize,
    len: usize,
    data: [u8; 64],
}

impl Default for Buffer {
    fn default() -> Self {
        Self {
            pos: 0,
            len: 0,
            data: [0; 64],
        }
    }
}

pub struct TerminalDevice {
    buffer: Mutex<Buffer>,
    read_tx: mpsc::SyncSender<()>,
}

impl TerminalDevice {
    pub fn new(interrupt: DeviceInterrupt) -> Arc<Self> {
        let (read_tx, read_rx) = mpsc::sync_channel(0);
        let ret = Arc::new(Self {
            buffer: Default::default(),
            read_tx,
        });

        let dev = ret.clone();
        thread::spawn(move || loop {
            dev.fill_buffer(&interrupt);
            read_rx.recv().unwrap();
        });

        ret
    }

    fn fill_buffer(&self, interrupt: &DeviceInterrupt) {
        trace!("terminal-device: fill buffer");
        let stdin = std::io::stdin();
        let mut handle = stdin.lock();
        let mut buffer = self.buffer.lock().unwrap();
        buffer.pos = 0;
        buffer.len = 0;
        match handle.read(&mut buffer.data) {
            Ok(len) => {
                buffer.len = len;
                trace!("terminal-device: interrupt cpu");
                interrupt.interrupt();
            }
            Err(e) => {
                trace!("terminal-device: stdin error: {e}");
            }
        }
    }
}

impl Device for TerminalDevice {
    fn read_u32(&self, _offset: u32) -> u32 {
        let mut buffer = self.buffer.lock().unwrap();
        if buffer.pos < buffer.len {
            let b = buffer.data[buffer.pos];
            trace!("terminal-device: read {}", b as char);
            buffer.pos += 1;
            b as u32
        } else {
            self.read_tx.send(()).unwrap();
            0
        }
    }

    fn write_u32(&self, _offset: u32, value: u32) {
        let stdout = std::io::stdout();
        let mut handle = stdout.lock();
        handle.write_all(&[value as u8]).unwrap();
        handle.flush().unwrap();
    }
}
