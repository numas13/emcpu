pub mod riscv;

pub enum Message {
    DeviceInterrupt(u32),
}
