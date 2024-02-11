#[cfg(feature = "target-riscv")]
pub mod riscv;

pub(crate) enum Message {
    DeviceInterrupt(u32),
}
