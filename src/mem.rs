use std::mem;
use std::sync::Arc;

use crate::device::Device;

pub struct DeviceId(usize);

#[derive(Debug)]
pub enum Error {
    AccessFault,
}

macro_rules! impl_read {
    ($($n:ident: $t:ty = $f:ident),+ $(,)?) => (
        $(fn $n(&mut self, address: usize) -> Result<$t, Error> {
            Ok(<$t>::$f(self.read_array::<{ mem::size_of::<$t>() }>(address)?))
        })+
    );
}

macro_rules! impl_write {
    ($($n:ident: $t:ty = $f:ident),+ $(,)?) => (
        $(fn $n(&mut self, address: usize, value: $t) -> Result<(), Error> {
            self.write_array(address, value.$f())
        })+
    );
}

pub trait Memory {
    fn read(&mut self, address: usize, data: &mut [u8]) -> Result<(), Error>;

    fn write(&mut self, address: usize, data: &[u8]) -> Result<(), Error>;

    fn register_device(&mut self, device: Arc<dyn Device>) -> DeviceId;

    fn get_device_mmio_base(&self, id: DeviceId) -> usize;

    fn read_array<const N: usize>(&mut self, address: usize) -> Result<[u8; N], Error> {
        let mut buf = [0; N];
        self.read(address, &mut buf)?;
        Ok(buf)
    }

    fn write_array<const N: usize>(&mut self, address: usize, data: [u8; N]) -> Result<(), Error> {
        self.write(address, &data)
    }

    fn read_u8(&mut self, address: usize) -> Result<u8, Error> {
        Ok(self.read_array::<1>(address)?[0])
    }

    fn read_i8(&mut self, address: usize) -> Result<i8, Error> {
        Ok(self.read_u8(address)? as i8)
    }

    fn write_u8(&mut self, address: usize, value: u8) -> Result<(), Error> {
        self.write(address, &[value])
    }

    fn write_i8(&mut self, address: usize, value: i8) -> Result<(), Error> {
        self.write(address, &[value as u8])
    }

    #[rustfmt::skip]
    impl_read! {
        read_u16_le:   u16 = from_le_bytes,
        read_u32_le:   u32 = from_le_bytes,
        read_u64_le:   u64 = from_le_bytes,
        read_u128_le: u128 = from_le_bytes,
        read_i16_le:   u16 = from_le_bytes,
        read_i32_le:   u32 = from_le_bytes,
        read_i64_le:   u64 = from_le_bytes,
        read_i128_le: u128 = from_le_bytes,

        read_u16_be:   i16 = from_be_bytes,
        read_u32_be:   i32 = from_be_bytes,
        read_u64_be:   i64 = from_be_bytes,
        read_u128_be: i128 = from_be_bytes,
        read_i16_be:   i16 = from_be_bytes,
        read_i32_be:   i32 = from_be_bytes,
        read_i64_be:   i64 = from_be_bytes,
        read_i128_be: i128 = from_be_bytes,
    }

    #[rustfmt::skip]
    impl_write! {
        write_u16_le:   u16 = to_le_bytes,
        write_u32_le:   u32 = to_le_bytes,
        write_u64_le:   u64 = to_le_bytes,
        write_u128_le: u128 = to_le_bytes,
        write_i16_le:   i16 = to_le_bytes,
        write_i32_le:   i32 = to_le_bytes,
        write_i64_le:   i64 = to_le_bytes,
        write_i128_le: i128 = to_le_bytes,

        write_u16_be:   u16 = to_be_bytes,
        write_u32_be:   u32 = to_be_bytes,
        write_u64_be:   u64 = to_be_bytes,
        write_u128_be: u128 = to_be_bytes,
        write_i16_be:   i16 = to_be_bytes,
        write_i32_be:   i32 = to_be_bytes,
        write_i64_be:   i64 = to_be_bytes,
        write_i128_be: i128 = to_be_bytes,
    }
}

pub struct SimpleMem {
    mem: Vec<u8>,
    devices: Vec<Arc<dyn Device>>,
}

impl SimpleMem {
    pub const PAGE_SIZE: usize = 4096;
    pub const MMIO_BASE: usize = 0x80000000;

    pub fn new(size: usize) -> Self {
        Self {
            mem: vec![0; size],
            devices: Default::default(),
        }
    }

    pub fn is_mmio_range(&self, address: usize) -> bool {
        (Self::MMIO_BASE..Self::MMIO_BASE + self.devices.len() * Self::PAGE_SIZE).contains(&address)
    }
}

impl Memory for SimpleMem {
    fn read(&mut self, address: usize, data: &mut [u8]) -> Result<(), Error> {
        if data.len() == 4 && self.is_mmio_range(address) {
            let i = (address - Self::MMIO_BASE) / Self::PAGE_SIZE;
            let offset = (address & (Self::PAGE_SIZE - 1)) as u32;
            let value = self.devices[i].read_u32(offset);
            data.copy_from_slice(&value.to_le_bytes());
            return Ok(());
        }
        if (address + data.len()) > self.mem.len() {
            return Err(Error::AccessFault);
        }
        data.copy_from_slice(&self.mem[address..address + data.len()]);
        Ok(())
    }

    fn write(&mut self, address: usize, data: &[u8]) -> Result<(), Error> {
        if data.len() == 4 && self.is_mmio_range(address) {
            let i = (address - Self::MMIO_BASE) / Self::PAGE_SIZE;
            let offset = (address & (Self::PAGE_SIZE - 1)) as u32;
            let mut buf = [0; 4];
            buf.copy_from_slice(data);
            let value = u32::from_le_bytes(buf);
            self.devices[i].write_u32(offset, value);
            return Ok(());
        }
        if (address + data.len()) > self.mem.len() {
            return Err(Error::AccessFault);
        }
        self.mem[address..address + data.len()].copy_from_slice(data);
        Ok(())
    }

    fn register_device(&mut self, device: Arc<dyn Device>) -> DeviceId {
        let id = DeviceId(self.devices.len());
        self.devices.push(device);
        id
    }

    fn get_device_mmio_base(&self, id: DeviceId) -> usize {
        Self::MMIO_BASE + id.0 * Self::PAGE_SIZE
    }
}
