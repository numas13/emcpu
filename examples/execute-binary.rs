#[macro_use]
extern crate log;

#[cfg(feature = "target-riscv")]
fn main() {
    use std::{env, fs};

    use emcpu::device;
    use emcpu::mem::{Memory, SimpleMem};
    use emcpu::target::riscv::Cpu;

    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let args: Vec<_> = env::args().collect();
    let path = args.get(1).map(String::as_str).unwrap_or("a.bin");
    let bin = fs::read(path).unwrap();

    let mem = SimpleMem::new(1024_usize.pow(2));
    let mut cpu = Cpu::new(0, mem);

    cpu.mem.write(0, &bin).unwrap();
    let dev = device::TerminalDevice::new(cpu.device_interrupt(0));
    let id = cpu.mem.register_device(dev);
    let base = cpu.mem.get_device_mmio_base(id);
    info!("terminal MMIO base is {base:#x}");

    cpu.run();
}

#[cfg(not(feature = "target-riscv"))]
fn main() {
    eprintln!("`target-riscv` feature must be enabled");
}
