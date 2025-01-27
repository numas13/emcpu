mod decode {
    #![allow(dead_code)]

    use crate::utils::{sextract, zextract};

    include!(concat!(env!("OUT_DIR"), "/target/riscv/insn16.rs"));
    include!(concat!(env!("OUT_DIR"), "/target/riscv/insn32.rs"));
}

use std::mem;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crate::device::DeviceInterrupt;
use crate::mem::{Error as MemoryError, Memory};
use crate::target::Message;
use crate::utils::deposit;

use self::decode::*;

const EXCP_INSN_ADDR_MISALIGNED: u64 = 0;
const EXCP_INSN_ACCESS_FAULT: u64 = 1;
const EXCP_ILLEGAL_INSN: u64 = 2;
const EXCP_BREAKPOINT: u64 = 3;
//const EXCP_LOAD_ADDR_MISALIGNED: u64 = 4;
const EXCP_LOAD_ACCESS_FAULT: u64 = 5;
//const EXCP_STORE_AMO_ADDR_MISALIGNED: u64 = 6;
const EXCP_STORE_AMO_ACCESS_FAULT: u64 = 7;
//const EXCP_ECALL_UMODE: u64 = 8;
//const EXCP_ECALL_SMODE: u64 = 9;
//const EXCP_ECALL_MMODE: u64 = 11;
//const EXCP_INSN_PAGE_FAULT: u64 = 12;
//const EXCP_LOAD_PAGE_FAULT: u64 = 13;
//const EXCP_STORE_AMO_PAGE_FAULT: u64 = 15;

const fn make_interrupt(code: u64) -> u64 {
    (1 << (mem::size_of::<u64>() * 8 - 1)) | code
}

//const EXCP_MACHINE_EXTERNAL_INT: u64 = make_interrupt(11);

// Unprivileged Floating-Point CSRs
//const CSR_FFLAGS: u32 = 0x001;
//const CSR_FRM: u32 = 0x002;
//const CSR_FCSR: u32 = 0x003;

// Unprivileged Counter/Timers
const CSR_CYCLE: u32 = 0xc00;
const CSR_TIME: u32 = 0xc01;
const CSR_INSTRET: u32 = 0xc02;
const CSR_HPMCOUNTER0: u32 = 0xc00;
const CSR_HPMCOUNTER31: u32 = 0xc1f;
//const CSR_CYCLEH: u32 = 0xc80;
//const CSR_TIMEH: u32 = 0xc81;
//const CSR_INSTRETH: u32 = 0xc82;
//const CSR_HPMCOUNTER0H: u32 = 0xc80;
//const CSR_HPMCOUNTER31H: u32 = 0xc9f;

// Machine Information Registers
const CSR_MVENDOR_ID: u32 = 0xf11;
const CSR_MARCH_ID: u32 = 0xf12;
const CSR_MIMP_ID: u32 = 0xf13;
const CSR_MHART_ID: u32 = 0xf14;
const CSR_MCONFIG_PTR: u32 = 0xf15;

// Machine Trap Setup
const CSR_MSTATUS: u32 = 0x300;
const CSR_MISA: u32 = 0x301;
const CSR_MEDELEG: u32 = 0x302;
const CSR_MIDELEG: u32 = 0x303;
const CSR_MIE: u32 = 0x304;
const CSR_MTVEC: u32 = 0x305;
//const CSR_MCOUTNEREN: u32 = 0x306;
//const CSR_MSTATUSH: u32 = 0x310;

// Machine Trap Handling
const CSR_MSCRATCH: u32 = 0x340;
const CSR_MEPC: u32 = 0x341;
const CSR_MCAUSE: u32 = 0x342;
const CSR_MTVAL: u32 = 0x343;
const CSR_MIP: u32 = 0x344;
const CSR_MINST: u32 = 0x34a;
//const CSR_MTVAL2: u32 = 0x34b;

// Machine Configuration
//const CSR_MENV_CFG: u32 = 0x30a;
//const CSR_MENV_CFGH: u32 = 0x31a;
//const CSR_MSEC_CFG: u32 = 0x747;
//const CSR_MSEC_CFGH: u32 = 0x757;

// Machine Memory Protection
//const CSR_PMP_CFG_0: u32 = 0x3a0;
//const CSR_PMP_CFG_15: u32 = 0x3af;
//const CSR_PMP_ADDR_0: u32 = 0x3b0;
//const CSR_PMP_ADDR_63: u32 = 0x3ef;

// Machine Non-Maskable Interrupt Handling
//const CSR_MNSCRATCH: u32 = 0x740;
//const CSR_MNEPC: u32 = 0x741;
//const CSR_MNCAUSE: u32 = 0x742;
//const CSR_MNSTATUS: u32 = 0x744;

// Machine Counter/Timers
const CSR_MCYCLE: u32 = 0xb00;
const CSR_MCYCLEH: u32 = 0xb80;
const CSR_MINSTRET: u32 = 0xb02;
//const CSR_MINSTRETH: u32 = 0xb82;
const CSR_MHPMCOUNTER0: u32 = 0xb00;
const CSR_MHPMCOUNTER31: u32 = 0xb1f;
//const CSR_MHPMCOUNTER0H: u32 = 0xb80;
//const CSR_MHPMCOUNTER31H: u32 = 0xb9f;

// Machine Counter Setup
const CSR_MCOUNTINHIBIT: u32 = 0x320;
const CSR_MHPM_EVENT0: u32 = 0x320;
const CSR_MHPM_EVENT31: u32 = 0x33f;

// Debug/Trace Registers (shared with Debug Mode)
//const CSR_TSELECT: u32 = 0x7a0;
//const CSR_TDATA1: u32 = 0x7a1;
//const CSR_TDATA2: u32 = 0x7a2;
//const CSR_TDATA3: u32 = 0x7a3;
//const CSR_MCONTEXT: u32 = 0x7a8;

// Debug Mode Registers
//const CSR_DCSR: u32 = 0x7b0;
//const CSR_DPC: u32 = 0x7b1;
//const CSR_SCRATCH0: u32 = 0x7b2;
//const CSR_SCRATCH1: u32 = 0x7b3;

const fn misa_ext(c: u8) -> u64 {
    match c {
        b'a'..=b'z' => 1 << (c - b'a'),
        b'A'..=b'A' => 1 << (c - b'A'),
        _ => 0,
    }
}

const MISA_EXT_C: u64 = misa_ext(b'c');
const MISA_EXT_M: u64 = misa_ext(b'm');
const MISA_EXT_I: u64 = misa_ext(b'i');
const MISA_EXT_X: u64 = misa_ext(b'x');
const MISA_DEFAULT: u64 = MISA_EXT_C | MISA_EXT_M | MISA_EXT_I | MISA_EXT_X;
const MISA_EXT_MASK: u64 = MISA_DEFAULT ^ MISA_EXT_I;

const MSTATUS_MIE_OFFSET: u32 = 3;
const MSTATUS_MIE: u64 = 1 << MSTATUS_MIE_OFFSET;
const MSTATUS_MPIE_OFFSET: u32 = 7;
const MSTATUS_MPIE: u64 = 1 << MSTATUS_MPIE_OFFSET;
const MSTATUS_MASK: u64 = MSTATUS_MIE;

const MIE_MASK: u64 = 0xffffffff_ffff0800;
const MTVEC_MASK: u64 = 0xffffffff_fffffffd;

//const MIE_SS: u32 = 1 << 1;
//const MIE_MS: u32 = 1 << 3;
//const MIE_ST: u32 = 1 << 5;
//const MIE_MT: u32 = 1 << 7;
//const MIE_SE: u32 = 1 << 9;
//const MIE_ME: u32 = 1 << 11;

const FREQUENCY: u64 = 1000;
const SLEEP_TIME: Duration = Duration::from_nanos(1_000_000_000 / FREQUENCY);
const PAUSE_TIME: Duration = Duration::from_secs(1);

pub struct Cpu<M> {
    pub mem: M,

    cpu_messages_tx: mpsc::SyncSender<Message>,
    cpu_messages_rx: mpsc::Receiver<Message>,

    pc: u64,
    npc: u64,
    gr: [u64; 32],

    mstatus: u64,
    misa: u64,
    mip: u64,
    mie: u64,
    mcause: u64,
    mepc: u64,
    mtvec: u64,
    mscratch: u64,
    mtval: u64,
    minst: u32,

    minstret: u64,
}

impl<M> Cpu<M> {
    pub fn new(pc: u64, mem: M) -> Self {
        let (cpu_messages_tx, cpu_messages_rx) = mpsc::sync_channel(32);

        Self {
            pc,
            npc: 0,
            gr: [0; 32],
            mem,
            cpu_messages_rx,
            cpu_messages_tx,

            mstatus: 0,
            misa: MISA_DEFAULT,
            mip: 0,
            mie: 0,
            mcause: 0,
            mepc: 0,
            mtvec: 0,
            mscratch: 0,
            mtval: 0,
            minst: 0,

            minstret: 0,
        }
    }

    pub fn device_interrupt(&mut self, id: u32) -> DeviceInterrupt {
        let id = id + 16;
        assert!(id < mem::size_of_val(&self.mie) as u32 * 8);
        DeviceInterrupt::new(self.cpu_messages_tx.clone(), id)
    }

    fn check_ext(&self, ext: u64) -> bool {
        self.misa & ((1 << 26) - 1) & ext == ext
    }

    fn mie(&self) -> bool {
        self.mstatus & MSTATUS_MIE != 0
    }

    fn mpie(&self) -> bool {
        self.mstatus & MSTATUS_MPIE != 0
    }

    fn disable_interrupts(&mut self) {
        self.mstatus &= !MSTATUS_MIE;
    }

    fn mtime(&self) -> u64 {
        // TODO: mtime
        0
    }

    fn write_gr(&mut self, i: usize, value: u64) {
        match i {
            0 => {}
            _ => self.gr[i] = value,
        }
    }
}

impl<M: Memory> Cpu<M> {
    pub fn run(&mut self) {
        info!("CPU frequency is ~{} Hz", FREQUENCY);

        loop {
            while let Ok(msg) = self.cpu_messages_rx.try_recv() {
                self.handle_message(msg);
            }

            if self.mie() && self.mip & self.mie != 0 {
                self.handle_interrupt();
                self.pc = self.npc;
            }

            self.decode();
            self.pc = self.npc;
            self.minstret += 1;

            thread::sleep(SLEEP_TIME);
        }
    }

    fn decode32(&mut self, insn: u32) {
        if RiscvDecode32::decode(self, insn).is_err() {
            trace!("{:x}: illegal instruction {insn:x}", self.pc);
            self.minst = insn;
            self.excp(EXCP_ILLEGAL_INSN, 0);
        }
    }

    fn decode16(&mut self, insn: u16) {
        if RiscvDecode16::decode(self, insn).is_err() {
            trace!("{:x}: illegal instruction {insn:x}", self.pc);
            self.minst = insn as u32;
            self.excp(EXCP_ILLEGAL_INSN, 0);
        }
    }

    fn decode(&mut self) {
        if self.pc & 1 != 0 {
            self.excp(EXCP_INSN_ADDR_MISALIGNED, 0);
            return;
        }

        match self.mem.read_u16_le(self.pc as usize) {
            Ok(lo) => {
                if lo & 3 == 3 {
                    match self.mem.read_u16_le(self.pc as usize + 2) {
                        Ok(hi) => {
                            self.npc = self.pc + 4;
                            self.decode32(((hi as u32) << 16) | lo as u32);
                        }
                        Err(MemoryError::AccessFault) => {
                            self.excp(EXCP_INSN_ACCESS_FAULT, 0);
                        }
                    }
                } else {
                    self.npc = self.pc + 2;
                    self.decode16(lo);
                }
            }
            Err(MemoryError::AccessFault) => {
                self.excp(EXCP_INSN_ACCESS_FAULT, 0);
            }
        }
    }

    fn handle_message(&mut self, msg: Message) {
        match msg {
            Message::DeviceInterrupt(i) => {
                self.mip |= 1 << i;
                trace!("device interrupt({i}), mip={:08x}", self.mip);
            }
        }
    }

    fn handle_interrupt(&mut self) {
        let code = (self.mip & 0xffff0800).trailing_zeros() as u64;
        trace!("interrupt({code})");
        self.excp(make_interrupt(code), 0);
    }

    fn exec<T, O>(&mut self, name: &str, args: T, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, &str, T) -> bool,
    {
        self.check_ext(ext) && op(self, name, args)
    }

    fn exec_b<O>(&mut self, name: &str, b: args_b, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u64, u64) -> bool,
    {
        self.exec(name, b, ext, |cpu, name, b| {
            let s1 = cpu.gr[b.rs1 as usize];
            let s2 = cpu.gr[b.rs2 as usize];
            trace!(
                "{:x}: {name}\tx{}={s1}, x{}={s2}, {}",
                cpu.pc,
                b.rs1,
                b.rs2,
                b.imm
            );
            if op(cpu, s1, s2) {
                cpu.npc = cpu.pc.wrapping_add(b.imm as u64);
            }
            true
        })
    }

    fn exec_i<O>(&mut self, name: &str, i: args_i, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u64, u64) -> u64,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let res = op(cpu, s1, i.imm as u64);
            trace!(
                "{:x}: {name}\tx{}={res}, x{}={s1}, {}",
                cpu.pc,
                i.rd,
                i.rs1,
                i.imm
            );
            cpu.write_gr(i.rd as usize, res);
            true
        })
    }

    fn exec_i32<O>(&mut self, name: &str, i: args_i, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
    {
        self.exec_i(name, i, ext, |cpu, a, b| {
            op(cpu, a as u32, b as u32) as i32 as u64
        })
    }

    fn exec_u<O>(&mut self, name: &str, u: args_u, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u64) -> u64,
    {
        self.exec(name, u, ext, |cpu, name, u| {
            let res = op(cpu, u.imm as u64);
            trace!("{:x}: {name}\tx{}={res}, {:x}", cpu.pc, u.rd, u.imm);
            cpu.write_gr(u.rd as usize, res);
            true
        })
    }

    fn exec_shift<O>(&mut self, name: &str, i: args_shift, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u64, u64) -> u64,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let res = op(cpu, s1, i.shamt as u64);
            trace!(
                "{:x}: {name}\tx{}={res}, x{}={s1}, {}",
                cpu.pc,
                i.rd,
                i.rs1,
                i.shamt
            );
            cpu.write_gr(i.rd as usize, res);
            true
        })
    }

    fn exec_shift32<O>(&mut self, name: &str, i: args_shift, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
    {
        self.exec_shift(name, i, ext, |cpu, a, b| {
            op(cpu, a as u32, b as u32) as i32 as u64
        })
    }

    fn exec_r<O>(&mut self, name: &str, r: args_r, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u64, u64) -> u64,
    {
        self.exec(name, r, ext, |cpu, name, r| {
            let s1 = cpu.gr[r.rs1 as usize];
            let s2 = cpu.gr[r.rs2 as usize];
            let res = op(cpu, s1, s2);
            trace!(
                "{:x}: {name}\tx{}={res}, x{}={s1}, x{}={s2}",
                cpu.pc,
                r.rd,
                r.rs1,
                r.rs2
            );
            cpu.write_gr(r.rd as usize, res);
            true
        })
    }

    fn exec_r32<O>(&mut self, name: &str, r: args_r, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
    {
        self.exec_r(name, r, ext, |cpu, a, b| {
            op(cpu, a as u32, b as u32) as i32 as u64
        })
    }

    fn exec_l<O>(&mut self, name: &str, i: args_i, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, usize) -> Result<u64, MemoryError>,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let addr = s1.wrapping_add(i.imm as u64);
            trace!(
                "{:x}: {name}\tx{}, {:x}(x{}={s1:x})",
                cpu.pc,
                i.rd,
                i.imm,
                i.rs1
            );
            match op(cpu, addr as usize) {
                Ok(value) => {
                    cpu.write_gr(i.rd as usize, value);
                }
                Err(MemoryError::AccessFault) => {
                    cpu.excp(EXCP_LOAD_ACCESS_FAULT, addr);
                }
            }
            true
        })
    }

    fn exec_s<O>(&mut self, name: &str, s: args_s, ext: u64, op: O) -> bool
    where
        O: Fn(&mut Self, usize, u64) -> Result<(), MemoryError>,
    {
        self.exec(name, s, ext, |cpu, name, s| {
            let s1 = cpu.gr[s.rs1 as usize];
            let s2 = cpu.gr[s.rs2 as usize];
            let addr = s1.wrapping_add(s.imm as u64);
            trace!(
                "{:x}: {name}\tx{}={s2}, {:x}(x{}={s1:x})",
                cpu.pc,
                s.rs2,
                s.imm,
                s.rs1
            );
            if let Err(err) = op(cpu, addr as usize, s2) {
                match err {
                    MemoryError::AccessFault => {
                        cpu.excp(EXCP_STORE_AMO_ACCESS_FAULT, addr);
                    }
                }
            }
            true
        })
    }

    fn csr_read(&mut self, csr: u32) -> Option<u64> {
        Some(match csr {
            CSR_MVENDOR_ID => 0,
            CSR_MARCH_ID => 0,
            CSR_MIMP_ID => 0,
            CSR_MHART_ID => 0,
            CSR_MCONFIG_PTR => 0,

            CSR_MSTATUS => self.mstatus,
            //CSR_MSTATUSH => zextract(self.mstatus, 32, 32) as u32,
            CSR_MISA => self.misa,
            CSR_MIE => self.mie,
            CSR_MTVEC => self.mtvec,
            CSR_MIP => self.mip,
            CSR_MCAUSE => self.mcause,
            CSR_MEPC => self.mepc,
            CSR_MSCRATCH => self.mscratch,
            CSR_MTVAL => self.mtval,
            CSR_MINST => self.minst as u64,
            CSR_MEDELEG => 0,
            CSR_MIDELEG => 0,

            CSR_MCYCLE => 0,
            CSR_MCYCLEH => 0,
            CSR_MINSTRET => self.minstret,
            //CSR_MINSTRETH => (self.minstret >> 32) as u32,
            CSR_MHPMCOUNTER0..=CSR_MHPMCOUNTER31 => 0,
            //CSR_MHPMCOUNTER0H..=CSR_MHPMCOUNTER31H => 0,
            CSR_MCOUNTINHIBIT => 0,
            CSR_MHPM_EVENT0..=CSR_MHPM_EVENT31 => 0,

            CSR_CYCLE => 0,
            //CSR_CYCLEH => 0,
            CSR_TIME => self.mtime(),
            //CSR_TIMEH => (self.mtime() >> 32),
            CSR_INSTRET => self.minstret,
            //CSR_INSTRETH => (self.minstret >> 32) as u32,
            CSR_HPMCOUNTER0..=CSR_HPMCOUNTER31 => 0,
            //CSR_HPMCOUNTER0H..=CSR_HPMCOUNTER31H => 0,
            _ => return None,
        })
    }

    fn csr_write(&mut self, csr: u32, value: u64) -> bool {
        match csr {
            CSR_MSTATUS => self.mstatus = value & MSTATUS_MASK,
            // CSR_MSTATUS => {
            //     let value = value & MSTATUS_MASK as u32;
            //     self.mstatus = deposit(self.mstatus, 0, 32, value);
            // }
            // CSR_MSTATUSH => {
            //     let value = value & (MSTATUS_MASK >> 32) as u32;
            //     self.mstatus = deposit(self.mstatus, 32, 32, value);
            // }
            CSR_MISA => self.misa = value & MISA_EXT_MASK,
            CSR_MIE => self.mie = value & MIE_MASK,
            CSR_MTVEC => self.mtvec = value & MTVEC_MASK,
            CSR_MIP => self.mip = value & MIE_MASK,
            CSR_MCAUSE => self.mcause = value,
            CSR_MEPC => self.mepc = value & !1,
            CSR_MSCRATCH => self.mscratch = value,
            CSR_MTVAL => self.mtval = value,
            CSR_MINST => self.minst = value as u32,

            CSR_MCYCLE => {}
            //CSR_MCYCLEH => {}
            CSR_MINSTRET => self.minstret = value,
            //CSR_MINSTRET => self.minstret = deposit(self.minstret, 0, 32, value),
            //CSR_MINSTRETH => self.minstret = deposit(self.minstret, 32, 32, value),
            CSR_MHPMCOUNTER0..=CSR_MHPMCOUNTER31 => {}
            //CSR_MHPMCOUNTER0H..=CSR_MHPMCOUNTER31H => {}
            CSR_MCOUNTINHIBIT => {}
            CSR_MHPM_EVENT0..=CSR_MHPM_EVENT31 => {}

            _ => return false,
        }
        true
    }

    fn exec_csr<O>(&mut self, csr: i32, rr: bool, wr: bool, value: u64, rd: i32, op: O) -> bool
    where
        O: Fn(u64, u64) -> u64,
    {
        let old = if rr {
            match self.csr_read(csr as u32) {
                Some(csr) => csr,
                None => return false,
            }
        } else {
            0
        };
        if wr && !self.csr_write(csr as u32, op(old, value)) {
            return false;
        }
        self.write_gr(rd as usize, old);
        true
    }

    fn excp(&mut self, cause: u64, mtval: u64) {
        if cause >> (mem::size_of_val(&cause) * 8 - 1) == 0 {
            error!("{:x}: exception({cause})", self.pc);
        } else {
            info!("{:x}: interrupt({})", self.pc, cause << 1 >> 1);
        }

        self.mstatus = deposit(self.mstatus, MSTATUS_MPIE_OFFSET, 1, self.mie());
        self.disable_interrupts();
        self.mepc = self.pc;
        self.mcause = cause;
        self.mtval = mtval;
        self.npc = self.mtvec & !3;
        match self.mtvec & 3 {
            0 => {}
            1 => self.npc += self.mcause * 4,
            _ => todo!(),
        };
    }

    fn jal(&mut self, name: &str, j: args_j) -> bool {
        let npc = self.pc.wrapping_add(j.imm as u64);
        trace!("{:x}: {name}\tx{}, {} # {npc:x}", self.pc, j.rd, j.imm);
        self.write_gr(j.rd as usize, self.npc);
        self.npc = npc;
        true
    }

    fn jalr(&mut self, name: &str, i: args_i) -> bool {
        let s1 = self.gr[i.rs1 as usize];
        let npc = s1.wrapping_add(i.imm as u64);
        trace!(
            "{:x}: {name}\tx{}, x{}={s1}, {} # {npc:x}",
            self.pc,
            i.rd,
            i.rs1,
            i.imm
        );
        self.write_gr(i.rd as usize, self.npc);
        self.npc = npc;
        true
    }
}

macro_rules! trans {
    ($($func:ident = $exec:ident($($arg:ident : $ty:ty),* ; $($param:expr),* $(,)?)),+ $(,)?) => (
        $(fn $func(&mut self, $($arg: $ty),*) -> Result<bool, ()> {
            Ok(self.$exec(stringify!($func), ($($arg),*), $($param,)*))
        })+
    );
}

macro_rules! forward_c {
    ($($func:ident($($arg:ident : $ty:ty),* $(,)?)),+ $(,)?) => (
        $(fn $func(&mut self, $($arg: $ty),*) -> Result<bool, ()> {
            Ok(self.check_ext(MISA_EXT_C) && <Self as RiscvDecode32>::$func(self, $($arg),*)?)
        })+
    );
}

impl<M: Memory> RiscvDecode16 for Cpu<M> {
    type Error = ();

    fn fail(&self) -> Self::Error {}

    fn ex_shift_1(&self, value: i32) -> i32 {
        value << 1
    }

    fn ex_shift_2(&self, value: i32) -> i32 {
        value << 2
    }

    fn ex_shift_3(&self, value: i32) -> i32 {
        value << 3
    }

    fn ex_shift_4(&self, value: i32) -> i32 {
        value << 4
    }

    fn ex_shift_12(&self, value: i32) -> i32 {
        value << 12
    }

    fn ex_rvc_shiftli(&self, value: i32) -> i32 {
        value
    }

    fn ex_rvc_shiftri(&self, value: i32) -> i32 {
        value
    }

    fn ex_rvc_register(&self, value: i32) -> i32 {
        value + 8
    }

    fn ex_sreg_register(&self, _value: i32) -> i32 {
        todo!("ex_sreg_register")
    }

    // RVC Standard Extension
    #[rustfmt::skip]
    trans! {
        trans_illegal   = exec(; 0, |cpu, _, _| {
            trace!("{:x}: c.illegal", cpu.pc);
            cpu.minst = 0; // TODO:
            cpu.excp(EXCP_ILLEGAL_INSN, 0);
            true
        }),

        trans_c64_illegal   = exec(; 0, |cpu, _, _| {
            trace!("{:x}: c64.illegal", cpu.pc);
            cpu.minst = 0; // TODO:
            cpu.excp(EXCP_ILLEGAL_INSN, 0);
            true
        }),
    }

    // RVC Standard Extension
    #[rustfmt::skip]
    forward_c! {
        trans_lui(u: args_u),

        trans_jal(j: args_j),
        trans_jalr(j: args_i),

        trans_beq(b: args_b),
        trans_bne(b: args_b),

        trans_lw(i: args_i),
        trans_ld(i: args_i),

        trans_sw(s: args_s),
        trans_sd(s: args_s),

        trans_andi(i: args_i),

        trans_slli(i: args_shift),
        trans_srli(i: args_shift),
        trans_srai(i: args_shift),

        trans_addi(i: args_i),

        trans_and(r: args_r),
        trans_or(r: args_r),
        trans_xor(r: args_r),
        trans_add(r: args_r),
        trans_sub(r: args_r),

        trans_addiw(i: args_i),

        trans_addw(r: args_r),
        trans_subw(r: args_r),

        trans_ebreak(),
    }
}

impl<M: Memory> RiscvDecode32 for Cpu<M> {
    type Error = ();

    fn fail(&self) -> Self::Error {}

    fn ex_plus_1(&self, value: i32) -> i32 {
        value + 1
    }

    fn ex_shift_12(&self, value: i32) -> i32 {
        value << 12
    }

    fn ex_shift_3(&self, value: i32) -> i32 {
        value << 3
    }

    fn ex_shift_1(&self, value: i32) -> i32 {
        value << 1
    }

    // RV64I Base Instruction Set
    #[rustfmt::skip]
    trans! {
        trans_lui       = exec_u(u: args_u; 0, |_, i| i),
        trans_auipc     = exec_u(u: args_u; 0, |cpu, i| cpu.pc.wrapping_add(i)),

        trans_jal       = exec(j: args_j; 0, Self::jal),
        trans_jalr      = exec(i: args_i; 0, Self::jalr),

        trans_beq       = exec_b(b: args_b; 0, |_, a, b| a == b),
        trans_bne       = exec_b(b: args_b; 0, |_, a, b| a != b),
        trans_blt       = exec_b(b: args_b; 0, |_, a, b| (a as i64) < (b as i64)),
        trans_bge       = exec_b(b: args_b; 0, |_, a, b| (a as i64) >= (b as i64)),
        trans_bltu      = exec_b(b: args_b; 0, |_, a, b| a < b),
        trans_bgeu      = exec_b(b: args_b; 0, |_, a, b| a >= b),

        trans_andi      = exec_i(i: args_i; 0, |_, a, b| a & b),
        trans_ori       = exec_i(i: args_i; 0, |_, a, b| a | b),
        trans_xori      = exec_i(i: args_i; 0, |_, a, b| a ^ b),
        trans_addi      = exec_i(i: args_i; 0, |_, a, b| a.wrapping_add(b)),
        trans_slti      = exec_i(i: args_i; 0, |_, a, b| slt(a, b)),
        trans_sltiu     = exec_i(i: args_i; 0, |_, a, b| sltu(a, b)),

        trans_slli      = exec_shift(i: args_shift; 0, |_, a, b| a.wrapping_shl(b as u32)),
        trans_srli      = exec_shift(i: args_shift; 0, |_, a, b| a.wrapping_shr(b as u32)),
        trans_srai      = exec_shift(i: args_shift; 0, |_, a, b| sra(a, b)),

        trans_and       = exec_r(r: args_r; 0, |_, a, b| a & b),
        trans_or        = exec_r(r: args_r; 0, |_, a, b| a | b),
        trans_xor       = exec_r(r: args_r; 0, |_, a, b| a ^ b),
        trans_add       = exec_r(r: args_r; 0, |_, a, b| a.wrapping_add(b)),
        trans_sub       = exec_r(r: args_r; 0, |_, a, b| a.wrapping_sub(b)),
        trans_slt       = exec_r(r: args_r; 0, |_, a, b| slt(a, b)),
        trans_sltu      = exec_r(r: args_r; 0, |_, a, b| sltu(a, b)),
        trans_sll       = exec_r(r: args_r; 0, |_, a, b| a.wrapping_shl(b as u32)),
        trans_srl       = exec_r(r: args_r; 0, |_, a, b| a.wrapping_shr(b as u32)),
        trans_sra       = exec_r(r: args_r; 0, |_, a, b| sra(a, b)),

        trans_addiw     = exec_i32(i: args_i; 0, |_, a, b| a.wrapping_add(b)),

        trans_slliw     = exec_shift32(i: args_shift; 0, |_, a, b| a.wrapping_shl(b)),
        trans_srliw     = exec_shift32(i: args_shift; 0, |_, a, b| a.wrapping_shr(b)),
        trans_sraiw     = exec_shift32(i: args_shift; 0, |_, a, b| sraw(a, b)),

        trans_addw      = exec_r32(r: args_r; 0, |_, a, b| a.wrapping_add(b)),
        trans_subw      = exec_r32(r: args_r; 0, |_, a, b| a.wrapping_sub(b)),
        trans_sllw      = exec_r32(r: args_r; 0, |_, a, b| a.wrapping_shl(b)),
        trans_srlw      = exec_r32(r: args_r; 0, |_, a, b| a.wrapping_shr(b)),
        trans_sraw      = exec_r32(r: args_r; 0, |_, a, b| sraw(a, b)),

        trans_lb        = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_i8(a).map(|i| i as i32 as u64)),
        trans_lh        = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_i16_le(a).map(|i| i as i32 as u64)),
        trans_lw        = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_i32_le(a).map(|i| i as i32 as u64)),
        trans_ld        = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_i64_le(a)),
        trans_lbu       = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_u8(a).map(|i| i as u64)),
        trans_lhu       = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_u16_le(a).map(|i| i as u64)),
        trans_lwu       = exec_l(i: args_i; 0, |cpu, a| cpu.mem.read_u32_le(a).map(|i| i as u64)),

        trans_sb        = exec_s(s: args_s; 0, |cpu, a, v| cpu.mem.write_u8(a, v as u8)),
        trans_sh        = exec_s(s: args_s; 0, |cpu, a, v| cpu.mem.write_u16_le(a, v as u16)),
        trans_sw        = exec_s(s: args_s; 0, |cpu, a, v| cpu.mem.write_u32_le(a, v as u32)),
        trans_sd        = exec_s(s: args_s; 0, |cpu, a, v| cpu.mem.write_u64_le(a, v)),

        trans_fence     = exec(pred: i32, succ: i32; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            // TODO: fence
            false
        }),

        trans_fence_i   = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            // TODO: fence_i
            false
        }),

        trans_pause     = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            thread::sleep(PAUSE_TIME);
            true
        }),

        trans_ecall     = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            // TODO: ecall
            false
        }),

        trans_ebreak    = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            cpu.excp(EXCP_BREAKPOINT, 0);
            true
        }),
    }

    // RV64 Zicsr Standard Extension
    #[rustfmt::skip]
    trans! {
        trans_csrrw     = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, rd != 0, true, s1, rd, |_, b| b)
        }),
        trans_csrrs     = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, s1, rd, |a, b| a | b)
        }),
        trans_csrrc     = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, s1, rd, |a, b| a & !b)
        }),
        trans_csrrwi    = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, rd != 0, true, rs1 as u64, rd, |_, b| b)
        }),
        trans_csrrsi    = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, rs1 as u64, rd, |a, b| a | b)
        }),
        trans_csrrci    = exec(csr: i32, rs1: i32, rd: i32; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, rs1 as u64, rd, |a, b| a & !b)
        }),
    }

    // RV64M Standard Extension
    #[rustfmt::skip]
    trans! {
        trans_mul       = exec_r(r: args_r; MISA_EXT_M, |_, a, b| mul(a, b)),
        trans_mulh      = exec_r(r: args_r; MISA_EXT_M, |_, a, b| mulh(a, b)),
        trans_mulhu     = exec_r(r: args_r; MISA_EXT_M, |_, a, b| mulhu(a, b)),
        trans_mulhsu    = exec_r(r: args_r; MISA_EXT_M, |_, a, b| mulhsu(a, b)),
        trans_div       = exec_r(r: args_r; MISA_EXT_M, |_, a, b| div(a, b)),
        trans_divu      = exec_r(r: args_r; MISA_EXT_M, |_, a, b| divu(a, b)),
        trans_rem       = exec_r(r: args_r; MISA_EXT_M, |_, a, b| rem(a, b)),
        trans_remu      = exec_r(r: args_r; MISA_EXT_M, |_, a, b| remu(a, b)),

        trans_mulw      = exec_r32(r: args_r; MISA_EXT_M, |_, a, b| mulw(a, b)),
        trans_divw      = exec_r32(r: args_r; MISA_EXT_M, |_, a, b| divw(a, b)),
        trans_divuw     = exec_r32(r: args_r; MISA_EXT_M, |_, a, b| divuw(a, b)),
        trans_remw      = exec_r32(r: args_r; MISA_EXT_M, |_, a, b| remw(a, b)),
        trans_remuw     = exec_r32(r: args_r; MISA_EXT_M, |_, a, b| remuw(a, b)),
    }

    // Privileged Instructions
    #[rustfmt::skip]
    trans! {
        trans_mret      = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            cpu.npc = cpu.mepc;
            cpu.mstatus = deposit(cpu.mstatus, MSTATUS_MIE_OFFSET, 1, cpu.mpie() as u64);
            true
        }),

        trans_wfi       = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            if let Ok(i) = cpu.cpu_messages_rx.recv() {
                cpu.handle_message(i);
            }
            true
        }),
    }
}

fn slt(a: u64, b: u64) -> u64 {
    ((a as i64) < (b as i64)) as u64
}

fn sltu(a: u64, b: u64) -> u64 {
    (a < b) as u64
}

fn sra(a: u64, b: u64) -> u64 {
    (a as i64).wrapping_shr(b as u32) as u64
}

fn sraw(a: u32, b: u32) -> u32 {
    (a as i32).wrapping_shr(b) as u32
}

fn mulw(a: u32, b: u32) -> u32 {
    (a as i32).wrapping_mul(b as i32) as u32
}

fn mul(a: u64, b: u64) -> u64 {
    (a as i64).wrapping_mul(b as i64) as u64
}

fn mulh(a: u64, b: u64) -> u64 {
    ((a as i128).wrapping_mul(b as i128) >> 64) as u64
}

fn mulhsu(a: u64, b: u64) -> u64 {
    ((a as i128).wrapping_mul(b as u128 as i128) >> 64) as u64
}

fn mulhu(a: u64, b: u64) -> u64 {
    ((a as u128).wrapping_mul(b as u128) >> 64) as u64
}

fn div(a: u64, b: u64) -> u64 {
    if b == 0 {
        return u64::MAX;
    }
    (a as i64)
        .checked_div(b as i64)
        .map(|i| i as u64)
        .unwrap_or(u64::MAX >> 1)
}

fn divu(a: u64, b: u64) -> u64 {
    a.checked_div(b).unwrap_or(u64::MAX - 1)
}

fn rem(a: u64, b: u64) -> u64 {
    if b == 0 {
        return a;
    }
    (a as i64).checked_rem(b as i64).unwrap_or(0) as u64
}

fn remu(a: u64, b: u64) -> u64 {
    a.checked_rem(b).unwrap_or(a)
}

fn divw(a: u32, b: u32) -> u32 {
    if b == 0 {
        return u32::MAX;
    }
    (a as i32)
        .checked_div(b as i32)
        .map(|i| i as u32)
        .unwrap_or(u32::MAX >> 1)
}

fn divuw(a: u32, b: u32) -> u32 {
    a.checked_div(b).unwrap_or(u32::MAX - 1)
}

fn remw(a: u32, b: u32) -> u32 {
    if b == 0 {
        return a;
    }
    (a as i32).checked_rem(b as i32).unwrap_or(0) as u32
}

fn remuw(a: u32, b: u32) -> u32 {
    a.checked_rem(b).unwrap_or(a)
}
