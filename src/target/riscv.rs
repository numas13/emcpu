use std::mem;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use crate::device::DeviceInterrupt;
use crate::mem::{Error as MemoryError, Memory};
use crate::target::Message;
use crate::utils::{deposit, sextract, zextract};

include!(concat!(env!("OUT_DIR"), "/target/riscv/insn16.rs"));
include!(concat!(env!("OUT_DIR"), "/target/riscv/insn32.rs"));

const EXCP_INSN_ADDR_MISALIGNED: u32 = 0;
const EXCP_INSN_ACCESS_FAULT: u32 = 1;
const EXCP_ILLEGAL_INSN: u32 = 2;
const EXCP_BREAKPOINT: u32 = 3;
const EXCP_LOAD_ADDR_MISALIGNED: u32 = 4;
const EXCP_LOAD_ACCESS_FAULT: u32 = 5;
const EXCP_STORE_AMO_ADDR_MISALIGNED: u32 = 6;
const EXCP_STORE_AMO_ACCESS_FAULT: u32 = 7;
const EXCP_ECALL_UMODE: u32 = 8;
const EXCP_ECALL_SMODE: u32 = 9;
const EXCP_ECALL_MMODE: u32 = 11;
const EXCP_INSN_PAGE_FAULT: u32 = 12;
const EXCP_LOAD_PAGE_FAULT: u32 = 13;
const EXCP_STORE_AMO_PAGE_FAULT: u32 = 15;

const fn make_interrupt(code: u32) -> u32 {
    (1 << 31) | code
}

const EXCP_MACHINE_EXTERNAL_INT: u32 = make_interrupt(11);

// Unprivileged Floating-Point CSRs
const CSR_FFLAGS: u32 = 0x001;
const CSR_FRM: u32 = 0x002;
const CSR_FCSR: u32 = 0x003;

// Unprivileged Counter/Timers
const CSR_CYCLE: u32 = 0xc00;
const CSR_TIME: u32 = 0xc01;
const CSR_INSTRET: u32 = 0xc02;
// 0..32
const CSR_HPMCOUNTER0: u32 = 0xc00;
const CSR_CYCLEH: u32 = 0xc80;
const CSR_TIMEH: u32 = 0xc81;
const CSR_INSTRETH: u32 = 0xc82;
const CSR_HPMCOUNTER0H: u32 = 0xc80;

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
const CSR_MCOUTNEREN: u32 = 0x306;
const CSR_MSTATUSH: u32 = 0x310;

// Machine Trap Handling
const CSR_MSCRATCH: u32 = 0x340;
const CSR_MEPC: u32 = 0x341;
const CSR_MCAUSE: u32 = 0x342;
const CSR_MTVAL: u32 = 0x343;
const CSR_MIP: u32 = 0x344;
const CSR_MINST: u32 = 0x34a;
const CSR_MTVAL2: u32 = 0x34b;

// Machine Configuration
const CSR_MENV_CFG: u32 = 0x30a;
const CSR_MENV_CFGH: u32 = 0x31a;
const CSR_MSEC_CFG: u32 = 0x747;
const CSR_MSEC_CFGH: u32 = 0x757;

// Machine Memory Protection
// 0..16
const CSR_PMP_CFG_0: u32 = 0x3a0;
// 0..64
const CSR_PMP_ADDR_0: u32 = 0x3b0;

// Machine Non-Maskable Interrupt Handling
const CSR_MNSCRATCH: u32 = 0x740;
const CSR_MNEPC: u32 = 0x741;
const CSR_MNCAUSE: u32 = 0x742;
const CSR_MNSTATUS: u32 = 0x744;

// Machine Counter/Timers
const CSR_MCYCLE: u32 = 0xb00;
const CSR_MCYCLEH: u32 = 0xb80;
const CSR_MINSTRET: u32 = 0xb02;
const CSR_MINSTRETH: u32 = 0xb82;
// 0..32
const CSR_MHPMCOUNTER0: u32 = 0xb00;
const CSR_MHPMCOUNTER0H: u32 = 0xb80;

// Machine Counter Setup
const CSR_MCOUNTINHIBIT: u32 = 0x320;
// 0..32
const CSR_MHPM_EVENT0: u32 = 0x320;

// Debug/Trace Registers (shared with Debug Mode)
const CSR_TSELECT: u32 = 0x7a0;
const CSR_TDATA1: u32 = 0x7a1;
const CSR_TDATA2: u32 = 0x7a2;
const CSR_TDATA3: u32 = 0x7a3;
const CSR_MCONTEXT: u32 = 0x7a8;

// Debug Mode Registers
const CSR_DCSR: u32 = 0x7b0;
const CSR_DPC: u32 = 0x7b1;
const CSR_SCRATCH0: u32 = 0x7b2;
const CSR_SCRATCH1: u32 = 0x7b3;

const fn misa_ext(c: u8) -> u32 {
    match c {
        b'a'..=b'z' => 1 << (c - b'a'),
        b'A'..=b'A' => 1 << (c - b'A'),
        _ => 0,
    }
}

const MISA_EXT_M: u32 = misa_ext(b'm');
const MISA_EXT_I: u32 = misa_ext(b'i');
const MISA_EXT_X: u32 = misa_ext(b'x');
const MISA_EXT_ALL: u32 = MISA_EXT_M | MISA_EXT_I;
const MISA_EXT_MASK: u32 = MISA_EXT_ALL ^ MISA_EXT_I;

const MSTATUS_MIE: u64 = 1 << 3;
const MSTATUS_MASK: u64 = MSTATUS_MIE;

const MIE_MASK: u32 = 0xffff0800;
const MTVEC_MASK: u32 = 0xfffffffd;

const MIE_SS: u32 = 1 << 1;
const MIE_MS: u32 = 1 << 3;
const MIE_ST: u32 = 1 << 5;
const MIE_MT: u32 = 1 << 7;
const MIE_SE: u32 = 1 << 9;
const MIE_ME: u32 = 1 << 11;

const SLEEP_TIME: Duration = Duration::from_micros(2000);
const PAUSE_TIME: Duration = Duration::from_secs(1);

pub struct Cpu<M> {
    pub pc: u32,
    pub npc: u32,
    pub gr: [u32; 32],
    pub mem: M,
    cpu_messages_tx: mpsc::SyncSender<Message>,
    cpu_messages_rx: mpsc::Receiver<Message>,

    mstatus: u64,
    misa: u32,
    mip: u32,
    mie: u32,
    mcause: u32,
    mepc: u32,
    mtvec: u32,
    mscratch: u32,
    mtval: u32,
    minst: u32,
}

impl<M> Cpu<M> {
    pub fn new(pc: u32, mem: M) -> Self {
        let (cpu_messages_tx, cpu_messages_rx) = mpsc::sync_channel(0);

        Self {
            pc,
            npc: 0,
            gr: [0; 32],
            mem,
            cpu_messages_rx,
            cpu_messages_tx,

            mstatus: 0,
            misa: MISA_EXT_ALL,
            mip: 0,
            mie: 0,
            mcause: 0,
            mepc: 0,
            mtvec: 0,
            mscratch: 0,
            mtval: 0,
            minst: 0,
        }
    }

    pub fn device_interrupt(&mut self, id: u32) -> DeviceInterrupt {
        assert!(id < (mem::size_of_val(&self.mie) * 8 - 16) as u32);
        DeviceInterrupt::new(self.cpu_messages_tx.clone(), id)
    }

    fn check_ext(&self, ext: u32) -> bool {
        self.misa & ((1 << 26) - 1) & ext == ext
    }

    fn mie(&self) -> bool {
        self.mstatus & MSTATUS_MIE != 0
    }

    fn disable_interrupts(&mut self) {
        self.mstatus &= !MSTATUS_MIE;
    }

    fn enable_interrupts(&mut self) {
        self.mstatus |= MSTATUS_MIE;
    }

    pub fn write_gr(&mut self, i: usize, value: u32) {
        match i {
            0 => {}
            _ => self.gr[i] = value,
        }
    }
}

impl<M: Memory> Cpu<M> {
    pub fn run(&mut self) {
        info!("insn execution delay is {SLEEP_TIME:?}");

        loop {
            if self.mie() {
                if let Ok(msg) = self.cpu_messages_rx.recv_timeout(SLEEP_TIME) {
                    self.handle_message(msg);
                }

                if self.mip & self.mie != 0 {
                    self.handle_interrupt();
                    self.pc = self.npc;
                    continue;
                }
            } else {
                thread::sleep(SLEEP_TIME);
            }

            match self.mem.read_u32_le(self.pc as usize) {
                Ok(insn) => {
                    self.npc = self.pc + 4;

                    if self.pc & 3 != 0 {
                        // TODO: lower priority than illegal insn exception
                        self.excp(EXCP_INSN_ADDR_MISALIGNED);
                    } else if !RiscvDecode32::decode(self, insn) {
                        trace!("{:x}: illegal instruction {insn:x}", self.pc);
                        self.minst = insn;
                        self.excp(EXCP_ILLEGAL_INSN);
                    }
                }
                Err(MemoryError::AccessFault) => {
                    self.excp(EXCP_INSN_ACCESS_FAULT);
                }
            }

            self.pc = self.npc;
        }
    }

    fn handle_message(&mut self, msg: Message) {
        match msg {
            Message::DeviceInterrupt(i) => {
                self.mip |= 1 << (16 + i);
                trace!("device interrupt({i}), mip={:08x}", self.mip);
            }
        }
    }

    fn handle_interrupt(&mut self) {
        let code = (self.mip & 0xffff0800).trailing_zeros();
        trace!("interrupt({code})");
        self.mip &= !(1 << code);
        self.excp(make_interrupt(code));
    }

    fn exec<T, O>(&mut self, name: &str, args: T, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, &str, T) -> bool,
    {
        self.check_ext(ext) && op(self, name, args)
    }

    fn exec_b<O>(&mut self, name: &str, b: &args_b, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> bool,
    {
        self.exec(name, b, ext, |cpu, name, b| {
            let s1 = cpu.gr[b.rs1 as usize];
            let s2 = cpu.gr[b.rs2 as usize];
            trace!(
                "{:x}: {name}\tx{}={s1}, x{}={s2}, {}",
                cpu.pc,
                b.rs1,
                b.rs2,
                b.imm as i32
            );
            if op(cpu, s1, s2) {
                cpu.npc = cpu.pc.wrapping_add(b.imm as u32);
            }
            true
        })
    }

    fn exec_i<O>(&mut self, name: &str, i: &args_i, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let imm = i.imm as u32;
            let res = op(cpu, s1, imm);
            trace!(
                "{:x}: {name}\tx{}={res}, x{}={s1}, {imm}",
                cpu.pc,
                i.rd,
                i.rs1
            );
            cpu.write_gr(i.rd as usize, res);
            true
        })
    }

    fn exec_u<O>(&mut self, name: &str, u: &args_u, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, u32) -> u32,
    {
        self.exec(name, u, ext, |cpu, name, u| {
            let imm = u.imm as u32;
            let res = op(cpu, imm);
            trace!("{:x}: {name}\tx{}={res}, {imm:x}", cpu.pc, u.rd);
            cpu.write_gr(u.rd as usize, res);
            true
        })
    }

    fn exec_shift<O>(&mut self, name: &str, i: &args_shift, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let imm = i.shamt as u32;
            let res = op(cpu, s1, imm);
            trace!(
                "{:x}: {name}\tx{}={res}, x{}={s1}, {imm}",
                cpu.pc,
                i.rd,
                i.rs1
            );
            cpu.write_gr(i.rd as usize, res);
            true
        })
    }

    fn exec_r<O>(&mut self, name: &str, r: &args_r, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, u32, u32) -> u32,
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

    fn exec_l<O>(&mut self, name: &str, i: &args_i, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, usize) -> Result<u32, MemoryError>,
    {
        self.exec(name, i, ext, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let imm = i.imm as i32;
            let addr = s1.wrapping_add(imm as u32);
            trace!("{:x}: {name}\tx{}, {imm}(x{}={s1})", cpu.pc, i.rd, i.rs1);
            match op(cpu, addr as usize) {
                Ok(value) => {
                    cpu.write_gr(i.rd as usize, value);
                }
                Err(MemoryError::AccessFault) => {
                    cpu.mtval = addr;
                    cpu.excp(EXCP_LOAD_ACCESS_FAULT);
                }
            }
            true
        })
    }

    fn exec_s<O>(&mut self, name: &str, s: &args_s, ext: u32, op: O) -> bool
    where
        O: Fn(&mut Self, usize, u32) -> Result<(), MemoryError>,
    {
        self.exec(name, s, ext, |cpu, name, s| {
            let s1 = cpu.gr[s.rs1 as usize];
            let s2 = cpu.gr[s.rs2 as usize];
            let imm = s.imm as i32;
            let addr = s1.wrapping_add(imm as u32);
            trace!(
                "{:x}: {name}\tx{}={s2}, {imm}(x{}={s1})",
                cpu.pc,
                s.rs2,
                s.rs1
            );
            if let Err(err) = op(cpu, addr as usize, s2) {
                match err {
                    MemoryError::AccessFault => {
                        cpu.mtval = addr;
                        cpu.excp(EXCP_STORE_AMO_ACCESS_FAULT);
                    }
                }
            }
            true
        })
    }

    fn csr_read(&mut self, csr: u32) -> Option<u32> {
        Some(match csr {
            CSR_MVENDOR_ID => 0,
            CSR_MARCH_ID => 0,
            CSR_MIMP_ID => 0,
            CSR_MHART_ID => 0,
            CSR_MSTATUS => self.mstatus as u32,
            CSR_MSTATUSH => zextract(self.mstatus, 32, 32) as u32,
            CSR_MISA => self.misa,
            CSR_MIE => self.mie,
            CSR_MTVEC => self.mtvec,
            CSR_MIP => self.mip,
            CSR_MCAUSE => self.mcause,
            CSR_MEPC => self.mepc,
            CSR_MSCRATCH => self.mscratch,
            CSR_MTVAL => self.mtval,
            CSR_MINST => self.minst,
            CSR_MEDELEG => 0,
            CSR_MIDELEG => 0,
            _ => return None,
        })
    }

    fn csr_write(&mut self, csr: u32, value: u32) -> bool {
        match csr {
            CSR_MSTATUS => self.mstatus = deposit(self.mstatus, 0, 32, value),
            CSR_MSTATUSH => self.mstatus = deposit(self.mstatus, 32, 32, value),
            CSR_MISA => self.misa = value & MISA_EXT_MASK,
            CSR_MIE => self.mie = value & MIE_MASK,
            CSR_MTVEC => self.mtvec = value & MTVEC_MASK,
            CSR_MIP => self.mip = value,
            CSR_MCAUSE => self.mcause = value,
            CSR_MEPC => self.mepc = value,
            CSR_MSCRATCH => self.mscratch = value,
            CSR_MTVAL => self.mtval = value,
            CSR_MINST => self.minst = value,
            _ => return false,
        }
        true
    }

    fn exec_csr<O>(&mut self, csr: isize, rr: bool, wr: bool, value: u32, rd: isize, op: O) -> bool
    where
        O: Fn(u32, u32) -> u32,
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

    fn excp(&mut self, cause: u32) {
        if cause >> (mem::size_of_val(&cause) * 8 - 1) == 0 {
            error!("{:x}: exception({cause})", self.pc);
        } else {
            info!("{:x}: interrupt({})", self.pc, cause << 1 >> 1);
        }
        self.disable_interrupts();
        self.mepc = self.pc;
        self.mcause = cause;
        self.npc = self.mtvec & !3;
        match self.mtvec & 3 {
            0 => {}
            1 => self.npc += self.mcause * 4,
            _ => todo!(),
        };
    }
}

macro_rules! trans {
    ($($func:ident = $exec:ident($($arg:ident : $ty:ty),* ; $($param:expr),* $(,)?)),+ $(,)?) => (
        $(fn $func(&mut self, $($arg: $ty),*) -> bool {
            self.$exec(stringify!($func), ($($arg),*), $($param,)*)
        })+
    );
}

impl<M: Memory> RiscvDecode32 for Cpu<M> {
    fn ex_plus_1(&mut self, value: isize) -> isize {
        value + 1
    }

    fn ex_shift_12(&mut self, value: isize) -> isize {
        value << 12
    }

    fn ex_shift_3(&mut self, value: isize) -> isize {
        value << 3
    }

    fn ex_shift_1(&mut self, value: isize) -> isize {
        value << 1
    }

    // RV32I Base Instruction Set
    #[rustfmt::skip]
    trans! {
        trans_lui       = exec_u(u: &args_u; 0, |_, i| i),
        trans_auipc     = exec_u(u: &args_u; 0, |cpu, i| cpu.pc.wrapping_add(i)),

        trans_jal       = exec(j: &args_j; 0, |cpu, name, j| {
            let npc = cpu.pc.wrapping_add(j.imm as u32);
            trace!("{:x}: {name}\tx{}, {} # {npc:x}", cpu.pc, j.rd, j.imm as i32);
            cpu.write_gr(j.rd as usize, cpu.npc);
            cpu.npc = npc;
            true
        }),

        trans_jalr      = exec(i: &args_i; 0, |cpu, name, i| {
            let s1 = cpu.gr[i.rs1 as usize];
            let npc = s1.wrapping_add(i.imm as u32);
            trace!("{:x}: {name}\tx{}, x{}={s1}, {} # {npc:x}", cpu.pc, i.rd, i.rs1, i.imm as i32);
            cpu.write_gr(i.rd as usize, cpu.npc);
            cpu.npc = npc;
            true
        }),

        trans_beq       = exec_b(b: &args_b; 0, |_, a, b| a == b),
        trans_bne       = exec_b(b: &args_b; 0, |_, a, b| a != b),
        trans_blt       = exec_b(b: &args_b; 0, |_, a, b| (a as i32) < (b as i32)),
        trans_bge       = exec_b(b: &args_b; 0, |_, a, b| (a as i32) >= (b as i32)),
        trans_bltu      = exec_b(b: &args_b; 0, |_, a, b| a < b),
        trans_bgeu      = exec_b(b: &args_b; 0, |_, a, b| a >= b),

        trans_andi      = exec_i(i: &args_i; 0, |_, a, b| a & b),
        trans_ori       = exec_i(i: &args_i; 0, |_, a, b| a | b),
        trans_xori      = exec_i(i: &args_i; 0, |_, a, b| a ^ b),
        trans_addi      = exec_i(i: &args_i; 0, |_, a, b| a.wrapping_add(b)),
        trans_slti      = exec_i(i: &args_i; 0, |_, a, b| slt(a, b)),
        trans_sltiu     = exec_i(i: &args_i; 0, |_, a, b| sltu(a, b)),

        trans_slli      = exec_shift(i: &args_shift; 0, |_, a, b| a.wrapping_shl(b)),
        trans_srli      = exec_shift(i: &args_shift; 0, |_, a, b| a.wrapping_shr(b)),
        trans_srai      = exec_shift(i: &args_shift; 0, |_, a, b| sra(a, b)),

        trans_and       = exec_r(r: &args_r; 0, |_, a, b| a & b),
        trans_or        = exec_r(r: &args_r; 0, |_, a, b| a | b),
        trans_xor       = exec_r(r: &args_r; 0, |_, a, b| a ^ b),
        trans_add       = exec_r(r: &args_r; 0, |_, a, b| a.wrapping_add(b)),
        trans_sub       = exec_r(r: &args_r; 0, |_, a, b| a.wrapping_sub(b)),
        trans_slt       = exec_r(r: &args_r; 0, |_, a, b| slt(a, b)),
        trans_sltu      = exec_r(r: &args_r; 0, |_, a, b| sltu(a, b)),
        trans_sll       = exec_r(r: &args_r; 0, |_, a, b| a.wrapping_shl(b)),
        trans_srl       = exec_r(r: &args_r; 0, |_, a, b| a.wrapping_shr(b)),
        trans_sra       = exec_r(r: &args_r; 0, |_, a, b| sra(a, b)),

        trans_lb        = exec_l(i: &args_i; 0, |cpu, a| cpu.mem.read_i8(a).map(|i| i as i32 as u32)),
        trans_lh        = exec_l(i: &args_i; 0, |cpu, a| cpu.mem.read_i16_le(a).map(|i| i as i32 as u32)),
        trans_lw        = exec_l(i: &args_i; 0, |cpu, a| cpu.mem.read_i32_le(a)),
        trans_lbu       = exec_l(i: &args_i; 0, |cpu, a| cpu.mem.read_u8(a).map(|i| i as u32)),
        trans_lhu       = exec_l(i: &args_i; 0, |cpu, a| cpu.mem.read_u16_le(a).map(|i| i as u32)),

        trans_sb        = exec_s(s: &args_s; 0, |cpu, a, v| cpu.mem.write_u8(a, v as u8)),
        trans_sh        = exec_s(s: &args_s; 0, |cpu, a, v| cpu.mem.write_u16_le(a, v as u16)),
        trans_sw        = exec_s(s: &args_s; 0, |cpu, a, v| cpu.mem.write_u32_le(a, v)),

        trans_fence     = exec(pred: isize, succ: isize; 0, |cpu, name, _| {
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
            cpu.excp(EXCP_BREAKPOINT);
            true
        }),
    }

    // RV32/RV64 Zicsr Standard Extension
    #[rustfmt::skip]
    trans! {
        trans_csrrw     = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, rd != 0, true, s1, rd, |_, b| b)
        }),
        trans_csrrs     = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, s1, rd, |a, b| a | b)
        }),
        trans_csrrc     = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            let s1 = cpu.gr[rs1 as usize];
            trace!("{:x}: {name}\tx{rd}, {csr:x}, x{rs1}={s1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, s1, rd, |a, b| a & !b)
        }),
        trans_csrrwi    = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, rd != 0, true, rs1 as u32, rd, |_, b| b)
        }),
        trans_csrrsi    = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, rs1 as u32, rd, |a, b| a | b)
        }),
        trans_csrrci    = exec(csr: isize, rs1: isize, rd: isize; 0, |cpu, name, (csr, rs1, rd)| {
            trace!("{:x}: {name}\tx{rd}, {csr:x}, {rs1}", cpu.pc);
            cpu.exec_csr(csr, true, rs1 != 0, rs1 as u32, rd, |a, b| a & !b)
        }),
    }

    // RV32M Standard Extension
    #[rustfmt::skip]
    trans! {
        trans_mul       = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| mul(a, b)),
        trans_mulh      = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| mulh(a, b)),
        trans_mulhu     = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| mulhu(a, b)),
        trans_mulhsu    = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| mulhsu(a, b)),
        trans_div       = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| div(a, b)),
        trans_divu      = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| divu(a, b)),
        trans_rem       = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| rem(a, b)),
        trans_remu      = exec_r(r: &args_r; MISA_EXT_M, |_, a, b| remu(a, b)),
    }

    // Privileged Instructions
    #[rustfmt::skip]
    trans! {
        trans_mret      = exec(; 0, |cpu, name, _| {
            trace!("{:x}: {name}", cpu.pc);
            cpu.npc = cpu.mepc;
            cpu.enable_interrupts();
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

fn slt(a: u32, b: u32) -> u32 {
    ((a as i32) < (b as i32)) as u32
}

fn sltu(a: u32, b: u32) -> u32 {
    (a < b) as u32
}

fn sra(a: u32, b: u32) -> u32 {
    (a as i32).wrapping_shr(b) as u32
}

fn mul(a: u32, b: u32) -> u32 {
    (a as i32).wrapping_mul(b as i32) as u32
}

fn mulh(a: u32, b: u32) -> u32 {
    ((a as i64).wrapping_mul(b as i64) >> 32) as u32
}

fn mulhsu(a: u32, b: u32) -> u32 {
    ((a as i64).wrapping_mul(b as u64 as i64) >> 32) as u32
}

fn mulhu(a: u32, b: u32) -> u32 {
    ((a as u64).wrapping_mul(b as u64) >> 32) as u32
}

fn div(a: u32, b: u32) -> u32 {
    if b == 0 {
        return u32::MAX;
    }
    (a as i32)
        .checked_div(b as i32)
        .map(|i| i as u32)
        .unwrap_or(u32::MAX >> 1)
}

fn divu(a: u32, b: u32) -> u32 {
    a.checked_div(b).unwrap_or(u32::MAX - 1)
}

fn rem(a: u32, b: u32) -> u32 {
    if b == 0 {
        return a;
    }
    (a as i32).checked_rem(b as i32).unwrap_or(0) as u32
}

fn remu(a: u32, b: u32) -> u32 {
    a.checked_rem(b).unwrap_or(a)
}
