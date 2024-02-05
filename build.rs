use std::{
    env, fmt,
    fs::{self, File},
    hash::Hash,
    io::BufWriter,
    path::Path,
};

use decodetree::{Generator, Insn};

fn gen<T>(trait_name: &str, path: &str, out: &str)
where
    T: Default + Hash + fmt::LowerHex + Ord + Insn,
{
    println!("cargo:rerun-if-changed={path}");
    let src = fs::read_to_string(path).unwrap();
    let tree = match decodetree::parse::<T>(&src) {
        Ok(tree) => tree,
        Err(errors) => {
            for err in errors.iter(path) {
                eprintln!("{err}");
            }
            std::process::exit(1);
        }
    };
    if let Some(parent) = Path::new(out).parent() {
        fs::create_dir_all(parent).unwrap();
    }
    let mut out = BufWriter::new(File::create(out).unwrap());

    Generator::<T>::builder()
        .trait_name(trait_name)
        .stubs(true)
        .build(&tree)
        .gen(&mut out)
        .unwrap();
}

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    gen::<u16>(
        "RiscvDecode16",
        "src/target/riscv/insn16.decode",
        &format!("{out_dir}/target/riscv/insn16.rs"),
    );

    gen::<u32>(
        "RiscvDecode32",
        "src/target/riscv/insn32.decode",
        &format!("{out_dir}/target/riscv/insn32.rs"),
    );
}
