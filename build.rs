#[cfg(feature = "target-riscv")]
fn gen<T>(trait_name: &str, path: &str, out: &str)
where
    T: Default + std::hash::Hash + std::fmt::LowerHex + Ord + decodetree::Insn,
{
    use std::{
        fs::{self, File},
        io::BufWriter,
        path::Path,
    };

    use decodetree::Generator;

    println!("cargo:rerun-if-changed={path}");
    let src = fs::read_to_string(path).unwrap();
    let tree = match decodetree::from_str::<T, decodetree::Str>(&src) {
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

    Generator::builder()
        .trait_name(trait_name)
        .stubs(true)
        .value_type("i32")
        .build(&tree, ())
        .generate(&mut out)
        .unwrap();
}

fn main() {
    #[cfg(feature = "target-riscv")]
    {
        let out_dir = std::env::var("OUT_DIR").unwrap();

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
}
