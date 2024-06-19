use std::io::Result;

fn main() -> Result<()> {
    prost_build::compile_protos(&["src/krpc/krpc.proto"], &["src/krpc/"])?;
    Ok(())
}
