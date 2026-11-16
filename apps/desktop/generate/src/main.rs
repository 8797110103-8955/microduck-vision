use std::{fs, io, path::Path};

extern crate self as duck_body;
#[path = "../../../../crates/duck-body/src/types.rs"]
mod duck_body_types;
pub use duck_body_types::*;

#[path = "../../src-tauri/src/types.rs"]
#[allow(dead_code)]
mod types;
#[path = "../../src-tauri/src/commands.rs"]
mod commands;

#[allow(unused_imports, unused_variables)]
mod client {
    include!(concat!(env!("OUT_DIR"), "/client.rs"));
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../generated");
    fs::create_dir_all(&directory)?;
    let output = directory.join("types.ts");
    let temporary = directory.join(format!(".types.{}.tmp.ts", std::process::id()));
    let result = (|| -> Result<bool, Box<dyn std::error::Error>> {
        commands::command_builder::<tauri::test::MockRuntime>()
            .export(specta_typescript::Typescript::default(), &temporary)?;
        let generated = fs::read(&temporary)?;
        match fs::read(&output) {
            Ok(current) if current == generated => return Ok(false),
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        fs::rename(&temporary, &output)?;
        Ok(true)
    })();
    if temporary.exists() {
        fs::remove_file(temporary)?;
    }
    println!("generate-types: {}", if result? { "generated" } else { "unchanged" });
    Ok(())
}
