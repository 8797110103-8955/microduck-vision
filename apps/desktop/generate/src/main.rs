use std::{fs, io, path::Path};

#[path = "../../src-tauri/src/platform/state/types.rs"]
#[allow(dead_code)]
mod types;

#[allow(unused_imports, unused_variables)]
mod client {
    include!(concat!(env!("OUT_DIR"), "/client.rs"));
}

mod commands {
    include!(concat!(env!("OUT_DIR"), "/commands.rs"));
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../generated");
    fs::create_dir_all(&directory)?;
    let output = directory.join("types.ts");
    let temporary = Path::new(env!("OUT_DIR")).join(format!("types.{}.tmp.ts", std::process::id()));
    let result = (|| -> Result<bool, Box<dyn std::error::Error>> {
        commands::command_builder::<tauri::test::MockRuntime>()
            .export(specta_typescript::Typescript::default(), &temporary)?;
        let generated = standalone_commands(&fs::read_to_string(&temporary)?)?;
        fs::write(&temporary, &generated)?;
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
    println!(
        "generate-types: {}",
        if result? { "generated" } else { "unchanged" }
    );
    Ok(())
}

/// Specta emits a command object. Export its generated signatures as free functions so the
/// application can import IPC calls directly without maintaining a second bridge contract.
fn standalone_commands(source: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let (before, remaining) = source
        .split_once("export const commands = {\n")
        .ok_or("Specta command export was not found")?;
    let (commands, after) = remaining
        .split_once("\n};")
        .ok_or("Specta command export is incomplete")?;
    let mut output = before.to_owned();
    for command in commands.lines().filter(|line| !line.trim().is_empty()) {
        let (name, signature) = command.trim().split_once(": ").ok_or("invalid command")?;
        let (arguments, body) = signature
            .trim_end_matches(',')
            .split_once(" => ")
            .ok_or("unsupported Specta command implementation")?;
        output.push_str(&format!(
            "export function {name}{arguments} {{\n  return {body};\n}}\n"
        ));
    }
    output.push_str(after);
    Ok(output.into_bytes())
}
