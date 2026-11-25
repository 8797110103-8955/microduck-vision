use std::fs;
use std::io;
use std::path::Path;

#[path = "../../training/package.rs"]
mod python_package;

fn main() -> io::Result<()> {
    let launcher = std::env::current_exe()?;
    let profile = launcher.parent().expect("Cargo profile directory");
    let output = Path::new(env!("DUCK_DESKTOP_RELEASE_DIR"));
    let executable = format!("microduck-vision{}", std::env::consts::EXE_SUFFIX);

    fs::create_dir_all(output)?;
    let staged = output.join(&executable);
    python_package::copy_bundle(&profile.join("python"), &output.join("python"))?;
    fs::copy(profile.join(executable), &staged)?;
    println!("Desktop release: {}", staged.display());
    Ok(())
}
