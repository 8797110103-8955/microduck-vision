use std::path::PathBuf;

pub fn test_directory() -> PathBuf {
    let directory = PathBuf::from(env!("MICRODUCK_TEST_DIR"));
    std::fs::create_dir_all(&directory).expect("create test output directory");
    directory
}
