use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let root_dir = manifest_dir.parent().unwrap();

    let include_dir = root_dir.join("include");
    let src_bno055 = root_dir.join("src").join("core").join("bno055.cpp");
    let src_bno055_c = root_dir.join("src").join("core").join("bno055_c.cpp");

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .include(&include_dir)
        .file(&src_bno055)
        .file(&src_bno055_c)
        .compile("bno055-linux");

    println!("cargo:rerun-if-changed={}", root_dir.join("include").join("libbno055-linux").join("bno055_c.h").display());
    println!("cargo:rerun-if-changed={}", root_dir.join("include").join("libbno055-linux").join("bno055.hpp").display());
    println!("cargo:rerun-if-changed={}", src_bno055.display());
    println!("cargo:rerun-if-changed={}", src_bno055_c.display());
}
