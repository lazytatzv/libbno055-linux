fn main() {
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .include("../include")
        .file("../src/bno055.cpp")
        .file("../src/bno055_c.cpp")
        .compile("bno055-linux");

    println!("cargo:rerun-if-changed=../include/libbno055-linux/bno055_c.h");
    println!("cargo:rerun-if-changed=../include/libbno055-linux/bno055.hpp");
    println!("cargo:rerun-if-changed=../src/bno055.cpp");
    println!("cargo:rerun-if-changed=../src/bno055_c.cpp");
}
