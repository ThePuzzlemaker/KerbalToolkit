fn main() {
    cxx_build::bridge("src/lib.rs")
        .file("src/bindings.cc")
        .std("c++17")
        //TODO: pkgbuild for `eigen3` and `ipopt`
        .include("/usr/include/eigen3")
        .include("/usr/include/coin-or")
        .compile("pagmo2");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/bindings.cc");
    println!("cargo:rerun-if-changed=src/bindings.hpp");
    println!("cargo:rerun-if-changed=src/common.hpp");
    println!("cargo::rustc-link-lib=pagmo");
}
