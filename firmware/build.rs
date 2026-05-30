use std::env;

fn main() {
    // Add the package directory to the linker search path so cortex-m-rt can
    // find memory.x via its `INCLUDE memory.x` directive in link.x.
    println!("cargo:rustc-link-search={}", env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rerun-if-changed=memory.x");
}
