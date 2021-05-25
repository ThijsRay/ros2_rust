extern crate bindgen;

use bindgen::*;
use std::env;
use std::path::PathBuf;

fn main() {
    let mut builder = Builder::default();

    // Add all possible ROS locations to the library search and link paths.
    let ros_location_key = "AMENT_PREFIX_PATH";
    let ros_locations = std::env::var(ros_location_key);

    if let Ok(ros_paths) = ros_locations {
        for ros_path in ros_paths.split(':') {
            builder = builder.clang_arg(format!("-I{}/include", ros_path));
            println!("cargo:rustc-link-search=native={}/lib", ros_path);
        }
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rcutils");

    // Tell cargo to invalidate the built crate whenever the wrapper changes
    println!("cargo:rerun-if-changed=wrapper.h");

    let bindings = builder
        // The input header we would like to generate
        // bindings for.
        .header("wrapper.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(CargoCallbacks))
        .default_enum_style(EnumVariation::Rust {
            non_exhaustive: false,
        })
        .default_alias_style(AliasVariation::TypeAlias)
        .derive_copy(false)
        .rustfmt_bindings(true)
        .dynamic_link_require_all(true)
        .translate_enum_integer_types(true)
        .generate_comments(false)
        .size_t_is_usize(true)
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
