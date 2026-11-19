use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=csrc/zenoh_shim.c");
    println!("cargo:rerun-if-changed=csrc/macos_surface_ipc.m");

    // Locate Zenoh C headers & library from PIXI_PROJECT_ROOT, CONDA_PREFIX, or workspace default
    let pixi_prefix = env::var("CONDA_PREFIX")
        .or_else(|_| {
            env::var("PIXI_PROJECT_ROOT").map(|p| format!("{p}/.pixi/envs/default"))
        })
        .unwrap_or_else(|_| {
            // Walk up from external_renderer/bevy_renderer -> rotary/.pixi/envs/default
            let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
            manifest
                .ancestors()
                .find(|p| p.join(".pixi/envs/default/include/zenoh.h").exists())
                .map(|p| p.join(".pixi/envs/default").to_string_lossy().into_owned())
                .unwrap_or_else(|| "/Users/addisuzt/ws/rotary/.pixi/envs/default".to_string())
        });

    let zenoh_inc = PathBuf::from(&pixi_prefix).join("include");
    let zenoh_lib = PathBuf::from(&pixi_prefix).join("lib");

    // Compile thin Zenoh 1.8 C shim (zenoh_configure.h already enables SHM + unstable API)
    cc::Build::new()
        .file("csrc/zenoh_shim.c")
        .include(&zenoh_inc)
        .flag_if_supported("-O2")
        .compile("zenoh_shim");

    println!("cargo:rustc-link-search=native={}", zenoh_lib.display());
    println!("cargo:rustc-link-lib=dylib=zenohc");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", zenoh_lib.display());

    #[cfg(target_os = "macos")]
    {
        cc::Build::new()
            .file("csrc/macos_surface_ipc.m")
            .flag("-fobjc-arc")
            .flag("-Wno-deprecated-declarations")
            .flag("-O2")
            .compile("macos_surface_ipc");

        println!("cargo:rustc-link-lib=framework=Foundation");
        println!("cargo:rustc-link-lib=framework=CoreFoundation");
        println!("cargo:rustc-link-lib=framework=CoreVideo");
        println!("cargo:rustc-link-lib=framework=IOSurface");
        println!("cargo:rustc-link-lib=framework=Metal");
    }
}
