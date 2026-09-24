fn main() {
    let version = std::fs::read_to_string("../VERSION").expect("read C bindings VERSION");
    let version = version.trim();
    let rust_version = std::env::var("CARGO_PKG_VERSION").expect("Cargo provides package version");
    let prefix = format!("{rust_version}+c.");
    let revision = version
        .strip_prefix(&prefix)
        .and_then(|value| value.parse::<u32>().ok())
        .expect("C bindings VERSION must be <Rust crate version>+c.<revision>");
    assert_eq!(version, format!("{prefix}{revision}"), "invalid C revision");
    println!("cargo:rustc-env=RAPIER_C_VERSION={version}");
    println!("cargo:rerun-if-changed=../VERSION");

    // Report the library's Cargo profile, independent of its C/C++ consumer.
    let profile = std::env::var("PROFILE").expect("Cargo provides PROFILE");
    println!("cargo:rustc-env=RAPIER_CARGO_PROFILE={profile}");
    // A distributable dylib must not retain Cargo's absolute target/deps install name.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        let name = std::env::var("CARGO_PKG_NAME").unwrap().replace('-', "_");
        println!("cargo:rustc-link-arg-cdylib=-Wl,-install_name,@rpath/lib{name}.dylib");
    }
    println!("cargo:rerun-if-changed=../build.rs");
}
