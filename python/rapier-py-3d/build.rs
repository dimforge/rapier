//! Phase 11 — translate the `RAPIER_PY_DETERMINISM` env var into a build-time
//! hint, and detect mismatches between the env var and the `determinism`
//! Cargo feature.
//!
//! Effective protocol:
//!   - `RAPIER_PY_DETERMINISM=1` without `-F determinism` → warn that the
//!     env var is ignored unless the feature is also enabled.
//!   - With both → no-op (the feature flag does the work upstream).
//!   - With neither → fast path with platform-native math.

fn main() {
    println!("cargo:rerun-if-env-changed=RAPIER_PY_DETERMINISM");
    let env_set = std::env::var("RAPIER_PY_DETERMINISM").is_ok();
    let feature_on = std::env::var("CARGO_FEATURE_DETERMINISM").is_ok();
    if env_set && !feature_on {
        println!(
            "cargo:warning=RAPIER_PY_DETERMINISM is set but the 'determinism' \
             Cargo feature is not enabled — the env var is informational only. \
             Re-build with `maturin build --release -F determinism` (or set \
             MATURIN_PEP517_ARGS=\"--release -F determinism\") to actually \
             switch on enhanced-determinism in rapier."
        );
    }
    if feature_on {
        println!("cargo:rustc-cfg=rapier_py_determinism");
    }
}
