//! Replay trusted C-testbed snapshots with the native Rust API, in the same Cargo
//! configuration. See c/testbed/tools/README.md. This is a manual benchmark.
use bincode::Options;
use rapier::prelude::*;
use std::{path::Path, time::Instant};

fn read(path: &Path) -> PhysicsWorld {
    let data = std::fs::read(path).unwrap();
    assert_eq!(&data[..3], b"RPR");
    assert_eq!(data[3], 1);
    assert_eq!(data[4], rapier::math::DIM as u8);
    assert_eq!(data[5], std::mem::size_of::<Real>() as u8);
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(256 * 1024 * 1024)
        .reject_trailing_bytes()
        .deserialize(&data[6..])
        .unwrap()
}
fn main() {
    let args: Vec<_> = std::env::args_os().collect();
    assert_eq!(args.len(), 3, "arguments: INITIAL_SNAPSHOT FINAL_SNAPSHOT");
    let mut world = read(Path::new(&args[1]));
    let expected = read(Path::new(&args[2]));
    #[cfg(feature = "parallel")]
    world.configure_thread_pool(1).unwrap();
    world.physics_pipeline.counters.enable();
    let mut wall = 0.0;
    let mut engine = 0.0;
    for step in 0..420 {
        let start = Instant::now();
        world.step();
        let elapsed = start.elapsed().as_secs_f64() * 1000.0;
        if step >= 120 {
            wall += elapsed;
            engine += world.physics_pipeline.counters.step_time_ms();
        }
    }
    assert_eq!(world.bodies.len(), expected.bodies.len());
    let mut active = 0;
    // Exact final poses and velocities verify the same physical trajectory.
    for (handle, body) in world.bodies.iter() {
        let other = &expected.bodies[handle];
        assert_eq!(body.position(), other.position(), "pose {handle:?}");
        assert_eq!(body.linvel(), other.linvel(), "linear velocity {handle:?}");
        assert_eq!(body.angvel(), other.angvel(), "angular velocity {handle:?}");
        if body.is_dynamic() {
            assert!(!body.is_sleeping());
            assert!(body.activation().normalized_linear_threshold < 0.0);
            active += 1;
        }
    }
    println!(
        "Rust bodies={} awake_dynamic={} SIMD={} workers=1 warmup=120 measured=300 wall_ms={:.6} engine_ms={:.6} final_state=exact_match",
        world.bodies.len(),
        active,
        rapier::math::SIMD_WIDTH,
        wall / 300.0,
        engine / 300.0
    );
}
