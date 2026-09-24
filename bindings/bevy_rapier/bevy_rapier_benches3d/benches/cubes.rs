//! Translated from avian benchmark.
//!
//! <https://github.com/Jondolf/avian/blob/81290423e146264120cf9711af716f6faf669717/crates/avian3d/benches/cubes.rs>

mod common;

use common::bench_app_updates;

use bevy::prelude::*;
use bevy_rapier3d::math::*;
use bevy_rapier3d::prelude::*;

fn setup_cubes(app: &mut App, size: u32) {
    app.add_systems(Startup, move |mut commands: Commands| {
        commands.spawn((
            RigidBody::Fixed,
            Transform::from_translation(-2.0 * Vect::Z),
            Collider::cuboid(100.0, 1.0, 100.0),
        ));
        for x in 0..size {
            for z in 0..size {
                commands.spawn((
                    RigidBody::Dynamic,
                    Transform::from_translation(Vec3::new(x as f32, 2.0, z as f32)),
                    Collider::cuboid(1.0, 1.0, 1.0),
                ));
            }
        }
    });
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn cubes_3x3(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 3))
}
#[divan::bench(sample_count = 60, sample_size = 1)]
fn cubes_5x5(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 5))
}
#[divan::bench(sample_count = 60, sample_size = 1)]
fn cubes_10x10(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 10))
}
#[divan::bench(sample_count = 60, sample_size = 1)]
fn cubes_20x20(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_cubes(app, 20))
}

fn main() {
    // Run registered benchmarks.
    divan::main();
}
