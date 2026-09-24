//! Translated from rapier benchmark.
//!
//! <https://github.com/dimforge/rapier/blob/87ada34008f4a1a313ccf8c3040040bab4f10e69/benchmarks3d/many_pyramids3.rs>

mod common;

use bevy_rapier_benches3d::pyramids::setup_pyramids;
use common::bench_app_updates;

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_1_with_height_2(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_pyramids(app, 1, 2));
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_1_with_height_20(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_pyramids(app, 1, 20));
}

#[divan::bench(sample_count = 60, sample_size = 1)]
fn pyramid_2_with_height_20(bencher: divan::Bencher) {
    bench_app_updates(bencher, |app| setup_pyramids(app, 2, 20));
}

fn main() {
    // Run registered benchmarks.
    divan::main();
}
