use bevy_rapier_benches3d::{custom_bencher, pyramids::setup_pyramids};

fn pyramid_1_with_height_2() {
    custom_bencher(1000, |app| setup_pyramids(app, 1, 2));
}

fn pyramid_2_with_height_20() {
    custom_bencher(100, |app| setup_pyramids(app, 3, 20));
}

fn main() {
    pyramid_1_with_height_2();
    pyramid_2_with_height_20();
}
