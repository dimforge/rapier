use bevy::app::App;
use bevy_rapier_benches3d::common::{default_app, wait_app_start};

pub fn bench_app_updates(bencher: divan::Bencher, setup: impl Fn(&mut App)) {
    let mut app = default_app();
    setup(&mut app);
    wait_app_start(&mut app);

    bencher.bench_local(|| {
        app.update();
    });
}
