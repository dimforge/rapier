use bevy::{app::PluginsState, prelude::*, time::TimeUpdateStrategy};
use bevy_rapier3d::prelude::*;

pub fn default_app() -> App {
    let mut app = App::new();

    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        RapierPhysicsPlugin::<()>::default(),
    ));

    // 60 physics
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1f32 / 60f32),
    ));
    app
}

pub fn wait_app_start(app: &mut App) {
    while app.plugins_state() != PluginsState::Ready {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }

    app.finish();
    app.cleanup();
}
