//! Example for RapierContext serialization, run with `--features serde-serialize`.

use bevy::prelude::*;
use bevy::MinimalPlugins;
use bevy_rapier2d::prelude::*;

/// Note: This will end up in duplication for testbed, but that's more simple.
mod joints2;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            MinimalPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
        ))
        .add_systems(Startup, joints2::setup_physics)
        .add_systems(PostUpdate, print_physics)
        .add_systems(Last, quit)
        .run();
}

pub fn print_physics(_context: ReadRapierContext) {
    #[cfg(feature = "serde-serialize")]
    println!(
        "{}",
        serde_json::to_string_pretty(&_context.single().expect("No rapier context"))
            .expect("Unable to serialize `RapierContext`")
    );
    #[cfg(not(feature = "serde-serialize"))]
    panic!("Example 'serialization' should be run with '--features serde-serialize'.");
}

fn quit(mut exit_event: MessageWriter<AppExit>) {
    exit_event.write(AppExit::Success);
}
