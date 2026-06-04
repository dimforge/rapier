use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    /* Create a horizontal cuboid. */
    let shape1 = Collider::cuboid(20.0, 10.0);
    let shape2 = Collider::ball(10.0);
    let pos1 = Vec2::ZERO;
    let pos2 = Vec2::new(0f32, -10f32);
    let rot1 = 0.0;
    let rot2 = 90_f32.to_radians();
    // DOCUSAURUS: ColliderCompound start
    commands.spawn(Collider::compound(vec![
        (pos1, rot1, shape1),
        (pos2, rot2, shape2),
    ]));
    // DOCUSAURUS: ColliderCompound stop
}
