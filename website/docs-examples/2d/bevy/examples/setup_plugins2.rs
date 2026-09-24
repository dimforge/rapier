use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    // DOCUSAURUS: Plugins start
    App::new()
        .add_plugins(DefaultPlugins)
        // Advance the simulation by the same dt at each run of the `FixedUpdate` schedule
        // (which runs 64 times per second by default).
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 64.0,
            substeps: 1,
        })
        .add_plugins((
            // 100 pixels make one meter, and the physics systems run in `FixedUpdate` instead
            // of `PostUpdate`.
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0).in_fixed_schedule(),
            // Draw what the physics engine sees.
            RapierDebugRenderPlugin::default(),
            // Measure the simulation with Bevy's diagnostics (must run in the physics schedule).
            RapierDiagnosticsPlugin::default().in_schedule(FixedUpdate),
        ))
        // DOCUSAURUS: Plugins stop
        .add_systems(Startup, setup)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
    commands.spawn((
        Transform::from_xyz(0.0, -100.0, 0.0),
        Collider::cuboid(500.0, 50.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(50.0),
    ));
}
