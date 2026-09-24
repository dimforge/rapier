use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

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
            // Run the physics systems in `FixedUpdate` instead of `PostUpdate`.
            RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule(),
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
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, -2.0, 0.0),
        Collider::cuboid(100.0, 0.1, 100.0),
    ));
    commands.spawn((
        Transform::from_xyz(0.0, 4.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.5),
    ));
}
