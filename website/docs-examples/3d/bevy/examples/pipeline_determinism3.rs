use bevy::prelude::*;
use bevy_rapier3d::na::ComplexField;
use bevy_rapier3d::prelude::*;

fn main() {
    // DOCUSAURUS: DeterministicSetup start
    App::new()
        .add_plugins(DefaultPlugins)
        // Run `FixedUpdate` 60 times per second, and advance the simulation by exactly 1/60
        // seconds at each of these updates, whatever the frame rate.
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 60.0,
            substeps: 1,
        })
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        // DOCUSAURUS: DeterministicSetup stop
        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_physics(mut commands: Commands) {
    // DOCUSAURUS: Determinism start
    // WRONG version:
    // The following will not work cross-platform-deterministically because the values
    // given to `Transform::from_xyz` won't be cross-platform deterministic.
    commands.spawn((
        Transform::from_xyz(1.0f32.sqrt(), 2.0f32.sin(), 3.0f32.cos()),
        Collider::ball(0.5),
    ));

    // CORRECT version:
    // The following will work cross-platform-deterministically because we use the
    // functions from nalgebra.
    commands.spawn((
        Transform::from_xyz(
            ComplexField::sqrt(1.0),
            ComplexField::sin(2.0),
            ComplexField::cos(3.0),
        ),
        Collider::ball(0.5),
    ));
    // DOCUSAURUS: Determinism stop
}
