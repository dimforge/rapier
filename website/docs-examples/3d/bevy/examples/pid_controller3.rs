use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, update_target)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

// DOCUSAURUS: Pid start
fn setup_physics(mut commands: Commands) {
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    let pid = PidController::new(60.0, 0.0, 0.8, AxesMask::LIN_AXES)
        .with_target(PidTarget::from_translation(Vec3::new(3.0, 2.0, 0.0)));

    commands.spawn((
        Transform::from_xyz(0.0, 1.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(0.5),
        pid,
    ));
}

/* Move the target of the controller inside of a system. */
fn update_target(time: Res<Time>, mut controllers: Query<&mut PidController>) {
    let t = time.elapsed_secs();
    for mut controller in controllers.iter_mut() {
        // The plugin drives the rigid-body toward this pose before each simulation step.
        controller.target = PidTarget::from_translation(Vec3::new(3.0 * t.cos(), 2.0, 0.0));
    }
}
// DOCUSAURUS: Pid stop
