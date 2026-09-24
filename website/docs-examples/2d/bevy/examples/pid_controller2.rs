use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, update_target)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

// DOCUSAURUS: Pid start
fn setup_physics(mut commands: Commands) {
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    let pid = PidController::new(60.0, 0.0, 0.8, AxesMask::LIN_AXES)
        .with_target(PidTarget::from_translation(Vec2::new(300.0, 200.0)));

    commands.spawn((
        Transform::from_xyz(0.0, 100.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(50.0),
        pid,
    ));
}

/* Move the target of the controller inside of a system. */
fn update_target(time: Res<Time>, mut controllers: Query<&mut PidController>) {
    let t = time.elapsed_secs();
    for mut controller in controllers.iter_mut() {
        // The plugin drives the rigid-body toward this pose before each simulation step.
        controller.target = PidTarget::from_translation(Vec2::new(300.0 * t.cos(), 200.0));
    }
}
// DOCUSAURUS: Pid stop
