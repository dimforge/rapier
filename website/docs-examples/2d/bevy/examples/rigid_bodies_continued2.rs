use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, modify_body_locked_flags)
        .add_systems(Update, modify_body_damping)
        .add_systems(Update, modify_body_dominance)
        .add_systems(Update, modify_body_ccd)
        .add_systems(Update, reset_position)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    // DOCUSAURUS: LockedAxes1 start
    /* Lock translations and/or rotations when the rigid-body bundle is created. */
    commands
        .spawn(RigidBody::Dynamic)
        .insert(LockedAxes::TRANSLATION_LOCKED);
    // DOCUSAURUS: LockedAxes1 stop
    // DOCUSAURUS: Damping1 start
    /* Set damping when the rigid-body bundle is created. */
    commands.spawn(RigidBody::Dynamic).insert(Damping {
        linear_damping: 0.5,
        angular_damping: 1.0,
    });
    // DOCUSAURUS: Damping1 stop
    // DOCUSAURUS: Dominance1 start
    /* Set dominance when the rigid-body bundle is created. */
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Dominance::group(10));
    // DOCUSAURUS: Dominance1 stop
    // DOCUSAURUS: Ccd1 start
    /* Enable CCD when the rigid-body bundle is created. */
    commands.spawn(RigidBody::Dynamic).insert(Ccd::enabled());
    // DOCUSAURUS: Ccd1 stop
}

// DOCUSAURUS: LockedAxes2 start
/* Lock translations and/or rotations inside of a system. */
fn modify_body_locked_flags(mut locked_axes: Query<&mut LockedAxes>) {
    for mut locked_axes in locked_axes.iter_mut() {
        *locked_axes = LockedAxes::ROTATION_LOCKED;
    }
}
// DOCUSAURUS: LockedAxes2 stop

// DOCUSAURUS: Damping2 start
/* Set damping inside of a system. */
fn modify_body_damping(mut dampings: Query<&mut Damping>) {
    for mut rb_damping in dampings.iter_mut() {
        rb_damping.linear_damping = 0.5;
        rb_damping.angular_damping = 1.0;
    }
}
// DOCUSAURUS: Damping2 stop

// DOCUSAURUS: Dominance2 start
/* Set dominance inside of a system. */
fn modify_body_dominance(mut dominances: Query<&mut Dominance>) {
    for mut rb_dominance in dominances.iter_mut() {
        rb_dominance.groups = 10;
    }
}
// DOCUSAURUS: Dominance2 stop

// DOCUSAURUS: Ccd2 start
/* Enable CCD inside of a system. */
fn modify_body_ccd(mut ccds: Query<&mut Ccd>) {
    for mut rb_ccd in ccds.iter_mut() {
        rb_ccd.enabled = true;
    }
}
// DOCUSAURUS: Ccd2 stop

/// System to avoid too much drift
fn reset_position(mut positions: Query<&mut Transform, With<RigidBody>>) {
    for mut position in positions.iter_mut() {
        if position.translation.y > 30.0 {
            position.translation.y -= 30.0;
        }
        if position.translation.x > 30.0 {
            position.translation.x -= 30.0;
        }
        if position.translation.z > 30.0 {
            position.translation.x -= 30.0;
        }
    }
}
