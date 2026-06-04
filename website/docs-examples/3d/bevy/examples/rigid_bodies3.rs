use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(
            Update,
            modify_body_velocity.run_if(input_just_pressed(KeyCode::KeyV)),
        )
        .add_systems(
            Update,
            modify_body_translation.run_if(input_just_pressed(KeyCode::KeyT)),
        )
        .add_systems(
            Update,
            modify_body_gravity_scale.run_if(input_just_pressed(KeyCode::KeyG)),
        )
        .add_systems(Update, reset_position)
        .add_systems(
            Update,
            apply_forces.run_if(input_just_pressed(KeyCode::KeyF)),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 10.0, 30.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    // DOCUSAURUS: Creation start
    use bevy::prelude::*;
    use bevy_rapier3d::prelude::*;

    // DOCUSAURUS: Position1 start
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Transform::from_xyz(0.0, 0.0, 0.0))
        // DOCUSAURUS: Position1 stop
        .insert(Velocity {
            linvel: Vec3::new(0.0, 2.0, 0.0),
            angvel: Vec3::new(0.2, 0.0, 0.0),
        })
        .insert(GravityScale(0.5))
        .insert(Sleeping::disabled())
        .insert(Ccd::enabled());
    // DOCUSAURUS: Creation stop

    // FIXME: this shows we can set a rotation but the resulting object does not rotate
    // (it needs a collider (or maybe justmass?))

    // DOCUSAURUS: Velocity1 start
    commands.spawn(RigidBody::Dynamic).insert(Velocity {
        linvel: Vec3::new(0.0, 2.0, 0.0),
        angvel: Vec3::new(0.2, 0.4, 0.8),
    });
    // DOCUSAURUS: Velocity1 stop

    // DOCUSAURUS: Gravity1 start
    /* Set the gravity scale when the rigid-body is created. */
    commands.spawn(RigidBody::Dynamic).insert(GravityScale(2.0));
    // DOCUSAURUS: Gravity1 stop

    // DOCUSAURUS: Forces1 start
    /* Apply forces when the rigid-body is created. */
    commands
        .spawn(RigidBody::Dynamic)
        .insert(ExternalForce {
            force: Vec3::new(10.0, 20.0, 30.0),
            torque: Vec3::new(1.0, 2.0, 3.0),
        })
        .insert(ExternalImpulse {
            impulse: Vec3::new(1.0, 2.0, 3.0),
            torque_impulse: Vec3::new(0.1, 0.2, 0.3),
        });
    // DOCUSAURUS: Forces1 stop
}

// DOCUSAURUS: Position2 start
/* Change the position inside of a system. */
fn modify_body_translation(mut positions: Query<&mut Transform, With<RigidBody>>) {
    for mut position in positions.iter_mut() {
        position.translation.y += 0.1;
    }
}
// DOCUSAURUS: Position2 stop

// DOCUSAURUS: Velocity2 start
/* Set the velocities inside of a system. */
fn modify_body_velocity(mut velocities: Query<&mut Velocity>) {
    for mut vel in velocities.iter_mut() {
        vel.linvel = Vec3::new(0.0, 2.0, 0.0);
        vel.angvel = Vec3::new(3.2, 0.4, 0.8);
    }
}
// DOCUSAURUS: Velocity2 stop

// DOCUSAURUS: Gravity2 start
/* Set the gravity scale inside of a system. */
fn modify_body_gravity_scale(mut grav_scale: Query<&mut GravityScale>) {
    for mut grav_scale in grav_scale.iter_mut() {
        grav_scale.0 = 2.0;
    }
}
// DOCUSAURUS: Gravity2 stop

// DOCUSAURUS: Forces2 start
/* Apply forces and impulses inside of a system. */
fn apply_forces(
    mut ext_forces: Query<&mut ExternalForce>,
    mut ext_impulses: Query<&mut ExternalImpulse>,
) {
    // Apply forces.
    for mut ext_force in ext_forces.iter_mut() {
        ext_force.force = Vec3::new(1000.0, 2000.0, 3000.0);
        ext_force.torque = Vec3::new(0.4, 0.5, 0.6);
    }

    // Apply impulses.
    for mut ext_impulse in ext_impulses.iter_mut() {
        ext_impulse.impulse = Vec3::new(100.0, 200.0, 300.0);
        ext_impulse.torque_impulse = Vec3::new(0.4, 0.5, 0.6);
    }
}
// DOCUSAURUS: Forces2 stop

/// to avoid too much drift
fn reset_position(mut positions: Query<&mut Transform, With<RigidBody>>) {
    for mut position in positions.iter_mut() {
        const OFFSET: f32 = 20.0;
        if position.translation.y > OFFSET {
            position.translation.y -= OFFSET;
        }
        if position.translation.x > OFFSET {
            position.translation.x -= OFFSET;
        }
        if position.translation.z > OFFSET {
            position.translation.x -= OFFSET;
        }
    }
}
