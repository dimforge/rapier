use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 3.0, 0.0).looking_at(Vec3::new(0.0, 3.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * A rectangle that only rotates along the `x` axis.
     */
    commands.spawn((
        Transform::from_xyz(0.0, 3.0, 0.0),
        RigidBody::Dynamic,
        LockedAxes::TRANSLATION_LOCKED
            | LockedAxes::ROTATION_LOCKED_Y
            | LockedAxes::ROTATION_LOCKED_Z,
        Collider::cuboid(0.2, 0.6, 2.0),
    ));

    /*
     * A tilted cuboid that cannot rotate.
     */
    commands.spawn((
        Transform::from_xyz(0.0, 5.0, 0.0).with_rotation(Quat::from_rotation_x(1.0)),
        RigidBody::Dynamic,
        LockedAxes::ROTATION_LOCKED,
        Collider::cuboid(0.6, 0.4, 0.4),
    ));
}
