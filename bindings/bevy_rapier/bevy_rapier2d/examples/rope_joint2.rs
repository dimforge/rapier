use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((Camera2d, Transform::from_xyz(0.0, -200.0, 0.0)));
}

pub fn setup_physics(mut commands: Commands) {
    let rad = 10.0;
    let rope_length = rad * 10.0;

    let parent = commands
        .spawn((
            Transform::from_xyz(0.0, 0.0, 0.0),
            RigidBody::Fixed,
            Collider::cuboid(rad, rad),
        ))
        .id();

    let joint = RopeJointBuilder::new(rope_length).local_anchor2(Vec2::new(rope_length, 0.0));

    commands
        .spawn((
            Transform::from_xyz(-rad * 2.0, 0.0, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(rad, rad),
        ))
        .insert(ImpulseJoint::new(parent, joint));
}
