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
        .add_systems(PostUpdate, display_events)
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2d);
}

pub fn display_events(
    mut collision_events: MessageReader<CollisionEvent>,
    mut contact_force_events: MessageReader<ContactForceEvent>,
) {
    for collision_event in collision_events.read() {
        println!("Received collision event: {collision_event:?}");
    }

    for contact_force_event in contact_force_events.read() {
        println!("Received contact force event: {contact_force_event:?}");
    }
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    commands.spawn((
        Transform::from_xyz(0.0, -24.0, 0.0),
        Collider::cuboid(80.0, 20.0),
    ));

    commands.spawn((
        Transform::from_xyz(0.0, 100.0, 0.0),
        Collider::cuboid(80.0, 30.0),
        Sensor,
    ));

    commands.spawn((
        Transform::from_xyz(0.0, 260.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(10.0, 10.0),
        ActiveEvents::COLLISION_EVENTS,
        ContactForceEventThreshold(10.0),
    ));
}
