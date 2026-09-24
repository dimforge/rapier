use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

/// A snapshot of the default physics context.
#[derive(Resource)]
struct Snapshot(Vec<u8>);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, (save_snapshot, restore_snapshot))
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2d);
    commands.spawn((Transform::from_xyz(0.0, -100.0, 0.0), Collider::cuboid(500.0, 50.0)));
    commands.spawn((
        Transform::from_xyz(0.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(50.0),
    ));
}

// DOCUSAURUS: Serialization start
/// The components holding the complete state of the simulation of a physics context, and its
/// configuration.
type ContextComponents = (
    RapierContextSimulation,
    RapierContextColliders,
    RapierRigidBodySet,
    RapierContextJoints,
    RapierConfiguration,
);

/// Serializes the default physics context when S is pressed.
fn save_snapshot(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    context: Query<
        (
            &RapierContextSimulation,
            &RapierContextColliders,
            &RapierRigidBodySet,
            &RapierContextJoints,
            &RapierConfiguration,
        ),
        With<DefaultRapierContext>,
    >,
) {
    if keys.just_pressed(KeyCode::KeyS) {
        let components = context.single().unwrap();
        let serialized =
            bincode::serde::encode_to_vec(components, bincode::config::standard()).unwrap();
        commands.insert_resource(Snapshot(serialized));
    }
}

/// Restores the snapshot when R is pressed.
fn restore_snapshot(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    snapshot: Option<Res<Snapshot>>,
    context: Query<Entity, With<DefaultRapierContext>>,
) {
    if let (true, Some(snapshot)) = (keys.just_pressed(KeyCode::KeyR), snapshot) {
        // The maps from entities to handles are rebuilt automatically by the deserialization.
        let (components, _): (ContextComponents, usize) =
            bincode::serde::decode_from_slice(&snapshot.0, bincode::config::standard()).unwrap();
        // Replace the components of the physics context by the deserialized ones.
        commands.entity(context.single().unwrap()).insert(components);
    }
}
// DOCUSAURUS: Serialization stop
