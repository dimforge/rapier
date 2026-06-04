use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, modify_collider_position)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    // DOCUSAURUS: Creation1 start
    use bevy_rapier3d::prelude::*;

    commands
        .spawn(Collider::cuboid(1.0, 2.0, 1.0))
        .insert(Sensor)
        .insert(Transform::from_xyz(2.0, 0.0, 0.0))
        .insert(Friction::coefficient(0.7))
        .insert(Restitution::coefficient(0.3))
        .insert(ColliderMassProperties::Density(2.0));
    // DOCUSAURUS: Creation1 stop

    // DOCUSAURUS: Mass start
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let collider_mprops = ColliderMassProperties::Density(2.0);
    // Second option: by setting the mass of the collider.
    let collider_mprops = ColliderMassProperties::Mass(0.8);
    // Third option: by setting the mass-properties explicitly.
    let collider_mprops = ColliderMassProperties::MassProperties(MassProperties {
        local_center_of_mass: Vec3::new(0.0, 1.0, 2.0),
        mass: 0.5,
        principal_inertia_local_frame: Quat::IDENTITY,
        principal_inertia: Vec3::new(0.3, 0.4, 0.5),
    });
    // When the collider is attached, the rigid-body's mass and angular
    // inertia will be automatically updated to take the collider into account.
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::ball(0.5))
        .insert(collider_mprops);
    // DOCUSAURUS: Mass stop

    // DOCUSAURUS: Position1 start
    /* Set the collider position when the collider is created. */
    commands
        .spawn(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(Transform::from_xyz(1.0, 2.0, 3.0));
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position3 start
    // Attach the collider to the rigid-body. The collider is attached as its
    // children, so the collider’s `Transform` components sets its position
    // relative to the parent rigid-body.
    commands
        .spawn(RigidBody::Dynamic)
        .with_children(|children| {
            children
                .spawn(Collider::cuboid(0.5, 0.5, 0.5))
                .insert(Transform::from_xyz(1.0, 2.0, 0.0));
        });
    // DOCUSAURUS: Position3 stop
}

// DOCUSAURUS: Position2 start
/* Set the collider position inside of a system. */
fn modify_collider_position(mut positions: Query<&mut Transform, With<Collider>>) {
    for mut position in positions.iter_mut() {
        position.translation.x = 2.0;
    }
}
// DOCUSAURUS: Position2 stop
