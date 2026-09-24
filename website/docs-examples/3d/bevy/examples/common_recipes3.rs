use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

// DOCUSAURUS: OneWayPlatform start
#[derive(Component)]
struct OneWayPlatform;

#[derive(SystemParam)]
struct OneWayPlatformHooks<'w, 's> {
    platforms: Query<'w, 's, (), With<OneWayPlatform>>,
}

impl BevyPhysicsHooks for OneWayPlatformHooks<'_, '_> {
    fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
        // Keep only the contacts pushing along the local +y axis of the platform; the other
        // ones (the character arriving from below) are discarded. The normal is expressed in
        // the frame of the first collider of the pair, hence the flip.
        let allowed_local_n1 = if self.platforms.contains(context.collider1()) {
            Vec3::Y
        } else if self.platforms.contains(context.collider2()) {
            -Vec3::Y
        } else {
            return;
        };
        context.update_as_oneway_platform(allowed_local_n1, 0.1);
    }
}
// DOCUSAURUS: OneWayPlatform stop

#[allow(dead_code)]
// DOCUSAURUS: ConveyorBelt start
#[derive(SystemParam)]
struct ConveyorBeltHooks<'w, 's> {
    belts: Query<'w, 's, (), With<ConveyorBelt>>,
}

#[derive(Component)]
struct ConveyorBelt;

impl BevyPhysicsHooks for ConveyorBeltHooks<'_, '_> {
    fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
        if !self.belts.contains(context.collider1()) && !self.belts.contains(context.collider2())
        {
            return;
        }
        if let Some(contacts) = context.solver_contacts_mut() {
            for contact in contacts.iter_mut() {
                // The belt drags the objects along the world-space z axis at 12 m/s.
                contact.tangent_velocity.z = 12.0;
            }
        }
    }
}
// DOCUSAURUS: ConveyorBelt stop

#[derive(Component)]
struct MovingPlatform;

fn main() {
    App::new()
        // DOCUSAURUS: HooksPlugin start
        .add_plugins((
            DefaultPlugins,
            // The hooks are given to the physics plugin.
            RapierPhysicsPlugin::<OneWayPlatformHooks>::default(),
        ))
        // DOCUSAURUS: HooksPlugin stop
        .add_systems(
            Startup,
            (
                setup_one_way_platform,
                setup_moving_platform,
                setup_conveyor_belt,
            ),
        )
        .add_systems(Update, move_platform)
        .run();
}

// DOCUSAURUS: Hooks start
fn setup_one_way_platform(mut commands: Commands) {
    // The hooks are only called for the colliders asking for them.
    commands.spawn((
        Transform::from_xyz(0.0, 1.0, 0.0),
        Collider::cuboid(2.0, 0.1, 2.0),
        ActiveHooks::MODIFY_SOLVER_CONTACTS,
        OneWayPlatform,
    ));
}
// DOCUSAURUS: Hooks stop

fn setup_conveyor_belt(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, 0.0, -5.0),
        Collider::cuboid(2.0, 0.1, 2.0),
        ActiveHooks::MODIFY_SOLVER_CONTACTS,
        ConveyorBelt,
    ));
}

// DOCUSAURUS: MovingPlatform start
fn setup_moving_platform(mut commands: Commands) {
    commands.spawn((
        RigidBody::KinematicPositionBased,
        Transform::from_xyz(0.0, 1.0, 5.0),
        Collider::cuboid(2.0, 0.1, 2.0),
        MovingPlatform,
    ));
}

fn move_platform(time: Res<Time>, mut platforms: Query<&mut Transform, With<MovingPlatform>>) {
    for mut transform in platforms.iter_mut() {
        // Setting the next position of the platform: the physics engine derives the
        // velocity needed to reach it at the end of the next timestep.
        transform.translation.x = time.elapsed_secs().sin() * 2.0;
    }
}
// DOCUSAURUS: MovingPlatform stop
