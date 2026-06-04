use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Update, modify_collider_type)
        .add_systems(Update, modify_collider_position)
        .add_systems(Update, modify_collider_friction)
        .add_systems(Update, modify_collider_restitution)
        .add_systems(Update, modify_collider_groups)
        .add_systems(Update, modify_collider_active_collision_types)
        .add_systems(Update, modify_collider_active_events)
        .add_systems(Update, modify_collider_active_hooks)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    // DOCUSAURUS: Creation1 start
    use bevy_rapier2d::prelude::*;

    commands
        .spawn(Collider::cuboid(1.0, 2.0))
        .insert(Sensor)
        .insert(Transform::from_xyz(2.0, 0.0, 0.0))
        .insert(Friction::coefficient(0.7))
        .insert(Restitution::coefficient(0.3))
        .insert(ColliderMassProperties::Density(2.0));
    // DOCUSAURUS: Creation1 stop

    // DOCUSAURUS: Creation2 start
    // Attach a single collider to a rigid-body.
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::ball(0.5));

    // Attach a multiple colliders to a rigid-body.
    commands
        .spawn((RigidBody::Dynamic, GlobalTransform::default()))
        .with_children(|children| {
            children
                .spawn(Collider::ball(0.5))
                // Position the collider relative to the rigid-body.
                .insert(Transform::from_xyz(0.0, 0.0, -1.0));
            children
                .spawn(Collider::ball(0.5))
                // Position the collider relative to the rigid-body.
                .insert(Transform::from_xyz(0.0, 0.0, 1.0));
        });
    // DOCUSAURUS: Creation2 stop

    // DOCUSAURUS: ColliderType1 start
    /* Set the collider sensor when the collider is created. */
    commands.spawn(Collider::ball(0.5)).insert(Sensor);
    // DOCUSAURUS: ColliderType1 stop

    // DOCUSAURUS: Mass start
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let collider_mprops = ColliderMassProperties::Density(2.0);
    // Second option: by setting the mass of the collider.
    let collider_mprops = ColliderMassProperties::Mass(0.8);
    // Third option: by setting the mass-properties explicitly.
    let collider_mprops = ColliderMassProperties::MassProperties(MassProperties {
        local_center_of_mass: Vec2::new(0.0, 1.0),
        mass: 0.5,
        principal_inertia: 0.3,
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
        .spawn(Collider::cuboid(0.5, 0.5))
        .insert(Transform::from_xyz(1.0, 2.0, 0.0));
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    // Attach the collider to the rigid-body. The collider is attached as its
    // children, so the collider’s `Transform` components sets its position
    // relative to the parent rigid-body.
    commands
        .spawn((RigidBody::Dynamic, GlobalTransform::default()))
        .with_children(|children| {
            children
                .spawn(Collider::cuboid(0.5, 0.5))
                .insert(Transform::from_xyz(1.0, 2.0, 0.0));
        });
    // DOCUSAURUS: Position2 stop

    // DOCUSAURUS: Friction1 start
    /* Set the friction coefficient and friction combine rule
    when the collider is created. */
    commands.spawn(Collider::ball(0.5)).insert(Friction {
        coefficient: 0.7,
        combine_rule: CoefficientCombineRule::Min,
    });
    // DOCUSAURUS: Friction1 stop

    // DOCUSAURUS: Restitution1 start
    /* Set the restitution coefficient and restitution combine rule
    when the collider is created. */
    commands.spawn(Collider::ball(0.5)).insert(Restitution {
        coefficient: 0.7,
        combine_rule: CoefficientCombineRule::Min,
    });
    // DOCUSAURUS: Restitution1 stop

    // DOCUSAURUS: Groups1 start
    /* Set the collision and/or solver groups when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(CollisionGroups::new(
            Group::GROUP_1 | Group::GROUP_3 | Group::GROUP_4,
            Group::GROUP_3,
        ))
        .insert(SolverGroups::new(
            Group::GROUP_1 | Group::GROUP_2,
            Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_4,
        ));
    // DOCUSAURUS: Groups1 stop

    // DOCUSAURUS: ActiveCollisionTypes1 start
    /* Set the active collision types when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_STATIC);
    // DOCUSAURUS: ActiveCollisionTypes1 stop

    // DOCUSAURUS: ActiveEvents1 start
    /* Set the active events when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(ActiveEvents::COLLISION_EVENTS);
    // DOCUSAURUS: ActiveEvents1 stop

    // DOCUSAURUS: ActiveHooks1 start
    /* Set the active hooks when the collider is created. */
    commands
        .spawn(Collider::ball(0.5))
        .insert(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS);
    // DOCUSAURUS: ActiveHooks1 stop
}

// DOCUSAURUS: ColliderType2 start
/* Change the collider sensor status inside of a system. */
fn modify_collider_type(mut commands: Commands, sensors: Query<Entity, With<Sensor>>) {
    for entity in sensors.iter() {
        commands.entity(entity).remove::<Sensor>();
    }
}
// DOCUSAURUS: ColliderType2 stop

// DOCUSAURUS: Position3 start
/* Set the collider position inside of a system. */
fn modify_collider_position(mut positions: Query<&mut Transform, With<Collider>>) {
    for mut position in positions.iter_mut() {
        position.translation.x = 2.0;
    }
}
// DOCUSAURUS: Position3 stop

// DOCUSAURUS: Friction2 start
/* Set the friction coefficient and friction combine rule
inside of a system. */
fn modify_collider_friction(mut frictions: Query<&mut Friction>) {
    for mut friction in frictions.iter_mut() {
        friction.coefficient = 0.7;
        friction.combine_rule = CoefficientCombineRule::Min;
    }
}
// DOCUSAURUS: Friction2 stop

// DOCUSAURUS: Restitution2 start
/* Set the restitution coefficient and restitution combine rule
inside of a system. */
fn modify_collider_restitution(mut restitutions: Query<&mut Restitution>) {
    for mut restitution in restitutions.iter_mut() {
        restitution.coefficient = 0.7;
        restitution.combine_rule = CoefficientCombineRule::Min;
    }
}
// DOCUSAURUS: Restitution2 stop

// DOCUSAURUS: Groups2 start
/* Set the collision and/or solver groups inside of a system. */
fn modify_collider_groups(
    mut collision_groups: Query<&mut CollisionGroups>,
    mut solver_groups: Query<&mut SolverGroups>,
) {
    for mut collision_groups in collision_groups.iter_mut() {
        collision_groups.memberships = Group::GROUP_1 | Group::GROUP_3 | Group::GROUP_4;
        collision_groups.filters = Group::GROUP_3;
    }

    for mut solver_groups in solver_groups.iter_mut() {
        solver_groups.memberships = Group::GROUP_1 | Group::GROUP_2;
        solver_groups.filters = Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_4;
    }
}
// DOCUSAURUS: Groups2 stop

// DOCUSAURUS: ActiveCollisionTypes2 start
/* Set the active collision types inside of a system. */
fn modify_collider_active_collision_types(mut active_types: Query<&mut ActiveCollisionTypes>) {
    for mut active_types in active_types.iter_mut() {
        *active_types = (ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_STATIC);
    }
}
// DOCUSAURUS: ActiveCollisionTypes2 stop

// DOCUSAURUS: ActiveEvents2 start
/* Set the active events inside of a system. */
fn modify_collider_active_events(mut active_events: Query<&mut ActiveEvents>) {
    for mut active_events in active_events.iter_mut() {
        *active_events = ActiveEvents::COLLISION_EVENTS;
    }
}
// DOCUSAURUS: ActiveEvents2 stop

// DOCUSAURUS: ActiveHooks2 start
/* Set the active hooks inside of a system. */
fn modify_collider_active_hooks(mut active_hooks: Query<&mut ActiveHooks>) {
    for mut active_hooks in active_hooks.iter_mut() {
        *active_hooks = ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS;
    }
}
// DOCUSAURUS: ActiveHooks2 stop
