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
            reattach_joints.run_if(input_just_pressed(KeyCode::KeyR)),
        )
        .add_systems(
            Update,
            disable_joints.run_if(input_just_pressed(KeyCode::KeyD)),
        )
        .add_systems(Update, break_joints)
        .run();
}

/// Marker of the rigid-body joints get re-attached to.
#[derive(Component)]
struct NewParent;

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, -10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    commands
        .spawn(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(Transform::from_xyz(0.0, -2.0, 0.0));

    let parent_entity = commands
        .spawn(RigidBody::Fixed)
        .insert(Transform::default().with_translation(Vec3::Y * 3f32))
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .id();
    // DOCUSAURUS: FixedJoint start
    let joint = FixedJointBuilder::new().local_anchor1(Vec3::new(0.0, 0.0, -2.0));
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: FixedJoint stop

    // DOCUSAURUS: SphericalJoint start
    let joint = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(0.0, 0.0, 1.0))
        .local_anchor2(Vec3::new(0.0, 0.0, -3.0));
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: SphericalJoint stop

    // DOCUSAURUS: RevoluteJoint start
    let x = Vec3::X;
    let joint = RevoluteJointBuilder::new(x)
        .local_anchor1(Vec3::new(0.0, 0.0, 1.0))
        .local_anchor2(Vec3::new(0.0, 0.0, -3.0));
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: RevoluteJoint stop

    // DOCUSAURUS: PrismaticJoint start
    let joint = PrismaticJointBuilder::new(Vec3::X)
        .local_anchor1(Vec3::new(0.0, 0.0, 1.0))
        .local_anchor2(Vec3::new(0.0, 1.0, -3.0))
        .limits([-2.0, 5.0]);
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(0.5, 0.5, 0.5))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: PrismaticJoint stop

    // DOCUSAURUS: Motor start
    let joint = PrismaticJointBuilder::new(Vec3::X)
        .local_anchor1(Vec3::new(0.0, 0.0, 1.0))
        .local_anchor2(Vec3::new(0.0, 0.0, -3.0))
        .motor_velocity(0.1, 0.05);
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::capsule_y(1f32, 0.5f32))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: Motor stop

    commands.spawn((RigidBody::Fixed, NewParent));

    // DOCUSAURUS: MultipleJoints start
    /* Attach two joints to the same rigid-body using child entities. */
    let body1 = commands.spawn(RigidBody::Fixed).id();
    let body2 = commands.spawn(RigidBody::Fixed).id();
    commands
        .spawn((RigidBody::Dynamic, Collider::cuboid(0.5, 0.5, 0.5)))
        .with_children(|children| {
            // The second rigid-body of these joints is the one of the parent entity.
            let joint1 =
                RevoluteJointBuilder::new(Vec3::Z).local_anchor2(Vec3::new(-1.0, 0.0, 0.0));
            let joint2 = RevoluteJointBuilder::new(Vec3::Z).local_anchor2(Vec3::new(1.0, 0.0, 0.0));
            children.spawn(ImpulseJoint::new(body1, joint1));
            children.spawn(ImpulseJoint::new(body2, joint2));
        });
    // DOCUSAURUS: MultipleJoints stop

    // DOCUSAURUS: Multibody start
    /* Build a chain of three links attached with multibody joints. */
    let mut parent = commands.spawn(RigidBody::Fixed).id();
    for i in 1..=3 {
        let joint = RevoluteJointBuilder::new(Vec3::Z).local_anchor2(Vec3::new(0.0, 2.0, 0.0));
        parent = commands
            .spawn((
                RigidBody::Dynamic,
                Collider::cuboid(0.5, 0.5, 0.5),
                Transform::from_xyz(0.0, -2.0 * i as f32, 0.0),
                MultibodyJoint::new(parent, joint),
            ))
            .id();
    }
    // DOCUSAURUS: Multibody stop
}

// DOCUSAURUS: JointComponents start
/* Re-attach the joints to another rigid-body inside of a system. */
fn reattach_joints(
    mut joints: Query<&mut ImpulseJoint>,
    new_parent: Query<Entity, With<NewParent>>,
) {
    let Ok(new_parent) = new_parent.single() else {
        return;
    };
    for mut joint in joints.iter_mut() {
        joint.parent = new_parent;
    }
}

/* Disable the joints inside of a system. */
fn disable_joints(mut commands: Commands, joints: Query<Entity, With<ImpulseJoint>>) {
    for entity in joints.iter() {
        // Removing this component enables the joint again.
        commands.entity(entity).insert(ImpulseJointDisabled);
    }
}

/* Break the joints applying a large impulse inside of a system. */
fn break_joints(mut commands: Commands, joints: Query<(Entity, &ImpulseJointImpulses)>) {
    for (entity, impulses) in joints.iter() {
        // The impulse applied by the joint along its locked translations during the last step.
        if impulses.linear.length() > 100.0 {
            commands.entity(entity).remove::<ImpulseJoint>();
        }
    }
}
// DOCUSAURUS: JointComponents stop
