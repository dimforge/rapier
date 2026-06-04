use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .run();
}

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
}
