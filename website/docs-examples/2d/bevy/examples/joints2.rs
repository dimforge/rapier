use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(10.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    let parent_entity = commands.spawn(RigidBody::Fixed).id();
    // DOCUSAURUS: FixedJoint start
    let joint = FixedJointBuilder::new().local_anchor1(Vec2::new(0.0, -20.0));
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(5f32, 5f32))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: FixedJoint stop

    // DOCUSAURUS: RevoluteJoint start
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vec2::new(0.0, 1.0))
        .local_anchor2(Vec2::new(0.0, -5.0));
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(5f32, 5f32))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: RevoluteJoint stop

    // DOCUSAURUS: PrismaticJoint start
    let joint = PrismaticJointBuilder::new(Vec2::X)
        .local_anchor1(Vec2::new(0.0, 1.0))
        .local_anchor2(Vec2::new(0.0, -3.0))
        .limits([-2.0, 5.0]);
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(5f32, 5f32))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: PrismaticJoint stop

    // DOCUSAURUS: Motor start
    let joint = PrismaticJointBuilder::new(Vec2::X)
        .local_anchor1(Vec2::new(0.0, 1.0))
        .local_anchor2(Vec2::new(0.0, -3.0))
        .motor_velocity(1.0, 1.0);
    commands
        .spawn(RigidBody::Dynamic)
        .insert(Collider::cuboid(5f32, 5f32))
        .insert(ImpulseJoint::new(parent_entity, joint));
    // DOCUSAURUS: Motor stop
}
