use rapier2d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();
    let collider_handle = world.insert_collider(ColliderBuilder::ball(0.5), None);

    let body_handle1 = world.insert_body(RigidBodyBuilder::dynamic().build());
    let body_handle2 = world.insert_body(RigidBodyBuilder::dynamic().build());

    // DOCUSAURUS: FixedJoint start
    // NOTE: setting the local anchors sets the translation part of the local frames.
    let joint = FixedJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 1.0))
        .local_anchor2(Vector::new(0.0, -3.0));
    world.insert_impulse_joint(body_handle1, body_handle2, joint);
    // DOCUSAURUS: FixedJoint stop

    // DOCUSAURUS: RevoluteJoint start
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 1.0))
        .local_anchor2(Vector::new(0.0, -3.0));
    world.insert_impulse_joint(body_handle1, body_handle2, joint);
    // DOCUSAURUS: RevoluteJoint stop

    // DOCUSAURUS: PrismaticJoint start
    let x = Vector::X;
    let mut joint = PrismaticJointBuilder::new(x)
        .local_anchor1(Vector::new(0.0, 1.0))
        .local_anchor2(Vector::new(0.0, -3.0))
        .limits([-2.0, 5.0]);
    world.insert_impulse_joint(body_handle1, body_handle2, joint);
    // DOCUSAURUS: PrismaticJoint stop

    // DOCUSAURUS: Motor start
    let x = Vector::X;
    let mut joint = PrismaticJointBuilder::new(x)
        .local_anchor1(Vector::new(0.0, 1.0))
        .local_anchor2(Vector::new(0.0, -3.0))
        .motor_velocity(1.0, 0.5);
    world.insert_impulse_joint(body_handle1, body_handle2, joint);
    // DOCUSAURUS: Motor stop
}
