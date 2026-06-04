use rapier2d::prelude::*;

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut joint_set = ImpulseJointSet::new();
    let collider_handle = collider_set.insert(ColliderBuilder::ball(0.5));

    let body_handle1 = rigid_body_set.insert(RigidBodyBuilder::dynamic().build());
    let body_handle2 = rigid_body_set.insert(RigidBodyBuilder::dynamic().build());

    // DOCUSAURUS: FixedJoint start
    // NOTE: setting the local anchors sets the translation part of the local frames.
    let joint = FixedJointBuilder::new()
        .local_anchor1(point![0.0, 1.0])
        .local_anchor2(point![0.0, -3.0]);
    joint_set.insert(body_handle1, body_handle2, joint, true);
    // DOCUSAURUS: FixedJoint stop

    // DOCUSAURUS: RevoluteJoint start
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(point![0.0, 1.0])
        .local_anchor2(point![0.0, -3.0]);
    joint_set.insert(body_handle1, body_handle2, joint, true);
    // DOCUSAURUS: RevoluteJoint stop

    // DOCUSAURUS: PrismaticJoint start
    let x = Vector::x_axis();
    let mut joint = PrismaticJointBuilder::new(x)
        .local_anchor1(point![0.0, 1.0])
        .local_anchor2(point![0.0, -3.0])
        .limits([-2.0, 5.0]);
    joint_set.insert(body_handle1, body_handle2, joint, true);
    // DOCUSAURUS: PrismaticJoint stop

    // DOCUSAURUS: Motor start
    let x = Vector::x_axis();
    let mut joint = PrismaticJointBuilder::new(x)
        .local_anchor1(point![0.0, 1.0])
        .local_anchor2(point![0.0, -3.0])
        .motor_velocity(1.0, 0.5);
    joint_set.insert(body_handle1, body_handle2, joint, true);
    // DOCUSAURUS: Motor stop
}
