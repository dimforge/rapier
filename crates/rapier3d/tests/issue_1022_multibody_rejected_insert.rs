use rapier3d::prelude::*;

fn joint() -> RevoluteJointBuilder {
    RevoluteJointBuilder::new(Vector::Z)
}

#[test]
fn self_joint_is_rejected() {
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    let body = bodies.insert(RigidBodyBuilder::dynamic());
    assert!(joints.insert(body, body, joint(), true).is_none());
    assert_eq!(joints.multibodies().count(), 0);
    assert!(joints.rigid_body_link(body).is_none());
}

#[test]
fn rejected_insert_leaves_topology_unchanged() {
    let mut bodies = RigidBodySet::new();
    let mut joints = MultibodyJointSet::new();
    let root = bodies.insert(RigidBodyBuilder::dynamic());
    let child = bodies.insert(RigidBodyBuilder::dynamic());
    assert!(joints.insert(root, child, joint(), true).is_some());
    assert_eq!(joints.multibodies().count(), 1);

    // `child` already has a parent, so attaching it again must fail without side effects.
    for _ in 0..10 {
        let fresh = bodies.insert(RigidBodyBuilder::dynamic());
        assert!(joints.insert(fresh, child, joint(), true).is_none());
        assert!(joints.rigid_body_link(fresh).is_none());
    }
    assert_eq!(joints.multibodies().count(), 1);

    // A loop inside one multibody is rejected too.
    assert!(joints.insert(child, root, joint(), true).is_none());
    assert_eq!(joints.multibodies().count(), 1);
}
