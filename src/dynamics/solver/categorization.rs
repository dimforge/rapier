use crate::dynamics::{JointGraphEdge, JointIndex, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

pub(crate) fn categorize_contacts(
    manifolds: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    out_ground: &mut Vec<ContactManifoldIndex>,
    out_not_ground: &mut Vec<ContactManifoldIndex>,
) {
    for manifold_i in manifold_indices {
        let manifold = &manifolds[*manifold_i];

        if manifold.data.relative_dominance != 0 {
            out_ground.push(*manifold_i)
        } else {
            out_not_ground.push(*manifold_i)
        }
    }
}

pub(crate) fn categorize_joints(
    bodies: &RigidBodySet,
    joints: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    ground_joints: &mut Vec<JointIndex>,
    nonground_joints: &mut Vec<JointIndex>,
) {
    for joint_i in joint_indices {
        let joint = &joints[*joint_i].weight;
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];

        if !rb1.is_dynamic() || !rb2.is_dynamic() {
            ground_joints.push(*joint_i);
        } else {
            nonground_joints.push(*joint_i);
        }
    }
}
