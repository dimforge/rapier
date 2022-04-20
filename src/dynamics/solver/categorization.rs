use crate::dynamics::{JointGraphEdge, JointIndex, MultibodyJointSet, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

pub(crate) fn categorize_contacts(
    _bodies: &RigidBodySet, // Unused but useful to simplify the parallel code.
    multibody_joints: &MultibodyJointSet,
    manifolds: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    out_ground: &mut Vec<ContactManifoldIndex>,
    out_not_ground: &mut Vec<ContactManifoldIndex>,
    out_generic_ground: &mut Vec<ContactManifoldIndex>,
    out_generic_not_ground: &mut Vec<ContactManifoldIndex>,
) {
    for manifold_i in manifold_indices {
        let manifold = &manifolds[*manifold_i];

        if manifold
            .data
            .rigid_body1
            .and_then(|h| multibody_joints.rigid_body_link(h))
            .is_some()
            || manifold
                .data
                .rigid_body2
                .and_then(|h| multibody_joints.rigid_body_link(h))
                .is_some()
        {
            if manifold.data.relative_dominance != 0 {
                out_generic_ground.push(*manifold_i);
            } else {
                out_generic_not_ground.push(*manifold_i);
            }
        } else if manifold.data.relative_dominance != 0 {
            out_ground.push(*manifold_i)
        } else {
            out_not_ground.push(*manifold_i)
        }
    }
}

pub(crate) fn categorize_joints(
    bodies: &RigidBodySet,
    multibody_joints: &MultibodyJointSet,
    impulse_joints: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    ground_joints: &mut Vec<JointIndex>,
    nonground_joints: &mut Vec<JointIndex>,
    generic_ground_joints: &mut Vec<JointIndex>,
    generic_nonground_joints: &mut Vec<JointIndex>,
) {
    for joint_i in joint_indices {
        let joint = &impulse_joints[*joint_i].weight;
        let rb1 = &bodies[joint.body1.0];
        let rb2 = &bodies[joint.body2.0];

        if multibody_joints.rigid_body_link(joint.body1).is_some()
            || multibody_joints.rigid_body_link(joint.body2).is_some()
        {
            if !rb1.is_dynamic() || !rb2.is_dynamic() {
                generic_ground_joints.push(*joint_i);
            } else {
                generic_nonground_joints.push(*joint_i);
            }
        } else if !rb1.is_dynamic() || !rb2.is_dynamic() {
            ground_joints.push(*joint_i);
        } else {
            nonground_joints.push(*joint_i);
        }
    }
}
