use crate::alloc_prelude::*;
use crate::dynamics::{JointGraphEdge, JointIndex, MultibodyJointSet};

pub(crate) fn categorize_joints(
    multibody_joints: &MultibodyJointSet,
    impulse_joints: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    two_body_joints: &mut Vec<JointIndex>,
    generic_two_body_joints: &mut Vec<JointIndex>,
) {
    for joint_i in joint_indices {
        let joint = &impulse_joints[*joint_i].weight;

        if multibody_joints.rigid_body_link(joint.body1).is_some()
            || multibody_joints.rigid_body_link(joint.body2).is_some()
        {
            generic_two_body_joints.push(*joint_i);
        } else {
            two_body_joints.push(*joint_i);
        }
    }
}
