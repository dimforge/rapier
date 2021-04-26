use crate::data::ComponentSet;
use crate::dynamics::{JointGraphEdge, JointIndex, RigidBodyType};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

pub(crate) fn categorize_contacts(
    _bodies: &impl ComponentSet<RigidBodyType>, // Unused but useful to simplify the parallel code.
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
    bodies: &impl ComponentSet<RigidBodyType>,
    joints: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    ground_joints: &mut Vec<JointIndex>,
    nonground_joints: &mut Vec<JointIndex>,
) {
    for joint_i in joint_indices {
        let joint = &joints[*joint_i].weight;
        let status1 = bodies.index(joint.body1.0);
        let status2 = bodies.index(joint.body2.0);

        if !status1.is_dynamic() || !status2.is_dynamic() {
            ground_joints.push(*joint_i);
        } else {
            nonground_joints.push(*joint_i);
        }
    }
}
