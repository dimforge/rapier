use crate::dynamics::{JointGraphEdge, JointIndex, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex, KinematicsCategory};

pub(crate) fn categorize_position_contacts(
    bodies: &RigidBodySet,
    manifolds: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    out_point_point_ground: &mut Vec<ContactManifoldIndex>,
    out_plane_point_ground: &mut Vec<ContactManifoldIndex>,
    out_point_point: &mut Vec<ContactManifoldIndex>,
    out_plane_point: &mut Vec<ContactManifoldIndex>,
) {
    for manifold_i in manifold_indices {
        let manifold = &manifolds[*manifold_i];
        let rb1 = &bodies[manifold.body_pair.body1];
        let rb2 = &bodies[manifold.body_pair.body2];

        if !rb1.is_dynamic() || !rb2.is_dynamic() {
            match manifold.kinematics.category {
                KinematicsCategory::PointPoint => out_point_point_ground.push(*manifold_i),
                KinematicsCategory::PlanePoint => out_plane_point_ground.push(*manifold_i),
            }
        } else {
            match manifold.kinematics.category {
                KinematicsCategory::PointPoint => out_point_point.push(*manifold_i),
                KinematicsCategory::PlanePoint => out_plane_point.push(*manifold_i),
            }
        }
    }
}

pub(crate) fn categorize_velocity_contacts(
    bodies: &RigidBodySet,
    manifolds: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    out_ground: &mut Vec<ContactManifoldIndex>,
    out_not_ground: &mut Vec<ContactManifoldIndex>,
) {
    for manifold_i in manifold_indices {
        let manifold = &manifolds[*manifold_i];
        let rb1 = &bodies[manifold.body_pair.body1];
        let rb2 = &bodies[manifold.body_pair.body2];

        if !rb1.is_dynamic() || !rb2.is_dynamic() {
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
