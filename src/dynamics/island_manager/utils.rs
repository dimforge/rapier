use crate::alloc_prelude::*;
use crate::dynamics::{ImpulseJointSet, MultibodyJointSet, RigidBodyColliders, RigidBodyHandle};
use crate::geometry::{ColliderSet, NarrowPhase};

// Read all the contacts and push objects touching this rigid-body.
#[inline]
pub(super) fn push_contacting_bodies(
    rb_colliders: &RigidBodyColliders,
    colliders: &ColliderSet,
    narrow_phase: &NarrowPhase,
    stack: &mut Vec<RigidBodyHandle>,
) {
    for collider_handle in &rb_colliders.0 {
        for inter in narrow_phase.contact_pairs_with(*collider_handle) {
            if inter.has_any_active_contact() {
                let other = crate::utils::select_other(
                    (inter.collider1, inter.collider2),
                    *collider_handle,
                );
                if let Some(other_body) = colliders[other].parent {
                    stack.push(other_body.handle);
                }
            }
        }
    }
}

pub(super) fn push_linked_bodies(
    impulse_joints: &ImpulseJointSet,
    multibody_joints: &MultibodyJointSet,
    handle: RigidBodyHandle,
    stack: &mut Vec<RigidBodyHandle>,
) {
    for inter in impulse_joints.attached_enabled_joints(handle) {
        let other = crate::utils::select_other((inter.0, inter.1), handle);
        stack.push(other);
    }

    for other in multibody_joints.bodies_attached_with_enabled_joint(handle) {
        if let Some(link) = multibody_joints.rigid_body_link(other) {
            let Some(mb) = multibody_joints.get_multibody(link.multibody) else {
                continue;
            };
            let Some(lnk) = mb.link(link.id) else { continue };

            if !mb.root_is_dynamic && lnk.is_root() {
                // If we are attached to the root, and the root is fixed, then
                // we need to push the root’s children too so that the entire multibody
                // ends up on the same island, even if the root has multiple branches
                // attached to it.
                for root_adj in multibody_joints.bodies_attached_with_enabled_joint(lnk.rigid_body) {
                    if root_adj != handle {
                        stack.push(root_adj);
                    }
                }
            }
        }
        stack.push(other);
    }
}
