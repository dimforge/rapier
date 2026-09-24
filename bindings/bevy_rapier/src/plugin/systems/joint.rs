use crate::dynamics::free_joint_dofs;
use crate::dynamics::ImpulseJoint;
use crate::dynamics::ImpulseJointDisabled;
use crate::dynamics::ImpulseJointImpulses;
use crate::dynamics::KinematicMultibodyJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::MultibodyJointArmature;
use crate::dynamics::MultibodyJointCoupling;
use crate::dynamics::MultibodyJointCouplings;
use crate::dynamics::MultibodyJointDamping;
use crate::dynamics::MultibodyJointFriction;
use crate::dynamics::MultibodyJointSprings;
use crate::dynamics::MultibodyJointState;
use crate::dynamics::MultibodySelfContactsDisabled;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::plugin::context::systemparams::RAPIER_CONTEXT_EXPECT_ERROR;
use crate::plugin::context::RapierContextEntityLink;
use crate::plugin::context::RapierContextJoints;
use crate::plugin::context::RapierRigidBodySet;
use bevy::platform::collections::HashSet;
use bevy::prelude::*;

use super::RapierContextLinkResolver;
use rapier::dynamics::{
    GenericJoint as RapierGenericJoint, ImpulseJointHandle, ImpulseJointSet, JointEnabled,
    Multibody, MultibodyDofCoupling, MultibodyJointHandle, MultibodyJointSet, RigidBodyHandle,
    RigidBodySet,
};
use rapier::math::SPATIAL_DIM;

/// Finds the rigid-body an [`ImpulseJoint`] of `entity` is attached to: the one of `entity`
/// itself, or of its closest ancestor with a rigid-body.
fn impulse_joint_target_body(
    rigidbody_set: &RapierRigidBodySet,
    child_of_query: &Query<&ChildOf>,
    entity: Entity,
) -> Option<RigidBodyHandle> {
    let mut body_entity = entity;
    loop {
        if let Some(handle) = rigidbody_set.entity2body.get(&body_entity) {
            return Some(*handle);
        }
        body_entity = child_of_query.get(body_entity).ok()?.parent();
    }
}

/// Converts the joint description of an [`ImpulseJoint`] component into the Rapier joint data.
fn impulse_joint_data(entity: Entity, joint: &ImpulseJoint, disabled: bool) -> RapierGenericJoint {
    let mut data = joint.data.as_ref().into_rapier();
    data.user_data = entity.to_bits() as u128;
    if disabled {
        data.set_enabled(false);
    }
    data
}

/// Converts the joint description of a [`MultibodyJoint`] component into the Rapier joint data.
fn multibody_joint_data(entity: Entity, joint: &MultibodyJoint) -> RapierGenericJoint {
    let mut data = joint.data.as_ref().into_rapier();
    data.user_data = entity.to_bits() as u128;
    data
}

/// Inserts a (possibly kinematic) multibody joint between two rigid-bodies.
fn insert_multibody_joint(
    joints: &mut MultibodyJointSet,
    body1: RigidBodyHandle,
    body2: RigidBodyHandle,
    data: RapierGenericJoint,
    kinematic: bool,
) -> Option<MultibodyJointHandle> {
    if kinematic {
        joints.insert_kinematic(body1, body2, data, true)
    } else {
        joints.insert(body1, body2, data, true)
    }
}

/// Writes the per-joint properties of a multibody joint, using the default values for the
/// missing components.
fn write_multibody_joint_properties(
    mb: &mut Multibody,
    link_id: usize,
    kinematic: bool,
    damping: Option<&MultibodyJointDamping>,
    friction: Option<&MultibodyJointFriction>,
    armature: Option<&MultibodyJointArmature>,
    springs: Option<&MultibodyJointSprings>,
) {
    let Some(link) = mb.link(link_id) else {
        return;
    };
    let offset = link.assembly_id();
    let locked_axes = link.joint().data.locked_axes;
    let damping = damping.copied().unwrap_or_default();
    let friction = friction.copied().unwrap_or_default();
    let armature = armature.copied().unwrap_or_default();
    let springs = springs.copied().unwrap_or_default();

    for (dof, axis) in free_joint_dofs(locked_axes) {
        mb.damping_mut()[offset + dof] = damping.0[axis];
        mb.frictions_mut()[offset + dof] = friction.0[axis];
        mb.armature_mut()[offset + dof] = armature.0[axis];
    }

    let Some(link) = mb.link_mut(link_id) else {
        return;
    };
    link.joint.kinematic = kinematic;
    for axis in 0..SPATIAL_DIM {
        if locked_axes.bits() & (1 << axis) == 0 {
            link.joint
                .set_spring(axis, springs.stiffness[axis], springs.rest[axis]);
        } else {
            link.joint.set_spring(axis, 0.0, 0.0);
        }
    }
}

/// Converts a [`MultibodyJointCoupling`] declared on `entity` into a Rapier coupling.
///
/// Returns `None` if the two joints are not part of the same multibody, or if one of the coupled
/// axes is locked.
fn rapier_dof_coupling(
    joints: &MultibodyJointSet,
    rigidbody_set: &RapierRigidBodySet,
    entity: Entity,
    coupling: &MultibodyJointCoupling,
) -> Option<MultibodyDofCoupling> {
    let link_id = |entity: &Entity| {
        let body = rigidbody_set.entity2body.get(entity)?;
        joints.rigid_body_link(*body)
    };
    let link1 = link_id(&coupling.source)?;
    let link2 = link_id(&entity)?;
    if link1.multibody != link2.multibody {
        return None;
    }

    let mb = joints.get_multibody(link1.multibody)?;
    let dof = |link: usize, axis: usize| {
        free_joint_dofs(mb.link(link)?.joint().data.locked_axes)
            .find(|(_, free_axis)| *free_axis == axis)
            .map(|(dof, _)| dof)
    };
    let (axis1, axis2) = (coupling.source_axis as usize, coupling.axis as usize);
    Some(MultibodyDofCoupling {
        link1: link1.id,
        dof1: dof(link1.id, axis1)?,
        axis1,
        link2: link2.id,
        dof2: dof(link2.id, axis2)?,
        axis2,
        coeff: coupling.coeff,
        offset: coupling.offset,
    })
}

/// Makes the Rapier DoF couplings of the multibody joint of `entity` match its declared
/// `couplings`: the stale ones are removed, and the missing ones that can be resolved are added.
fn sync_dof_couplings(
    joints: &mut MultibodyJointSet,
    rigidbody_set: &RapierRigidBodySet,
    entity: Entity,
    handle: MultibodyJointHandle,
    couplings: Option<&MultibodyJointCouplings>,
) {
    let declared: Vec<_> = couplings
        .iter()
        .flat_map(|couplings| couplings.0.iter())
        .filter_map(|c| rapier_dof_coupling(joints, rigidbody_set, entity, c))
        .collect();
    let Some((mb, link_id)) = joints.get_mut(handle) else {
        return;
    };
    // The couplings driving this joint are the ones declared on its entity.
    mb.retain_dof_couplings(|c| {
        c.link2 != link_id || declared.iter().any(|d| same_dof_coupling(c, d))
    });
    for coupling in declared {
        if !mb
            .couplings()
            .iter()
            .any(|c| same_dof_coupling(c, &coupling))
        {
            mb.add_dof_coupling(coupling);
        }
    }
}

/// Wakes up all the rigid-bodies of a multibody.
pub(crate) fn wake_up_multibody(mb: &Multibody, bodies: &mut RigidBodySet) {
    for link in mb.links() {
        if let Some(body) = bodies.get_mut(link.rigid_body_handle()) {
            body.wake_up(true);
        }
    }
}

/// Checks if two Rapier DoF couplings are identical.
fn same_dof_coupling(a: &MultibodyDofCoupling, b: &MultibodyDofCoupling) -> bool {
    (a.link1, a.dof1, a.axis1, a.link2, a.dof2, a.axis2)
        == (b.link1, b.dof1, b.axis1, b.link2, b.dof2, b.axis2)
        && a.coeff == b.coeff
        && a.offset == b.offset
}

/// Enables or disables an impulse joint.
///
/// A joint re-enabled while attached to a disabled rigid-body stays disabled until that body is
/// enabled again. Rapier updates the islands of joints accessed through `get_mut` at the next step.
pub(crate) fn set_impulse_joint_enabled(
    joints: &mut ImpulseJointSet,
    bodies: &RigidBodySet,
    handle: ImpulseJointHandle,
    enabled: bool,
) {
    let Some(joint) = joints.get_mut(handle, true) else {
        return;
    };
    let body_enabled = |h| bodies.get(h).is_none_or(|rb| rb.is_enabled());
    joint.data.enabled = if !enabled {
        JointEnabled::Disabled
    } else if joint.data.enabled != JointEnabled::Disabled {
        joint.data.enabled
    } else if body_enabled(joint.body1()) && body_enabled(joint.body2()) {
        JointEnabled::Enabled
    } else {
        JointEnabled::DisabledByAttachedBody
    };
}

/// System responsible for creating new Rapier joints from the related `bevy_rapier` components.
pub fn init_joints(
    mut commands: Commands,
    mut context_access: Query<(&RapierRigidBodySet, &mut RapierContextJoints)>,
    context_links: RapierContextLinkResolver,
    impulse_joints: Query<
        (
            Entity,
            Option<&RapierContextEntityLink>,
            &ImpulseJoint,
            Has<ImpulseJointDisabled>,
        ),
        Without<RapierImpulseJointHandle>,
    >,
    multibody_joints: Query<
        (
            Entity,
            Option<&RapierContextEntityLink>,
            &MultibodyJoint,
            Has<KinematicMultibodyJoint>,
        ),
        Without<RapierMultibodyJointHandle>,
    >,
    child_of_query: Query<&ChildOf>,
) {
    for (entity, entity_context_link, joint, disabled) in impulse_joints.iter() {
        // Use the RapierContextEntityLink, or insert the context of an ancestor or the default one.
        let context_entity = context_links.resolve(entity, entity_context_link, &mut commands);
        let Some(context_entity) = context_entity else {
            continue;
        };

        let Ok(rigidbody_set_joints) = context_access.get_mut(context_entity) else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let rigidbody_set = rigidbody_set_joints.0;
        let target = impulse_joint_target_body(rigidbody_set, &child_of_query, entity);
        let joints = rigidbody_set_joints.1.into_inner();

        if let (Some(target), Some(source)) = (target, rigidbody_set.entity2body.get(&joint.parent))
        {
            let handle = joints.impulse_joints.insert(
                *source,
                target,
                impulse_joint_data(entity, joint, disabled),
                true,
            );
            commands
                .entity(entity)
                .insert(RapierImpulseJointHandle(handle));
            joints.entity2impulse_joint.insert(entity, handle);
        }
    }

    for (entity, entity_context_link, joint, kinematic) in multibody_joints.iter() {
        // Use the RapierContextEntityLink, or insert the context of an ancestor or the default one.
        let context_entity = context_links.resolve(entity, entity_context_link, &mut commands);
        let Some(context_entity) = context_entity else {
            continue;
        };

        let Ok(context_joints) = context_access.get_mut(context_entity) else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let context = context_joints.0;
        let target = context.entity2body.get(&entity);
        let joints = context_joints.1.into_inner();

        if let (Some(target), Some(source)) = (target, context.entity2body.get(&joint.parent)) {
            if let Some(handle) = insert_multibody_joint(
                &mut joints.multibody_joints,
                *source,
                *target,
                multibody_joint_data(entity, joint),
                kinematic,
            ) {
                commands
                    .entity(entity)
                    .insert(RapierMultibodyJointHandle(handle));
                joints.entity2multibody_joint.insert(entity, handle);
            } else {
                log::error!("Failed to create multibody joint: loop detected.")
            }
        }
    }
}

/// System responsible for applying changes the user made to a joint component.
///
/// This also re-attaches an [`ImpulseJoint`] to other rigid-bodies if its `parent`, or the
/// rigid-body of its entity, changed, and a [`MultibodyJoint`] if its `parent` changed. If one of
/// the new bodies doesn’t exist yet, the joint is removed from the physics scene until
/// [`init_joints`] can create it again.
#[allow(clippy::type_complexity)]
pub fn apply_joint_user_changes(
    mut commands: Commands,
    mut context: Query<(&RapierRigidBodySet, &mut RapierContextJoints)>,
    changed_impulse_joints: Query<
        (
            Entity,
            &RapierContextEntityLink,
            &RapierImpulseJointHandle,
            &ImpulseJoint,
            Has<ImpulseJointDisabled>,
        ),
        Or<(
            Changed<ImpulseJoint>,
            Changed<ImpulseJointDisabled>,
            Changed<ChildOf>,
        )>,
    >,
    changed_multibody_joints: Query<
        (
            Entity,
            &RapierContextEntityLink,
            &RapierMultibodyJointHandle,
            &MultibodyJoint,
            Has<KinematicMultibodyJoint>,
        ),
        Changed<MultibodyJoint>,
    >,
    child_of_query: Query<&ChildOf>,
) {
    for (entity, link, handle, changed_joint, disabled) in changed_impulse_joints.iter() {
        let (rigidbody_set, mut context) =
            context.get_mut(link.0).expect(RAPIER_CONTEXT_EXPECT_ERROR);
        let joints = &mut context.impulse_joints;

        let body1 = rigidbody_set
            .entity2body
            .get(&changed_joint.parent)
            .copied();
        let body2 = impulse_joint_target_body(rigidbody_set, &child_of_query, entity);
        let (Some(body1), Some(body2), true) = (body1, body2, joints.contains(handle.0)) else {
            // Detach the joint until its bodies exist; `init_joints` will then re-create it.
            joints.remove(handle.0, true);
            context.entity2impulse_joint.remove(&entity);
            commands.entity(entity).remove::<RapierImpulseJointHandle>();
            continue;
        };

        let Some(joint) = joints.set_bodies(handle.0, body1, body2, true) else {
            continue;
        };

        // Keep the solver’s warmstart impulses and let `set_impulse_joint_enabled` handle the
        // enabled state, since Rapier may have disabled the joint because of its bodies.
        let old_data = joint.data;
        let mut new_data = impulse_joint_data(entity, changed_joint, false);
        new_data.enabled = old_data.enabled;
        for (new_limit, old_limit) in new_data.limits.iter_mut().zip(old_data.limits.iter()) {
            new_limit.impulse = old_limit.impulse;
        }
        for (new_motor, old_motor) in new_data.motors.iter_mut().zip(old_data.motors.iter()) {
            new_motor.impulse = old_motor.impulse;
        }
        joint.data = new_data;

        let enabled = changed_joint.data.as_ref().is_enabled() && !disabled;
        set_impulse_joint_enabled(joints, &rigidbody_set.bodies, handle.0, enabled);
    }

    for (entity, link, handle, changed_joint, kinematic) in changed_multibody_joints.iter() {
        let (rigidbody_set, mut context) =
            context.get_mut(link.0).expect(RAPIER_CONTEXT_EXPECT_ERROR);
        let context = &mut *context;
        let data = multibody_joint_data(entity, changed_joint);
        let new_parent = rigidbody_set
            .entity2body
            .get(&changed_joint.parent)
            .copied();

        let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) else {
            continue;
        };
        let Some(mb_link) = mb.link(link_id) else {
            continue;
        };
        let body = mb_link.rigid_body_handle();
        let old_parent = mb_link
            .parent_id()
            .and_then(|id| mb.link(id))
            .map(|parent| parent.rigid_body_handle());

        if old_parent.is_some() && old_parent == new_parent {
            // TODO: not sure this will always work properly, e.g., if the number of Dofs is changed.
            if let Some(mb_link) = mb.link_mut(link_id) {
                mb_link.joint.data = data;
            }
            continue;
        }

        // The parent changed: re-attach the joint to the new parent.
        context.multibody_joints.remove(handle.0, true);
        let new_handle = new_parent.and_then(|parent| {
            insert_multibody_joint(&mut context.multibody_joints, parent, body, data, kinematic)
        });
        if let Some(new_handle) = new_handle {
            context.entity2multibody_joint.insert(entity, new_handle);
            // Insert the handle even if unchanged: this flags the multibody topology change.
            commands
                .entity(entity)
                .insert(RapierMultibodyJointHandle(new_handle));
        } else {
            if new_parent.is_some() {
                log::error!("Failed to re-attach multibody joint: loop detected.");
            }
            // Detach the joint; `init_joints` will then try to re-create it.
            context.entity2multibody_joint.remove(&entity);
            commands
                .entity(entity)
                .remove::<RapierMultibodyJointHandle>();
        }
    }
}

/// Components configuring a multibody joint, read by [`apply_multibody_joint_properties`].
type MultibodyJointPropertiesComponents = (
    &'static RapierContextEntityLink,
    &'static RapierMultibodyJointHandle,
    Has<KinematicMultibodyJoint>,
    Option<&'static MultibodyJointDamping>,
    Option<&'static MultibodyJointFriction>,
    Option<&'static MultibodyJointArmature>,
    Option<&'static MultibodyJointSprings>,
    Option<&'static MultibodyJointCouplings>,
);

/// Filter selecting the multibody joints with modified properties.
type ChangedMultibodyJointProperties = Or<(
    Added<RapierMultibodyJointHandle>,
    Changed<MultibodyJoint>,
    Changed<KinematicMultibodyJoint>,
    Changed<MultibodyJointDamping>,
    Changed<MultibodyJointFriction>,
    Changed<MultibodyJointArmature>,
    Changed<MultibodyJointSprings>,
    Changed<MultibodyJointCouplings>,
)>;

/// System responsible for applying the properties of multibody joints and multibodies set by the
/// user through the [`KinematicMultibodyJoint`], [`MultibodyJointDamping`],
/// [`MultibodyJointFriction`], [`MultibodyJointArmature`], [`MultibodyJointSprings`],
/// [`MultibodyJointCouplings`] and [`MultibodySelfContactsDisabled`] components.
///
/// The properties are applied when the joint is created, and when these components are modified
/// or removed. DoF couplings and disabled self-contacts are also applied again when a multibody
/// joint is created or re-attached, since they might only be resolvable once joints are merged
/// into the same multibody.
#[allow(clippy::type_complexity)]
pub fn apply_multibody_joint_properties(
    mut context: Query<(&mut RapierRigidBodySet, &mut RapierContextJoints)>,
    changed_joints: Query<Entity, (With<MultibodyJoint>, ChangedMultibodyJointProperties)>,
    changed_couplings: Query<Entity, Changed<MultibodyJointCouplings>>,
    changed_topology: Query<
        &RapierContextEntityLink,
        (With<MultibodyJoint>, Changed<RapierMultibodyJointHandle>),
    >,
    joints: Query<MultibodyJointPropertiesComponents, With<MultibodyJoint>>,
    all_couplings: Query<(
        Entity,
        &RapierContextEntityLink,
        &RapierMultibodyJointHandle,
        &MultibodyJointCouplings,
    )>,
    self_contacts_disabled: Query<(
        Entity,
        &RapierContextEntityLink,
        Ref<MultibodySelfContactsDisabled>,
    )>,
    (
        mut removed_kinematic,
        mut removed_damping,
        mut removed_friction,
        mut removed_armature,
        mut removed_springs,
        mut removed_couplings,
    ): (
        RemovedComponents<KinematicMultibodyJoint>,
        RemovedComponents<MultibodyJointDamping>,
        RemovedComponents<MultibodyJointFriction>,
        RemovedComponents<MultibodyJointArmature>,
        RemovedComponents<MultibodyJointSprings>,
        RemovedComponents<MultibodyJointCouplings>,
    ),
    mut removed_self_contacts_disabled: RemovedComponents<MultibodySelfContactsDisabled>,
) {
    let mut couplings_modified: HashSet<Entity> = changed_couplings.iter().collect();
    couplings_modified.extend(removed_couplings.read());
    let mut modified: HashSet<Entity> = changed_joints.iter().collect();
    modified.extend(couplings_modified.iter().copied());
    modified.extend(removed_kinematic.read());
    modified.extend(removed_damping.read());
    modified.extend(removed_friction.read());
    modified.extend(removed_armature.read());
    modified.extend(removed_springs.read());
    // The contexts where multibody joints were created or re-attached.
    let topology_changed: HashSet<Entity> = changed_topology.iter().map(|link| link.0).collect();

    for entity in modified {
        let Ok((link, handle, kinematic, damping, friction, armature, springs, couplings)) =
            joints.get(entity)
        else {
            continue;
        };
        let Ok((mut rigidbody_set, mut context)) = context.get_mut(link.0) else {
            continue;
        };
        let (rigidbody_set, context) = (&mut *rigidbody_set, &mut *context);

        if couplings_modified.contains(&entity) {
            sync_dof_couplings(
                &mut context.multibody_joints,
                rigidbody_set,
                entity,
                handle.0,
                couplings,
            );
        }

        if let Some((mb, link_id)) = context.multibody_joints.get_mut(handle.0) {
            write_multibody_joint_properties(
                mb, link_id, kinematic, damping, friction, armature, springs,
            );
            wake_up_multibody(mb, &mut rigidbody_set.bodies);
        }
    }

    // Rapier keeps the couplings across topology changes, but the ones that couldn't be resolved
    // yet (e.g. joints in different multibodies) might be now.
    for (entity, link, handle, couplings) in all_couplings.iter() {
        if !topology_changed.contains(&link.0) {
            continue;
        }
        let Ok((rigidbody_set, mut context)) = context.get_mut(link.0) else {
            continue;
        };
        sync_dof_couplings(
            &mut context.multibody_joints,
            &rigidbody_set,
            entity,
            handle.0,
            Some(couplings),
        );
    }

    // Enable self-contacts again when the marker is removed. If another link still has the
    // marker, the multibody is disabled again right after.
    let mut any_marker_removed = false;
    for entity in removed_self_contacts_disabled.read() {
        if !self_contacts_disabled.contains(entity) {
            set_multibody_self_contacts(&mut context, entity, true);
            any_marker_removed = true;
        }
    }
    // Rapier keeps self-contacts disabled when multibodies merge, but a marker added before its
    // rigid-body joined a multibody has to be applied once it does.
    for (entity, link, marker) in self_contacts_disabled.iter() {
        if any_marker_removed || marker.is_changed() || topology_changed.contains(&link.0) {
            set_multibody_self_contacts(&mut context, entity, false);
        }
    }
}

/// System responsible for writing the impulses applied by impulse joints back into the
/// [`ImpulseJointImpulses`] component.
pub fn writeback_impulse_joint_impulses(
    context: Query<&RapierContextJoints>,
    mut impulse_joints: Query<(
        &RapierContextEntityLink,
        &RapierImpulseJointHandle,
        &mut ImpulseJointImpulses,
    )>,
) {
    for (link, handle, mut impulses) in impulse_joints.iter_mut() {
        let Some(joint) = context
            .get(link.0)
            .ok()
            .and_then(|joints| joints.impulse_joints.get(handle.0))
        else {
            continue;
        };

        // A disabled joint isn't solved, so its last impulses are stale.
        let new_impulses = if joint.data.is_enabled() {
            ImpulseJointImpulses::from_rapier(joint)
        } else {
            ImpulseJointImpulses::default()
        };
        // NOTE: only write actual changes to avoid triggering bevy's change detection.
        impulses.set_if_neq(new_impulses);
    }
}

/// Enables or disables the self-contacts of the multibody containing the rigid-body of `entity`.
fn set_multibody_self_contacts(
    contexts: &mut Query<(&mut RapierRigidBodySet, &mut RapierContextJoints)>,
    entity: Entity,
    enabled: bool,
) {
    for (rigidbody_set, mut context) in contexts.iter_mut() {
        let Some(body) = rigidbody_set.entity2body.get(&entity) else {
            continue;
        };
        let Some(link) = context.multibody_joints.rigid_body_link(*body).copied() else {
            return;
        };
        let needs_change = context
            .multibody_joints
            .get_multibody(link.multibody)
            .is_some_and(|mb| mb.self_contacts_enabled() != enabled);
        if needs_change {
            if let Some(mb) = context.multibody_joints.get_multibody_mut(link.multibody) {
                mb.set_self_contacts_enabled(enabled);
            }
        }
        return;
    }
}

/// System responsible for writing the state of multibody joints back into the
/// [`MultibodyJointState`] component.
pub fn writeback_multibody_joint_states(
    context: Query<&RapierContextJoints>,
    mut multibody_joints: Query<(
        &RapierContextEntityLink,
        &RapierMultibodyJointHandle,
        &mut MultibodyJointState,
    )>,
) {
    for (link, handle, mut state) in multibody_joints.iter_mut() {
        let Some(new_state) = context
            .get(link.0)
            .ok()
            .and_then(|joints| joints.multibody_joint_state_from_handle(handle.0))
        else {
            continue;
        };
        // NOTE: only write actual changes to avoid triggering bevy's change detection.
        state.set_if_neq(new_state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
        app.finish();
        app.update();
        app
    }

    fn run(app: &mut App, frames: usize) {
        for _ in 0..frames {
            app.update();
        }
    }

    fn spawn_body(app: &mut App, body: RigidBody, x: f32, y: f32) -> Entity {
        app.world_mut()
            .spawn((Transform::from_xyz(x, y, 0.0), body, Collider::ball(0.1)))
            .id()
    }

    fn translation(app: &App, entity: Entity) -> Vec3 {
        app.world().get::<Transform>(entity).unwrap().translation
    }

    fn with_joints<R>(
        app: &mut App,
        f: impl FnOnce(&RapierRigidBodySet, &RapierContextJoints) -> R,
    ) -> R {
        let world = app.world_mut();
        let (bodies, joints) = world
            .query::<(&RapierRigidBodySet, &RapierContextJoints)>()
            .single(world)
            .unwrap();
        f(bodies, joints)
    }

    fn raw_joint<R>(
        app: &mut App,
        entity: Entity,
        f: impl FnOnce(&RapierRigidBodySet, &rapier::dynamics::ImpulseJoint) -> R,
    ) -> R {
        with_joints(app, |bodies, joints| {
            let handle = joints.entity2impulse_joint()[&entity];
            f(bodies, joints.impulse_joints.get(handle).unwrap())
        })
    }

    #[cfg(feature = "dim2")]
    #[test]
    fn pin_slot_joint_constrains_motion() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let body = spawn_body(&mut app, RigidBody::Dynamic, 0.0, 0.0);
        app.world_mut().entity_mut(body).insert((
            Velocity {
                linear: Vect::new(1.0, 0.0),
                angular: 2.0,
            },
            ImpulseJoint::new(
                ground,
                PinSlotJointBuilder::new(Vect::X).contacts_enabled(false),
            ),
        ));
        run(&mut app, 60);

        let pos = translation(&app, body);
        assert!(pos.x > 0.5, "the body should slide along the slot: {pos}");
        assert!(pos.y.abs() < 1.0e-2, "the body should not fall: {pos}");
        let angvel = app.world().get::<Velocity>(body).unwrap().angular;
        assert!(angvel.abs() > 1.0, "rotations should be free: {angvel}");

        let joint = app.world().get::<ImpulseJoint>(body).unwrap();
        assert!(matches!(joint.data, TypedJoint::PinSlotJoint(_)));
        assert!(joint.data.as_ref().as_pin_slot().is_some());
        assert!(joint.data.as_ref().as_prismatic().is_none());
    }

    #[test]
    fn disabled_impulse_joint_lets_bodies_separate() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let body = spawn_body(&mut app, RigidBody::Dynamic, 0.0, 0.0);
        let joint = FixedJointBuilder::new().contacts_enabled(false);
        app.world_mut()
            .entity_mut(body)
            .insert((ImpulseJoint::new(ground, joint), ImpulseJointDisabled));
        run(&mut app, 30);

        assert!(translation(&app, body).y < -0.5, "the body should fall");
        raw_joint(&mut app, body, |_, j| assert!(!j.data.is_enabled()));
        with_joints(&mut app, |bodies, joints| {
            assert_eq!(joints.attached_impulse_joints(bodies, ground).count(), 1);
            assert_eq!(
                joints
                    .attached_enabled_impulse_joints(bodies, ground)
                    .count(),
                0
            );
        });
        assert_eq!(
            *app.world().get::<ImpulseJointImpulses>(body).unwrap(),
            ImpulseJointImpulses::default()
        );

        app.world_mut()
            .entity_mut(body)
            .remove::<ImpulseJointDisabled>();
        run(&mut app, 120);

        raw_joint(&mut app, body, |_, j| assert!(j.data.is_enabled()));
        let pos = translation(&app, body);
        assert!(
            pos.length() < 0.1,
            "the joint should pull the body back: {pos}"
        );

        app.world_mut()
            .entity_mut(body)
            .insert(ImpulseJointDisabled);
        run(&mut app, 30);
        raw_joint(&mut app, body, |_, j| assert!(!j.data.is_enabled()));
        assert!(
            translation(&app, body).y < -0.5,
            "the body should fall again"
        );
    }

    #[test]
    fn disabled_impulse_joint_splits_islands() {
        let mut app = test_app();
        let body1 = spawn_body(&mut app, RigidBody::Dynamic, 0.0, 0.0);
        let body2 = spawn_body(&mut app, RigidBody::Dynamic, 2.0, 0.0);
        app.world_mut()
            .entity_mut(body1)
            .insert((GravityScale(0.0), Sleeping::default()));
        app.world_mut().entity_mut(body2).insert((
            GravityScale(0.0),
            Sleeping::default(),
            ImpulseJoint::new(body1, FixedJointBuilder::new().local_anchor1(Vect::X * 2.0)),
        ));
        run(&mut app, 2);

        // With the joint disabled, the resting body falls asleep while the other keeps moving.
        app.world_mut()
            .entity_mut(body2)
            .insert((ImpulseJointDisabled, Velocity::linear(Vect::X)));
        run(&mut app, 240);
        let sleeping = |app: &App, entity| app.world().get::<Sleeping>(entity).unwrap().sleeping;
        assert!(sleeping(&app, body1));
        assert!(!sleeping(&app, body2));

        // Re-enabling the joint merges the islands again, waking the resting body up.
        app.world_mut()
            .entity_mut(body2)
            .remove::<ImpulseJointDisabled>();
        run(&mut app, 2);
        assert!(!sleeping(&app, body1));
    }

    #[test]
    fn impulses_are_written_back() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let body = spawn_body(&mut app, RigidBody::Dynamic, 1.0, 0.0);
        app.world_mut().entity_mut(body).insert((
            AdditionalMassProperties::Mass(10.0),
            ImpulseJoint::new(
                ground,
                FixedJointBuilder::new()
                    .local_anchor1(Vect::X)
                    .contacts_enabled(false),
            ),
        ));
        run(&mut app, 10);

        let impulses = *app.world().get::<ImpulseJointImpulses>(body).unwrap();
        assert!(
            impulses.linear.length() > 1.0e-2,
            "the joint should hold the body against gravity: {impulses:?}"
        );
        let from_context = with_joints(&mut app, |_, joints| {
            joints.impulse_joint_impulses(body).unwrap()
        });
        assert_eq!(impulses, from_context);
    }

    #[test]
    fn reparenting_moves_the_joint() {
        let mut app = test_app();
        let ground1 = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let ground2 = spawn_body(&mut app, RigidBody::Fixed, 5.0, 0.0);
        let body = spawn_body(&mut app, RigidBody::Dynamic, 0.0, -1.0);
        app.world_mut().entity_mut(body).insert(ImpulseJoint::new(
            ground1,
            FixedJointBuilder::new().local_anchor1(-Vect::Y),
        ));
        run(&mut app, 2);

        let handle = app.world().get::<RapierImpulseJointHandle>(body).unwrap().0;
        with_joints(&mut app, |bodies, joints| {
            let between: Vec<_> = joints
                .impulse_joints_between(bodies, ground1, body)
                .collect();
            assert_eq!(between, vec![body]);
            let between: Vec<_> = joints
                .impulse_joints_between(bodies, body, ground1)
                .collect();
            assert_eq!(between, vec![body]);
            assert_eq!(
                joints.impulse_joints_between(bodies, ground2, body).count(),
                0
            );
            assert_eq!(joints.impulse_joint_entity(handle), Some(body));
        });

        app.world_mut()
            .get_mut::<ImpulseJoint>(body)
            .unwrap()
            .parent = ground2;
        run(&mut app, 120);

        assert_eq!(
            app.world().get::<RapierImpulseJointHandle>(body).unwrap().0,
            handle
        );
        with_joints(&mut app, |bodies, joints| {
            assert_eq!(
                joints.impulse_joints_between(bodies, ground1, body).count(),
                0
            );
            let between: Vec<_> = joints
                .impulse_joints_between(bodies, ground2, body)
                .collect();
            assert_eq!(between, vec![body]);
            let raw = joints.impulse_joints.get(handle).unwrap();
            assert_eq!(raw.body1(), bodies.entity2body()[&ground2]);
        });
        let pos = translation(&app, body);
        assert!((pos.x - 5.0).abs() < 0.1, "the body should follow: {pos}");

        // Re-parenting to an entity without rigid-body detaches the joint until it gets one.
        let pending = app
            .world_mut()
            .spawn(Transform::from_xyz(-5.0, 0.0, 0.0))
            .id();
        app.world_mut()
            .get_mut::<ImpulseJoint>(body)
            .unwrap()
            .parent = pending;
        run(&mut app, 2);
        assert!(app.world().get::<RapierImpulseJointHandle>(body).is_none());
        with_joints(&mut app, |_, joints| {
            assert!(joints.impulse_joints.is_empty())
        });

        app.world_mut().entity_mut(pending).insert(RigidBody::Fixed);
        run(&mut app, 2);
        raw_joint(&mut app, body, |bodies, raw| {
            assert_eq!(raw.body1(), bodies.entity2body()[&pending]);
        });
    }

    #[test]
    fn softness_round_trip() {
        let softness = SpringCoefficients::new(10.0, 0.5);
        let mut joint = PrismaticJointBuilder::new(Vect::X)
            .softness(softness)
            .build();
        assert_eq!(joint.softness(), softness);
        let other = SpringCoefficients::new(20.0, 1.0);
        joint.set_softness(other);
        assert_eq!(joint.softness(), other);
        assert_eq!(joint.data.softness(), other);
        assert_eq!(
            GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
                .softness(softness)
                .build()
                .softness(),
            softness
        );

        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let body = spawn_body(&mut app, RigidBody::Dynamic, 0.0, 0.0);
        app.world_mut()
            .entity_mut(body)
            .insert(ImpulseJoint::new(ground, joint));
        run(&mut app, 1);
        raw_joint(&mut app, body, |_, j| assert_eq!(j.data.softness, other));

        app.world_mut()
            .get_mut::<ImpulseJoint>(body)
            .unwrap()
            .data
            .as_mut()
            .set_softness(softness);
        run(&mut app, 1);
        raw_joint(&mut app, body, |_, j| assert_eq!(j.data.softness, softness));
    }

    #[cfg(feature = "dim3")]
    #[test]
    fn spherical_joint_frames() {
        let basis1 = Quat::from_rotation_z(0.5);
        let basis2 = Quat::from_rotation_x(-0.3);
        let joint = SphericalJointBuilder::new()
            .local_anchor1(Vect::X)
            .local_basis1(basis1)
            .local_basis2(basis2)
            .contacts_enabled(false)
            .build();
        assert!(joint.local_basis1().abs_diff_eq(basis1, 1.0e-6));
        assert!(joint.local_basis2().abs_diff_eq(basis2, 1.0e-6));
        assert_eq!(joint.local_frame1().translation, Vect::X);
        assert!(!joint.contacts_enabled());

        let mut other = SphericalJoint::new();
        other.set_local_frame1(joint.local_frame1());
        assert_eq!(other.local_frame1(), joint.local_frame1());
        assert_eq!(other.local_anchor1(), Vect::X);
    }

    #[test]
    fn changing_a_joint_wakes_up_its_bodies() {
        let mut app = test_app();
        let body1 = spawn_body(&mut app, RigidBody::Dynamic, 0.0, 0.0);
        let body2 = spawn_body(&mut app, RigidBody::Dynamic, 2.0, 0.0);
        app.world_mut()
            .entity_mut(body1)
            .insert((GravityScale(0.0), Sleeping::default()));
        app.world_mut().entity_mut(body2).insert((
            GravityScale(0.0),
            Sleeping::default(),
            ImpulseJoint::new(body1, FixedJointBuilder::new().local_anchor1(Vect::X * 2.0)),
        ));
        run(&mut app, 120);
        assert!(app.world().get::<Sleeping>(body1).unwrap().sleeping);
        assert!(app.world().get::<Sleeping>(body2).unwrap().sleeping);

        app.world_mut()
            .get_mut::<ImpulseJoint>(body2)
            .unwrap()
            .data
            .as_mut()
            .set_contacts_enabled(false);
        app.update();

        with_joints(&mut app, |bodies, _| {
            for entity in [body1, body2] {
                let rb = &bodies.bodies[bodies.entity2body()[&entity]];
                assert!(!rb.is_sleeping());
            }
        });
    }

    fn with_joints_mut<R>(
        app: &mut App,
        f: impl FnOnce(&mut RapierRigidBodySet, &mut RapierContextJoints) -> R,
    ) -> R {
        let world = app.world_mut();
        let (mut bodies, mut joints) = world
            .query::<(&mut RapierRigidBodySet, &mut RapierContextJoints)>()
            .single_mut(world)
            .unwrap();
        f(&mut bodies, &mut joints)
    }

    fn vect(x: f32, y: f32) -> Vect {
        #[cfg(feature = "dim2")]
        return Vect::new(x, y);
        #[cfg(feature = "dim3")]
        return Vect::new(x, y, 0.0);
    }

    /// The rotational axis of the joints returned by [`revolute`], in the joint frame.
    const ROT_AXIS: JointAxis = JointAxis::AngX;

    /// A revolute joint rotating in the XY plane, with its second anchor one unit behind the
    /// second body along X, and its first anchor at `anchor1`.
    fn revolute(anchor1: Vect) -> TypedJoint {
        #[cfg(feature = "dim2")]
        let builder = RevoluteJointBuilder::new();
        #[cfg(feature = "dim3")]
        let builder = RevoluteJointBuilder::new(Vect::Z);
        builder
            .local_anchor1(anchor1)
            .local_anchor2(-Vect::X)
            .build()
            .into()
    }

    /// A prismatic joint sliding along X, with its first anchor at `anchor1`.
    fn prismatic(anchor1: Vect) -> TypedJoint {
        PrismaticJointBuilder::new(Vect::X)
            .local_anchor1(anchor1)
            .build()
            .into()
    }

    /// Spawns a dynamic body with unit mass attached to `parent` by a multibody joint.
    fn spawn_link(app: &mut App, parent: Entity, pos: Vect, joint: TypedJoint) -> Entity {
        let entity = spawn_body(app, RigidBody::Dynamic, pos.x, pos.y);
        app.world_mut().entity_mut(entity).insert((
            ColliderMassProperties::Mass(1.0),
            MultibodyJoint::new(parent, joint),
        ));
        entity
    }

    fn joint_state(app: &App, entity: Entity) -> MultibodyJointState {
        *app.world().get::<MultibodyJointState>(entity).unwrap()
    }

    #[test]
    fn kinematic_multibody_joint_ignores_gravity() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let arm = spawn_link(&mut app, ground, Vect::X, revolute(Vect::ZERO));
        app.world_mut()
            .entity_mut(arm)
            .insert(KinematicMultibodyJoint);
        run(&mut app, 60);

        let pos = translation(&app, arm);
        assert!(
            (pos.x - 1.0).abs() < 1.0e-3 && pos.y.abs() < 1.0e-3,
            "the kinematic arm should not fall: {pos}"
        );
        with_joints(&mut app, |bodies, joints| {
            let (mb, link_id) = joints.multibody(bodies, arm).unwrap();
            assert!(mb.link(link_id).unwrap().joint().kinematic);
        });

        // A kinematic joint follows the velocities set by the user.
        with_joints_mut(&mut app, |bodies, joints| {
            joints.multibody_joint_velocity_mut(bodies, arm).unwrap()[0] = 1.0;
        });
        run(&mut app, 30);
        let angle = joint_state(&app, arm).coords[ROT_AXIS as usize];
        assert!((angle - 0.5).abs() < 0.05, "the arm should rotate: {angle}");

        app.world_mut()
            .entity_mut(arm)
            .remove::<KinematicMultibodyJoint>();
        with_joints_mut(&mut app, |bodies, joints| {
            joints.multibody_joint_velocity_mut(bodies, arm).unwrap()[0] = 0.0;
        });
        run(&mut app, 30);
        with_joints(&mut app, |bodies, joints| {
            let (mb, link_id) = joints.multibody(bodies, arm).unwrap();
            assert!(!mb.link(link_id).unwrap().joint().kinematic);
        });
        let angle = joint_state(&app, arm).coords[ROT_AXIS as usize];
        assert!(angle < 0.3, "the arm should fall once dynamic: {angle}");
    }

    #[test]
    fn multibody_joint_friction_and_damping_slow_down_the_joint() {
        let mut app = test_app();
        let mut links = vec![];
        for (i, y) in [0.0, 5.0, 10.0].into_iter().enumerate() {
            let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, y);
            let link = spawn_link(&mut app, ground, vect(0.0, y), prismatic(Vect::ZERO));
            match i {
                1 => {
                    let friction = MultibodyJointFriction::default().with(JointAxis::LinX, 1.0);
                    app.world_mut().entity_mut(link).insert(friction);
                }
                2 => {
                    let damping = MultibodyJointDamping::default().with(JointAxis::LinX, 1.0);
                    app.world_mut().entity_mut(link).insert(damping);
                }
                _ => {}
            }
            links.push(link);
        }
        run(&mut app, 1);
        with_joints_mut(&mut app, |bodies, joints| {
            for link in &links {
                joints.multibody_joint_velocity_mut(bodies, *link).unwrap()[0] = 2.0;
            }
        });
        run(&mut app, 30);

        let [free, friction, damping] =
            std::array::from_fn(|i| joint_state(&app, links[i]).velocities[0]);
        assert!(free > 1.95, "the free joint should keep moving: {free}");
        assert!(
            friction < 1.8 && friction > 0.0,
            "friction should slow down the joint: {friction}"
        );
        assert!(
            damping < 1.8 && damping > 0.0,
            "damping should slow down the joint: {damping}"
        );

        // Removing the friction lets the joint move freely again.
        app.world_mut()
            .entity_mut(links[1])
            .remove::<MultibodyJointFriction>();
        run(&mut app, 1);
        with_joints(&mut app, |bodies, joints| {
            let (mb, _) = joints.multibody(bodies, links[1]).unwrap();
            let dofs = joints.multibody_link_dofs(bodies, links[1]).unwrap();
            assert_eq!(dofs.len(), 1);
            assert_eq!(mb.frictions()[dofs.start], 0.0);
        });
    }

    #[test]
    fn multibody_joint_spring_pulls_towards_rest() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let link = spawn_link(&mut app, ground, Vect::ZERO, prismatic(Vect::ZERO));
        app.world_mut().entity_mut(link).insert((
            MultibodyJointSprings::default().with(JointAxis::LinX, 50.0, 1.0),
            MultibodyJointDamping::splat(5.0),
        ));
        run(&mut app, 300);

        let coord = joint_state(&app, link).coords[0];
        assert!(
            (coord - 1.0).abs() < 0.05,
            "the spring should reach its rest position: {coord}"
        );
        assert!((translation(&app, link).x - 1.0).abs() < 0.05);
        with_joints(&mut app, |bodies, joints| {
            let (mb, link_id) = joints.multibody(bodies, link).unwrap();
            let joint = mb.link(link_id).unwrap().joint();
            assert_eq!(joint.spring(0), (50.0, 1.0));
        });

        app.world_mut()
            .entity_mut(link)
            .remove::<MultibodyJointSprings>();
        run(&mut app, 1);
        with_joints(&mut app, |bodies, joints| {
            let (mb, link_id) = joints.multibody(bodies, link).unwrap();
            assert_eq!(mb.link(link_id).unwrap().joint().spring(0), (0.0, 0.0));
        });
    }

    #[test]
    fn multibody_inverse_kinematics_reaches_target() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let link1 = spawn_link(&mut app, ground, Vect::X, revolute(Vect::ZERO));
        let link2 = spawn_link(&mut app, link1, Vect::X * 2.0, revolute(Vect::ZERO));
        run(&mut app, 1);

        let target = Transform::from_xyz(1.0, 1.0, 0.0);
        let options = InverseKinematicsOption {
            constrained_axes: JointAxesMask::LIN_X | JointAxesMask::LIN_Y,
            max_iters: 50,
            ..Default::default()
        };
        let distance = |bodies: &RapierRigidBodySet| {
            let body = &bodies.bodies[bodies.entity2body()[&link2]];
            let pos = body.translation();
            (vect(pos.x, pos.y) - vect(1.0, 1.0)).length()
        };

        with_joints_mut(&mut app, |bodies, joints| {
            let before = distance(bodies);
            let mut displacements = vec![];
            assert!(joints.multibody_inverse_kinematics(
                bodies,
                link2,
                target,
                &options,
                |_| true,
                &mut displacements,
            ));
            assert_eq!(displacements.len(), 2);
            // A reused buffer is reset before solving: its previous content doesn't matter.
            let mut reused = vec![10.0, -3.0, 7.0];
            assert!(joints.multibody_inverse_kinematics(
                bodies,
                link2,
                target,
                &options,
                |_| true,
                &mut reused,
            ));
            assert_eq!(reused, displacements);
            assert!(!joints.multibody_apply_displacements(bodies, link2, &[0.0]));
            assert!(joints.multibody_apply_displacements(bodies, link2, &displacements));
            assert!(joints.multibody_forward_kinematics(bodies, link2, false));
            let after = distance(bodies);
            assert!(
                after < before && after < 0.05,
                "IK should move the end effector to the target: {before} -> {after}"
            );

            // Locking every joint prevents any motion.
            joints.multibody_inverse_kinematics(
                bodies,
                link2,
                Transform::from_xyz(-1.0, 1.0, 0.0),
                &options,
                |_| false,
                &mut displacements,
            );
            assert!(displacements.iter().all(|d| *d == 0.0));
            assert!(joints.multibody_body_jacobian(bodies, link2).is_some());
        });
    }

    #[test]
    fn multibody_joint_state_is_written_back() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let arm = spawn_link(&mut app, ground, Vect::X, revolute(Vect::ZERO));
        run(&mut app, 20);

        let state = joint_state(&app, arm);
        let angle = state.coords[ROT_AXIS as usize];
        assert!(angle < -0.1, "the arm should swing down: {state:?}");
        assert!(state.velocities[ROT_AXIS as usize] < 0.0);
        assert_eq!(state.coords[0], 0.0);

        let rotation = app.world().get::<Transform>(arm).unwrap().rotation;
        let (_, _, body_angle) = rotation.to_euler(EulerRot::XYZ);
        assert!(
            (body_angle - angle).abs() < 1.0e-3,
            "{body_angle} vs {angle}"
        );
        #[cfg(feature = "dim2")]
        assert!((state.rotation - angle).abs() < 1.0e-3);
        #[cfg(feature = "dim3")]
        assert!(state
            .rotation
            .abs_diff_eq(Quat::from_rotation_x(angle), 1.0e-3));

        let from_context = with_joints(&mut app, |_, joints| {
            joints.multibody_joint_state(arm).unwrap()
        });
        assert_eq!(state, from_context);
        with_joints(&mut app, |bodies, joints| {
            let velocities = joints.multibody_generalized_velocity(bodies, arm).unwrap();
            assert_eq!(velocities, &[state.velocities[ROT_AXIS as usize]]);
            let raw = joints
                .multibody_joints
                .get(joints.entity2multibody_joint()[&arm]);
            let (mb, link_id) = raw.unwrap();
            assert_eq!(
                RapierContextJoints::entity_from_multibody_joint(mb.link(link_id).unwrap().joint()),
                arm
            );
        });
    }

    #[test]
    fn multibody_topology_helpers() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let other_ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 5.0);
        let b1 = spawn_link(&mut app, ground, Vect::X, revolute(Vect::ZERO));
        let b2 = spawn_link(&mut app, b1, Vect::X * 2.0, revolute(Vect::ZERO));
        let b3 = spawn_link(&mut app, b1, vect(2.0, -1.0), revolute(-Vect::Y));
        run(&mut app, 1);

        with_joints(&mut app, |bodies, joints| {
            for entity in [ground, b1, b2, b3] {
                assert_eq!(joints.multibody_root(bodies, entity), Some(ground));
            }
            assert_eq!(joints.multibody_root(bodies, other_ground), None);

            let links: Vec<_> = joints.multibody_links(bodies, b2).collect();
            assert_eq!(links.len(), 4);
            assert_eq!(links[0], ground);
            for entity in [b1, b2, b3] {
                assert!(links.contains(&entity));
            }

            assert_eq!(joints.multibody_parent_link(bodies, ground), None);
            assert_eq!(joints.multibody_parent_link(bodies, b1), Some(ground));
            assert_eq!(joints.multibody_parent_link(bodies, b2), Some(b1));
            assert_eq!(joints.multibody_parent_link(bodies, b3), Some(b1));

            assert_eq!(joints.multibody_joint_between(bodies, b1, b2), Some(b2));
            assert_eq!(joints.multibody_joint_between(bodies, b2, b1), Some(b2));
            assert_eq!(joints.multibody_joint_between(bodies, ground, b2), None);

            let mut attached: Vec<_> = joints.attached_multibody_joints(bodies, b1).collect();
            attached.sort();
            let mut expected = vec![b1, b2, b3];
            expected.sort();
            assert_eq!(attached, expected);
        });

        // Re-parenting moves the joint (and its descendants) to the new parent.
        app.world_mut()
            .get_mut::<MultibodyJoint>(b1)
            .unwrap()
            .parent = other_ground;
        run(&mut app, 1);
        with_joints(&mut app, |bodies, joints| {
            assert_eq!(joints.multibody_parent_link(bodies, b1), Some(other_ground));
            assert_eq!(joints.multibody_root(bodies, b2), Some(other_ground));
            assert_eq!(joints.multibody_root(bodies, ground), None);
        });

        // Removing all the joints of a multibody.
        let joint_entities: Vec<_> = with_joints(&mut app, |bodies, joints| {
            joints.multibody_links(bodies, b1).skip(1).collect()
        });
        for entity in joint_entities {
            app.world_mut()
                .entity_mut(entity)
                .remove::<MultibodyJoint>();
        }
        run(&mut app, 1);
        with_joints(&mut app, |bodies, joints| {
            assert_eq!(joints.multibody_joints.multibodies().count(), 0);
            assert_eq!(joints.multibody_root(bodies, b1), None);
        });
    }

    #[test]
    fn multibody_dof_couplings() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let b1 = spawn_link(&mut app, ground, Vect::Y, prismatic(Vect::Y));
        let b2 = spawn_link(&mut app, ground, -Vect::Y, prismatic(-Vect::Y));
        app.world_mut()
            .entity_mut(b2)
            .insert(MultibodyJointCouplings(vec![MultibodyJointCoupling::new(
                JointAxis::LinX,
                b1,
                JointAxis::LinX,
                -1.0,
                0.0,
            )]));
        run(&mut app, 1);
        with_joints_mut(&mut app, |bodies, joints| {
            joints.multibody_joint_velocity_mut(bodies, b1).unwrap()[0] = 1.0;
        });
        run(&mut app, 30);

        let x1 = joint_state(&app, b1).coords[0];
        let x2 = joint_state(&app, b2).coords[0];
        assert!(x1 > 0.1, "the first joint should move: {x1}");
        assert!(
            (x1 + x2).abs() < 1.0e-2,
            "the joints should be coupled: {x1} {x2}"
        );
        let num_couplings = |app: &mut App| {
            with_joints(app, |bodies, joints| {
                joints.multibody(bodies, b1).unwrap().0.couplings().len()
            })
        };
        assert_eq!(num_couplings(&mut app), 1);

        // Removing the coupling keeps the joint state.
        app.world_mut()
            .entity_mut(b2)
            .remove::<MultibodyJointCouplings>();
        run(&mut app, 1);
        assert_eq!(num_couplings(&mut app), 0);
        let new_x2 = joint_state(&app, b2).coords[0];
        assert!((new_x2 - x2).abs() < 0.1, "{x2} -> {new_x2}");
        assert_eq!(
            with_joints(&mut app, |bodies, joints| joints.multibody_root(bodies, b2)),
            Some(ground)
        );
    }

    #[test]
    fn multibody_couplings_and_self_contacts_follow_topology_changes() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let b1 = spawn_link(&mut app, ground, Vect::Y, prismatic(Vect::Y));
        // `b2` declares its coupling and disabled self-contacts before being attached.
        let b2 = spawn_body(&mut app, RigidBody::Dynamic, 0.0, -1.0);
        let coupling = |coeff| {
            MultibodyJointCouplings(vec![MultibodyJointCoupling::new(
                JointAxis::LinX,
                b1,
                JointAxis::LinX,
                coeff,
                0.0,
            )])
        };
        app.world_mut().entity_mut(b2).insert((
            ColliderMassProperties::Mass(1.0),
            MultibodySelfContactsDisabled,
            coupling(-1.0),
        ));
        run(&mut app, 1);

        let read = |app: &mut App| {
            with_joints(app, |bodies, joints| {
                let (mb, _) = joints.multibody(bodies, b1).unwrap();
                let coeffs: Vec<_> = mb.couplings().iter().map(|c| c.coeff).collect();
                (coeffs, mb.self_contacts_enabled())
            })
        };
        assert_eq!(read(&mut app), (vec![], true));

        app.world_mut()
            .entity_mut(b2)
            .insert(MultibodyJoint::new(ground, prismatic(-Vect::Y)));
        run(&mut app, 1);
        assert_eq!(read(&mut app), (vec![-1.0], false));

        // Editing the coupling replaces it.
        app.world_mut().entity_mut(b2).insert(coupling(2.0));
        run(&mut app, 1);
        assert_eq!(read(&mut app), (vec![2.0], false));

        // Attaching another link keeps the coupling and the disabled self-contacts.
        let b3 = spawn_link(&mut app, b1, Vect::Y * 2.0, prismatic(Vect::Y));
        run(&mut app, 1);
        assert_eq!(read(&mut app), (vec![2.0], false));
        assert_eq!(
            with_joints(&mut app, |bodies, joints| joints.multibody_root(bodies, b3)),
            Some(ground)
        );

        // Re-attaching the coupling source drops its coupling, which is then declared again.
        app.world_mut()
            .get_mut::<MultibodyJoint>(b1)
            .unwrap()
            .parent = b2;
        run(&mut app, 1);
        assert_eq!(read(&mut app), (vec![2.0], false));
        with_joints(&mut app, |bodies, joints| {
            let parent = |entity| {
                let (mb, link_id) = joints.multibody(bodies, entity).unwrap();
                let parent_id = mb.link(link_id).unwrap().parent_id().unwrap();
                mb.link(parent_id).unwrap().rigid_body_handle()
            };
            assert_eq!(parent(b1), bodies.entity2body()[&b2]);
        });
    }

    #[test]
    fn multibody_self_contacts_armature_and_damping() {
        let mut app = test_app();
        let ground = spawn_body(&mut app, RigidBody::Fixed, 0.0, 0.0);
        let arm = spawn_link(&mut app, ground, Vect::X, revolute(Vect::ZERO));
        app.world_mut()
            .entity_mut(ground)
            .insert(MultibodySelfContactsDisabled);
        app.world_mut().entity_mut(arm).insert((
            MultibodyJointArmature::splat(0.5),
            MultibodyJointDamping::splat(2.0),
        ));
        run(&mut app, 1);

        let read = |app: &mut App| {
            with_joints(app, |bodies, joints| {
                let (mb, _) = joints.multibody(bodies, arm).unwrap();
                let dof = joints.multibody_link_dofs(bodies, arm).unwrap().start;
                (
                    mb.self_contacts_enabled(),
                    mb.armature()[dof],
                    mb.damping()[dof],
                )
            })
        };
        assert_eq!(read(&mut app), (false, 0.5, 2.0));

        app.world_mut()
            .entity_mut(ground)
            .remove::<MultibodySelfContactsDisabled>();
        app.world_mut()
            .entity_mut(arm)
            .remove::<(MultibodyJointArmature, MultibodyJointDamping)>();
        run(&mut app, 1);
        let default_damping = MultibodyJointDamping::default().0[ROT_AXIS as usize];
        assert_eq!(read(&mut app), (true, 0.0, default_damping));
    }

    #[test]
    fn multibody_joint_properties_apply_to_late_joints() {
        let mut app = test_app();
        let ground = app
            .world_mut()
            .spawn(Transform::from_xyz(0.0, 0.0, 0.0))
            .id();
        let link = spawn_link(&mut app, ground, Vect::ZERO, prismatic(Vect::ZERO));
        app.world_mut().entity_mut(link).insert((
            KinematicMultibodyJoint,
            MultibodyJointSprings::default().with(JointAxis::LinX, 10.0, 2.0),
        ));
        run(&mut app, 2);
        assert!(app
            .world()
            .get::<RapierMultibodyJointHandle>(link)
            .is_none());

        // The joint is only created once its parent gets a rigid-body.
        app.world_mut().entity_mut(ground).insert(RigidBody::Fixed);
        run(&mut app, 1);
        with_joints(&mut app, |bodies, joints| {
            let (mb, link_id) = joints.multibody(bodies, link).unwrap();
            let joint = mb.link(link_id).unwrap().joint();
            assert!(joint.kinematic);
            assert_eq!(joint.spring(0), (10.0, 2.0));
        });
    }
}
