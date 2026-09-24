//! Systems synchronizing the soft-body components with Rapier.

#[cfg(feature = "fem")]
use crate::dynamics::SoftBodyElasticitySolver;
use crate::dynamics::{
    RapierRigidBodyHandle, RapierSoftBody, RapierSoftBodyHandle, RapierSoftBodyTearEvent, SoftBody,
    SoftBodyAttachment, SoftBodyAttachments, SoftBodyBuilder, SoftBodyCluster,
    SoftBodyClusterKinematicTarget, SoftBodyClusterMaterial, SoftBodyClusterPinned,
    SoftBodyClusterShapeMatching, SoftBodyDisabled, SoftBodyExternalForce, SoftBodyExternalImpulse,
    SoftBodyKinematicTargets, SoftBodyMaterial, SoftBodyPinnedParticles, SoftBodyState,
    SoftBodyVolumeFactor,
};
use crate::geometry::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, CollisionGroups, ContactForceEventThreshold,
    ContactSkin, Friction, RapierColliderHandle, Restitution, Sensor, SolverGroups,
};
use crate::pipeline::{
    SoftBodyClusterSplit, SoftBodyJointMove, SoftBodyTearEvent, SoftBodyTearPiece,
};
use crate::plugin::context::{
    RapierContextColliders, RapierContextEntityLink, RapierContextJoints, RapierContextSimulation,
    RapierRigidBodySet,
};
use crate::utils;
use bevy::ecs::query::QueryItem;
use bevy::prelude::*;

use super::RapierContextLinkResolver;
use rapier::dynamics::{RigidBodyHandle, RigidBodySet, SoftBodyHandle};
use rapier::geometry::Collider as RapierCollider;
use std::collections::HashMap;

/// The collider components of a soft body entity, applied to its surface colliders.
pub type SoftBodyColliderComponents<'a> = (
    Option<&'a Friction>,
    Option<&'a Restitution>,
    Option<&'a CollisionGroups>,
    Option<&'a SolverGroups>,
    Option<&'a ActiveEvents>,
    Option<&'a ActiveHooks>,
    Option<&'a ActiveCollisionTypes>,
    Option<&'a ContactForceEventThreshold>,
    Option<&'a ContactSkin>,
    Has<Sensor>,
);

/// The filter matching the soft bodies whose [`SoftBodyColliderComponents`] must be applied.
type SoftBodyColliderChanges = (
    With<RapierSoftBodyHandle>,
    Or<(
        Changed<Friction>,
        Changed<Restitution>,
        Changed<CollisionGroups>,
        Changed<SolverGroups>,
        Changed<ActiveEvents>,
        Changed<ActiveHooks>,
        Changed<ActiveCollisionTypes>,
        Changed<ContactForceEventThreshold>,
        Changed<ContactSkin>,
        Changed<Sensor>,
        Added<RapierSoftBodyHandle>,
    )>,
);

/// The components of a soft body entity read when the soft body is created.
pub type SoftBodyComponents<'a> = (
    (Entity, Option<&'a RapierContextEntityLink>),
    &'a SoftBody,
    Option<&'a GlobalTransform>,
    Option<&'a SoftBodyMaterial>,
    Option<&'a SoftBodyVolumeFactor>,
    Option<&'a SoftBodyPinnedParticles>,
    Option<&'a SoftBodyDisabled>,
);

/// The state written back into [`SoftBodyState`].
fn soft_body_state(sb: &RapierSoftBody) -> SoftBodyState {
    SoftBodyState {
        center_of_mass: sb.center_of_mass(),
        is_sleeping: sb.is_sleeping(),
        is_enabled: sb.is_enabled(),
        topology_version: sb.topology_version(),
        num_particles: sb.num_particles(),
    }
}

/// Applies the collider components of a soft body entity to one of its surface colliders.
fn apply_collider_components(
    co: &mut RapierCollider,
    (
        friction,
        restitution,
        collision_groups,
        solver_groups,
        active_events,
        active_hooks,
        active_collision_types,
        force_threshold,
        contact_skin,
        sensor,
    ): QueryItem<SoftBodyColliderComponents>,
) {
    if let Some(friction) = friction {
        co.set_friction(friction.coefficient);
        co.set_friction_combine_rule(friction.combine_rule.into());
    }
    if let Some(restitution) = restitution {
        co.set_restitution(restitution.coefficient);
        co.set_restitution_combine_rule(restitution.combine_rule.into());
    }
    if let Some(groups) = collision_groups {
        co.set_collision_groups((*groups).into());
    }
    if let Some(groups) = solver_groups {
        co.set_solver_groups((*groups).into());
    }
    if let Some(events) = active_events {
        co.set_active_events((*events).into());
    }
    if let Some(hooks) = active_hooks {
        co.set_active_hooks((*hooks).into());
    }
    if let Some(types) = active_collision_types {
        co.set_active_collision_types((*types).into());
    }
    if let Some(threshold) = force_threshold {
        co.set_contact_force_event_threshold(threshold.0);
    }
    if let Some(skin) = contact_skin {
        co.set_contact_skin(skin.0);
    }
    if sensor {
        co.set_sensor(true);
    }
}

/// Transforms the world-space data of a soft-body builder by `transform`.
fn transform_builder(builder: &mut SoftBodyBuilder, transform: &GlobalTransform) {
    let affine = transform.affine();
    let apply = |p: &mut crate::math::Vect| {
        #[cfg(feature = "dim2")]
        {
            *p = affine.transform_point3(p.extend(0.0)).truncate();
        }
        #[cfg(feature = "dim3")]
        {
            *p = affine.transform_point3(*p);
        }
    };
    builder.positions.iter_mut().for_each(apply);
    if let Some((vertices, _)) = &mut builder.skin {
        vertices.iter_mut().for_each(apply);
    }
}

/// Pins exactly the particles listed in `pinned` (invalid indices are ignored).
fn set_pinned_particles(sb: &mut RapierSoftBody, pinned: &[u32]) {
    let mut target = vec![false; sb.num_particles()];
    for &i in pinned {
        if let Some(pin) = target.get_mut(i as usize) {
            *pin = true;
        }
    }
    for (i, pin) in target.into_iter().enumerate() {
        if sb.particles()[i].is_pinned() != pin {
            sb.set_particle_pinned(i, pin);
        }
    }
}

/// The particles of the clusters of `sb` with a [`SoftBodyClusterPinned`] component.
fn pinned_cluster_particles(
    sb: &RapierSoftBody,
    entity2body: &HashMap<Entity, RigidBodyHandle>,
    pinned_clusters: &Query<Entity, With<SoftBodyClusterPinned>>,
) -> Vec<u32> {
    let mut particles = vec![];
    for entity in pinned_clusters {
        let Some(proxy) = entity2body.get(&entity) else {
            continue;
        };
        if let Some((_, cluster)) = sb.live_clusters().find(|(_, c)| c.proxy() == *proxy) {
            particles.extend_from_slice(cluster.particles());
        }
    }
    particles
}

/// Makes the particle attachments of `sb` match `attachments`.
///
/// Only the particles whose set of attached bodies changed are detached and attached again, so
/// the other attachments keep their anchor. Returns `false` if some bodies don't exist yet.
fn update_attachments(
    sb: &mut RapierSoftBody,
    attachments: &[SoftBodyAttachment],
    bodies: &RigidBodySet,
    entity2body: &HashMap<Entity, RigidBodyHandle>,
) -> bool {
    let mut complete = true;
    let mut desired: HashMap<u32, Vec<RigidBodyHandle>> = HashMap::new();
    for attachment in attachments {
        let Some(body) = entity2body.get(&attachment.body) else {
            // The rigid-body may not be created yet: try again next frame.
            complete = false;
            continue;
        };
        if (attachment.particle as usize) < sb.num_particles() {
            desired.entry(attachment.particle).or_default().push(*body);
        }
    }
    let mut current: HashMap<u32, Vec<RigidBodyHandle>> = HashMap::new();
    for attachment in sb.particle_attachments() {
        current
            .entry(attachment.particle)
            .or_default()
            .push(attachment.body);
    }
    let sorted = |mut bodies: Vec<RigidBodyHandle>| {
        bodies.sort_by_key(|h| h.into_raw_parts());
        bodies
    };
    let mut particles: Vec<u32> = desired.keys().chain(current.keys()).copied().collect();
    particles.sort_unstable();
    particles.dedup();
    for particle in particles {
        let wanted = sorted(desired.remove(&particle).unwrap_or_default());
        let existing = sorted(current.remove(&particle).unwrap_or_default());
        if wanted != existing {
            sb.detach_particle(particle as usize);
            for body in wanted {
                sb.attach_particle(particle as usize, body, bodies);
            }
        }
    }
    complete
}

/// Detaches every particle attached to a rigid-body.
fn detach_all_particles(sb: &mut RapierSoftBody) {
    let mut attached: Vec<u32> = sb
        .particle_attachments()
        .iter()
        .map(|a| a.particle)
        .collect();
    attached.dedup();
    for particle in attached {
        sb.detach_particle(particle as usize);
    }
}

/// System responsible for creating the Rapier soft bodies of new [`SoftBody`] components.
pub fn init_soft_bodies(
    mut commands: Commands,
    context_links: RapierContextLinkResolver,
    mut context_access: Query<(&mut RapierRigidBodySet, &mut RapierContextColliders)>,
    soft_bodies: Query<SoftBodyComponents, Without<RapierSoftBodyHandle>>,
) {
    for (
        (entity, entity_context_link),
        soft_body,
        transform,
        material,
        volume_factor,
        pinned,
        disabled,
    ) in soft_bodies.iter()
    {
        // Use the RapierContextEntityLink, or insert the context of an ancestor or the default one.
        let context_entity = context_links.resolve(entity, entity_context_link, &mut commands);
        let Some(context_entity) = context_entity else {
            continue;
        };
        let Ok((mut rigidbody_set, mut context_colliders)) = context_access.get_mut(context_entity)
        else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let rigidbody_set = &mut *rigidbody_set;

        let mut builder = soft_body.builder.clone();
        if let Some(transform) = transform {
            transform_builder(&mut builder, transform);
        }
        if let Some(material) = material {
            builder.material = material.0;
        }
        if let Some(volume_factor) = volume_factor {
            builder.volume_factor = volume_factor.0;
        }
        if let Some(pinned) = pinned {
            builder.pinned = pinned.0.clone();
        }
        let user_data = entity.to_bits() as u128;
        builder.user_data = user_data;
        // The collider components are applied by `apply_soft_body_user_changes`.
        builder.collider_template = builder
            .collider_template
            .take()
            .map(|template| template.user_data(user_data));

        let handle = rigidbody_set.soft_bodies.insert(
            builder,
            &mut rigidbody_set.bodies,
            &mut context_colliders.colliders,
        );
        let sb = &mut rigidbody_set.soft_bodies[handle];
        if disabled.is_some() {
            sb.set_enabled(false);
        }
        let state = soft_body_state(sb);
        rigidbody_set.map_soft_body_collision_mesh(&mut context_colliders, handle, entity);
        rigidbody_set.entity2soft_body.insert(entity, handle);
        rigidbody_set.map_soft_body_whole_proxy(entity, handle);
        commands
            .entity(entity)
            .insert((RapierSoftBodyHandle(handle), state));
    }
}

/// System responsible for creating the proxy rigid-bodies of new [`SoftBodyCluster`] components.
pub fn init_soft_body_clusters(
    mut commands: Commands,
    mut context_access: Query<(&mut RapierRigidBodySet, &mut RapierContextColliders)>,
    links: Query<&RapierContextEntityLink>,
    clusters: Query<(Entity, &SoftBodyCluster), Without<RapierRigidBodyHandle>>,
) {
    for (entity, cluster) in clusters.iter() {
        if entity == cluster.soft_body {
            log::error!(
                "The `SoftBodyCluster` of {entity} must be on another entity than its soft body."
            );
            commands.entity(entity).remove::<SoftBodyCluster>();
            continue;
        }
        // The soft body may not be created yet: try again next frame.
        let Ok(link) = links.get(cluster.soft_body) else {
            continue;
        };
        let Ok((mut rigidbody_set, mut context_colliders)) = context_access.get_mut(link.0) else {
            continue;
        };
        let rigidbody_set = &mut *rigidbody_set;
        let Some(handle) = rigidbody_set
            .entity2soft_body
            .get(&cluster.soft_body)
            .copied()
        else {
            continue;
        };

        let Some(index) = rigidbody_set.soft_bodies.add_cluster(
            handle,
            &cluster.particles,
            &mut rigidbody_set.bodies,
            &mut context_colliders.colliders,
        ) else {
            log::error!("The `SoftBodyCluster` of {entity} doesn't contain any valid particle.");
            commands.entity(entity).remove::<SoftBodyCluster>();
            continue;
        };
        let Some(proxy) = rigidbody_set.soft_bodies[handle].cluster_proxy(index) else {
            continue;
        };
        if let Some(rb) = rigidbody_set.bodies.get_mut(proxy) {
            rb.user_data = entity.to_bits() as u128;
        }
        rigidbody_set.entity2body.insert(entity, proxy);
        commands
            .entity(entity)
            .insert((RapierRigidBodyHandle(proxy), *link));
    }
}

/// System responsible for applying changes the user made to a soft-body-related component.
pub fn apply_soft_body_user_changes(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    (changed_materials, changed_volume_factors, changed_disabled): (
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                &SoftBodyMaterial,
            ),
            Or<(Changed<SoftBodyMaterial>, Added<RapierSoftBodyHandle>)>,
        >,
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                &SoftBodyVolumeFactor,
            ),
            Or<(Changed<SoftBodyVolumeFactor>, Added<RapierSoftBodyHandle>)>,
        >,
        Query<
            (&RapierSoftBodyHandle, &RapierContextEntityLink),
            (
                With<SoftBodyDisabled>,
                Or<(Changed<SoftBodyDisabled>, Added<RapierSoftBodyHandle>)>,
            ),
        >,
    ),
    (changed_pinned, changed_targets): (
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                &SoftBodyPinnedParticles,
            ),
            Or<(
                Changed<SoftBodyPinnedParticles>,
                Added<RapierSoftBodyHandle>,
            )>,
        >,
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                &SoftBodyKinematicTargets,
            ),
            Or<(
                Changed<SoftBodyKinematicTargets>,
                Added<RapierSoftBodyHandle>,
            )>,
        >,
    ),
    mut changed_attachments: Query<
        (
            &RapierSoftBodyHandle,
            &RapierContextEntityLink,
            Mut<SoftBodyAttachments>,
        ),
        Or<(Changed<SoftBodyAttachments>, Added<RapierSoftBodyHandle>)>,
    >,
    (changed_forces, mut changed_impulses): (
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                &SoftBodyExternalForce,
            ),
            Or<(Changed<SoftBodyExternalForce>, Added<RapierSoftBodyHandle>)>,
        >,
        Query<
            (
                &RapierSoftBodyHandle,
                &RapierContextEntityLink,
                Mut<SoftBodyExternalImpulse>,
            ),
            Or<(
                Changed<SoftBodyExternalImpulse>,
                Added<RapierSoftBodyHandle>,
            )>,
        >,
    ),
    #[cfg(feature = "fem")] changed_solvers: Query<
        (
            &RapierSoftBodyHandle,
            &RapierContextEntityLink,
            &SoftBodyElasticitySolver,
        ),
        Or<(
            Changed<SoftBodyElasticitySolver>,
            Added<RapierSoftBodyHandle>,
        )>,
    >,
    pinned_clusters: Query<Entity, With<SoftBodyClusterPinned>>,
    (mut context_colliders, changed_colliders): (
        Query<&mut RapierContextColliders>,
        Query<
            (Entity, &RapierContextEntityLink, SoftBodyColliderComponents),
            SoftBodyColliderChanges,
        >,
    ),
) {
    for (entity, link, components) in changed_colliders.iter() {
        let (Ok(rigidbody_set), Ok(mut colliders)) = (
            rigid_body_sets.get(link.0),
            context_colliders.get_mut(link.0),
        ) else {
            continue;
        };
        for handle in rigidbody_set
            .soft_body_colliders(&colliders.colliders, entity)
            .unwrap_or_default()
        {
            if let Some(co) = colliders.colliders.get_mut(handle) {
                apply_collider_components(co, components);
            }
        }
    }

    let mut with_soft_body = |link: &RapierContextEntityLink,
                              handle: &RapierSoftBodyHandle,
                              f: &mut dyn FnMut(
        &mut RapierSoftBody,
        &RigidBodySet,
        &HashMap<Entity, RigidBodyHandle>,
    )| {
        let Ok(mut rigidbody_set) = rigid_body_sets.get_mut(link.0) else {
            return false;
        };
        let rigidbody_set = &mut *rigidbody_set;
        let Some(sb) = rigidbody_set.soft_bodies.get_mut(handle.0) else {
            return false;
        };
        f(sb, &rigidbody_set.bodies, &rigidbody_set.entity2body);
        true
    };

    for (handle, link, material) in changed_materials.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| sb.set_material(material.0));
    }

    for (handle, link, factor) in changed_volume_factors.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| sb.set_volume_factor(factor.0));
    }

    for (handle, link) in changed_disabled.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| sb.set_enabled(false));
    }

    #[cfg(feature = "fem")]
    for (handle, link, solver) in changed_solvers.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| {
            if sb.solver() != solver.0 {
                sb.set_solver(solver.0);
            }
        });
    }

    for (handle, link, pinned) in changed_pinned.iter() {
        with_soft_body(link, handle, &mut |sb, _, entity2body| {
            let mut all_pinned = pinned.0.clone();
            all_pinned.extend(pinned_cluster_particles(sb, entity2body, &pinned_clusters));
            set_pinned_particles(sb, &all_pinned)
        });
    }

    for (handle, link, targets) in changed_targets.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| {
            for (i, target) in &targets.0 {
                if (*i as usize) < sb.num_particles() {
                    sb.set_particle_kinematic_target(*i as usize, *target);
                }
            }
        });
    }

    for (handle, link, mut attachments) in changed_attachments.iter_mut() {
        let mut retry = false;
        with_soft_body(link, handle, &mut |sb, bodies, entity2body| {
            retry = !update_attachments(sb, &attachments.0, bodies, entity2body);
        });
        if retry {
            attachments.set_changed();
        }
    }

    for (handle, link, forces) in changed_forces.iter() {
        with_soft_body(link, handle, &mut |sb, _, _| {
            sb.reset_forces(false);
            sb.add_force(forces.force, true);
            for (i, force) in &forces.particle_forces {
                if (*i as usize) < sb.num_particles() {
                    sb.add_particle_force(*i as usize, *force, true);
                }
            }
        });
    }

    for (handle, link, mut impulses) in changed_impulses.iter_mut() {
        let applied = with_soft_body(link, handle, &mut |sb, _, _| {
            if impulses.velocity_change != crate::math::Vect::ZERO {
                sb.apply_impulse(impulses.velocity_change, true);
            }
            for (i, impulse) in &impulses.particle_impulses {
                if (*i as usize) < sb.num_particles() {
                    sb.apply_particle_impulse(*i as usize, *impulse, true);
                }
            }
        });
        if applied {
            impulses.bypass_change_detection().reset();
        }
    }
}

/// System responsible for removing from Rapier the soft bodies whose [`SoftBody`] component was
/// removed by the user (through component removal or despawn).
pub fn sync_soft_body_removals(
    mut commands: Commands,
    mut contexts: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    has_handle: Query<(), With<RapierSoftBodyHandle>>,
    mut removed_handles: RemovedComponents<RapierSoftBodyHandle>,
    orphans: Query<Entity, (With<RapierSoftBodyHandle>, Without<SoftBody>)>,
) {
    let mut remove = |entity: Entity| {
        for (mut simulation, mut colliders, mut joints, mut rigidbody_set) in contexts.iter_mut() {
            if let Some(handle) = rigidbody_set.entity2soft_body.get(&entity).copied() {
                simulation.remove_soft_body(
                    &mut colliders,
                    &mut joints,
                    &mut rigidbody_set,
                    handle,
                );
                break;
            }
        }
    };

    for entity in removed_handles.read() {
        if !has_handle.contains(entity) {
            remove(entity);
        }
    }

    for entity in orphans.iter() {
        remove(entity);
        commands.entity(entity).remove::<RapierSoftBodyHandle>();
    }
}

/// Removal detection for a single component: the entities it was removed from, paired with a
/// query checking if it was re-inserted since.
type RemovedComponent<'w, 's, T> = (RemovedComponents<'w, 's, T>, Query<'w, 's, (), With<T>>);

/// System responsible for resetting the Rapier soft-body properties to the values of their
/// [`SoftBody`] builder when the corresponding component is removed by the user.
pub fn reset_removed_soft_body_components(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    soft_bodies: Query<&SoftBody>,
    (mut removed_materials, mut removed_volume_factors, mut removed_disabled): (
        RemovedComponent<SoftBodyMaterial>,
        RemovedComponent<SoftBodyVolumeFactor>,
        RemovedComponent<SoftBodyDisabled>,
    ),
    (mut removed_pinned, mut removed_attachments, mut removed_forces): (
        RemovedComponent<SoftBodyPinnedParticles>,
        RemovedComponent<SoftBodyAttachments>,
        RemovedComponent<SoftBodyExternalForce>,
    ),
    #[cfg(feature = "fem")] mut removed_solvers: RemovedComponent<SoftBodyElasticitySolver>,
    pinned_clusters: Query<Entity, With<SoftBodyClusterPinned>>,
) {
    let sets = &mut rigid_body_sets;
    let builders = &soft_bodies;
    reset_removed(&mut removed_materials, sets, builders, |sb, builder| {
        sb.set_material(builder.map(|b| b.material).unwrap_or_default());
    });
    reset_removed(
        &mut removed_volume_factors,
        sets,
        builders,
        |sb, builder| {
            sb.set_volume_factor(builder.map(|b| b.volume_factor).unwrap_or(1.0));
        },
    );
    reset_removed(&mut removed_disabled, sets, builders, |sb, _| {
        sb.set_enabled(true);
    });
    let unpinned = reset_removed(&mut removed_pinned, sets, builders, |sb, builder| {
        set_pinned_particles(sb, builder.map(|b| &b.pinned[..]).unwrap_or_default());
    });
    // Pin the pinned clusters again.
    if unpinned {
        for mut set in sets.iter_mut() {
            let set = &mut *set;
            for entity in &pinned_clusters {
                if let Some((handle, index)) = set.soft_body_cluster_index(entity) {
                    if let Some(sb) = set.soft_bodies.get_mut(handle) {
                        sb.set_cluster_pinned(index, true);
                    }
                }
            }
        }
    }
    reset_removed(&mut removed_attachments, sets, builders, |sb, _| {
        detach_all_particles(sb);
    });
    reset_removed(&mut removed_forces, sets, builders, |sb, _| {
        sb.reset_forces(true);
    });
    #[cfg(feature = "fem")]
    reset_removed(&mut removed_solvers, sets, builders, |sb, builder| {
        sb.set_solver(builder.map(|b| b.solver).unwrap_or_default());
    });
}

/// Calls `reset` on the Rapier soft body of each entity `T` was removed from, unless `T` was
/// inserted back since. Returns `true` if `reset` was called.
fn reset_removed<T: Component>(
    (removed, still_present): &mut RemovedComponent<T>,
    rigid_body_sets: &mut Query<&mut RapierRigidBodySet>,
    soft_bodies: &Query<&SoftBody>,
    mut reset: impl FnMut(&mut RapierSoftBody, Option<&SoftBodyBuilder>),
) -> bool {
    let mut any = false;
    for entity in removed.read() {
        if still_present.contains(entity) {
            continue;
        }

        for mut rigid_body_set in rigid_body_sets.iter_mut() {
            if let Some(sb) = rigid_body_set.soft_body_mut(entity) {
                reset(sb, soft_bodies.get(entity).ok().map(|sb| &sb.builder));
                any = true;
                break;
            }
        }
    }
    any
}

/// Calls `f` with the soft body and the index of the cluster standing for `entity` (see
/// [`RapierRigidBodySet::soft_body_cluster_index`]).
fn with_cluster(
    rigid_body_sets: &mut Query<&mut RapierRigidBodySet>,
    link: &RapierContextEntityLink,
    entity: Entity,
    f: impl FnOnce(&mut RapierSoftBody, u32),
) {
    let Ok(mut set) = rigid_body_sets.get_mut(link.0) else {
        return;
    };
    let Some((handle, index)) = set.soft_body_cluster_index(entity) else {
        return;
    };
    if let Some(sb) = set.soft_bodies.get_mut(handle) {
        f(sb, index);
    }
}

/// Applies a [`SoftBodyClusterMaterial`] to the `index`-th cluster of `sb`.
fn apply_cluster_material(sb: &mut RapierSoftBody, index: u32, material: &SoftBodyClusterMaterial) {
    sb.set_cluster_stiffness_scale(index, material.stiffness_scale);
    sb.set_cluster_edge_softness(index, material.edge_softness);
    sb.set_cluster_tear_resistance(index, material.tear_resistance);
}

/// The entities whose cluster was just created: the cluster entities with a new proxy, and the
/// soft body entities (for their whole-body cluster) with a new soft body.
type NewCluster = Or<(Added<RapierRigidBodyHandle>, Added<RapierSoftBodyHandle>)>;

/// System responsible for applying the changes the user made to the cluster components
/// ([`SoftBodyClusterPinned`], [`SoftBodyClusterKinematicTarget`],
/// [`SoftBodyClusterShapeMatching`] and [`SoftBodyClusterMaterial`]).
pub fn apply_soft_body_cluster_changes(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    pinned: Query<
        (Entity, &RapierContextEntityLink),
        (
            With<SoftBodyClusterPinned>,
            Or<(Changed<SoftBodyClusterPinned>, NewCluster)>,
        ),
    >,
    targets: Query<
        (
            Entity,
            &RapierContextEntityLink,
            &SoftBodyClusterKinematicTarget,
        ),
        Or<(Changed<SoftBodyClusterKinematicTarget>, NewCluster)>,
    >,
    shape_matching: Query<
        (
            Entity,
            &RapierContextEntityLink,
            &SoftBodyClusterShapeMatching,
        ),
        Or<(Changed<SoftBodyClusterShapeMatching>, NewCluster)>,
    >,
    materials: Query<
        (Entity, &RapierContextEntityLink, &SoftBodyClusterMaterial),
        Or<(Changed<SoftBodyClusterMaterial>, NewCluster)>,
    >,
) {
    let sets = &mut rigid_body_sets;
    for (entity, link) in &pinned {
        with_cluster(sets, link, entity, |sb, i| sb.set_cluster_pinned(i, true));
    }
    for (entity, link, target) in &targets {
        let pose = utils::transform_to_iso(&target.0);
        with_cluster(sets, link, entity, |sb, i| {
            sb.set_cluster_kinematic_target(i, pose)
        });
    }
    for (entity, link, shape_matching) in &shape_matching {
        let target = shape_matching.target.as_ref().map(utils::transform_to_iso);
        with_cluster(sets, link, entity, |sb, i| {
            sb.enable_cluster_shape_matching(i, true);
            if let Some(cluster) = sb.cluster_mut(i) {
                cluster.set_shape_matching_target(target);
            }
        });
    }
    for (entity, link, material) in &materials {
        with_cluster(sets, link, entity, |sb, i| {
            apply_cluster_material(sb, i, material)
        });
    }
}

/// The context index, soft body and cluster index standing for each entity `T` was removed from,
/// unless `T` was inserted back since.
fn removed_clusters<T: Component>(
    (removed, still_present): &mut RemovedComponent<T>,
    rigid_body_sets: &Query<&mut RapierRigidBodySet>,
) -> Vec<(usize, SoftBodyHandle, u32)> {
    let mut result = vec![];
    for entity in removed.read() {
        if still_present.contains(entity) {
            continue;
        }
        for (set_index, set) in rigid_body_sets.iter().enumerate() {
            if let Some((handle, index)) = set.soft_body_cluster_index(entity) {
                result.push((set_index, handle, index));
                break;
            }
        }
    }
    result
}

/// System responsible for resetting the clusters whose [`SoftBodyClusterPinned`],
/// [`SoftBodyClusterShapeMatching`] or [`SoftBodyClusterMaterial`] component was removed.
///
/// This must run before the removal of the clusters of despawned entities.
pub fn reset_removed_soft_body_cluster_components(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    soft_bodies: Query<(&SoftBody, Option<&SoftBodyPinnedParticles>)>,
    pinned_clusters: Query<Entity, With<SoftBodyClusterPinned>>,
    (mut removed_pinned, mut removed_shape_matching, mut removed_materials): (
        RemovedComponent<SoftBodyClusterPinned>,
        RemovedComponent<SoftBodyClusterShapeMatching>,
        RemovedComponent<SoftBodyClusterMaterial>,
    ),
) {
    let unpinned = removed_clusters(&mut removed_pinned, &rigid_body_sets);
    let unmatched = removed_clusters(&mut removed_shape_matching, &rigid_body_sets);
    let unmaterialized = removed_clusters(&mut removed_materials, &rigid_body_sets);

    for (set_index, mut set) in rigid_body_sets.iter_mut().enumerate() {
        let set = &mut *set;
        for (_, handle, index) in unpinned.iter().filter(|(s, _, _)| *s == set_index) {
            let Some(sb) = set.soft_bodies.get(*handle) else {
                continue;
            };
            // The particles pinned without this cluster: by the soft body, or by other clusters.
            let components = set
                .soft_body_entity(*handle)
                .and_then(|entity| soft_bodies.get(entity).ok());
            let mut still_pinned = match components {
                Some((_, Some(pinned))) => pinned.0.clone(),
                Some((soft_body, None)) => soft_body.builder.pinned.clone(),
                None => vec![],
            };
            still_pinned.extend(pinned_cluster_particles(
                sb,
                &set.entity2body,
                &pinned_clusters,
            ));
            let particles = sb.cluster(*index).map(|c| c.particles().to_vec());
            let Some(sb) = set.soft_bodies.get_mut(*handle) else {
                continue;
            };
            for particle in particles.unwrap_or_default() {
                if (particle as usize) < sb.num_particles() {
                    sb.set_particle_pinned(particle as usize, still_pinned.contains(&particle));
                }
            }
        }
        for (_, handle, index) in unmatched.iter().filter(|(s, _, _)| *s == set_index) {
            let builder_setting = set
                .soft_body_entity(*handle)
                .and_then(|entity| soft_bodies.get(entity).ok())
                .is_some_and(|(soft_body, _)| soft_body.builder.shape_matching);
            let Some(sb) = set.soft_bodies.get_mut(*handle) else {
                continue;
            };
            // Only the whole-body cluster is shape-matched by the builder.
            let enabled = *index == 0 && builder_setting;
            sb.enable_cluster_shape_matching(*index, enabled);
            if let Some(cluster) = sb.cluster_mut(*index) {
                cluster.set_shape_matching_target(None);
            }
        }
        for (_, handle, index) in unmaterialized.iter().filter(|(s, _, _)| *s == set_index) {
            if let Some(sb) = set.soft_bodies.get_mut(*handle) {
                apply_cluster_material(sb, *index, &SoftBodyClusterMaterial::default());
            }
        }
    }
}

/// Writes `world` into `transform`, relative to the parent's global transform if any.
fn set_world_transform(
    transform: &mut Transform,
    mut world: Transform,
    parent: Option<&GlobalTransform>,
) {
    world.scale = transform.scale;
    if let Some(parent) = parent {
        world = GlobalTransform::from(world).reparented_to(parent);
        world.scale = transform.scale;
    }
    // In 2D, preserve the `z` component set by the user.
    #[cfg(feature = "dim2")]
    {
        world.translation.z = transform.translation.z;
    }
    if transform.translation != world.translation || transform.rotation != world.rotation {
        transform.translation = world.translation;
        transform.rotation = world.rotation;
    }
}

/// System responsible for writing the state of the soft bodies and cluster proxies back into
/// [`SoftBodyState`] and [`Transform`] components after a simulation step.
pub fn writeback_soft_bodies(
    rigid_body_sets: Query<&RapierRigidBodySet>,
    global_transforms: Query<&GlobalTransform>,
    mut soft_bodies: Query<(
        Entity,
        &RapierSoftBodyHandle,
        &RapierContextEntityLink,
        Option<&mut SoftBodyState>,
        Option<&mut Transform>,
        Option<&ChildOf>,
    )>,
    mut clusters: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &mut Transform,
            Option<&ChildOf>,
        ),
        (With<SoftBodyCluster>, Without<RapierSoftBodyHandle>),
    >,
) {
    for (entity, handle, link, state, transform, child_of) in soft_bodies.iter_mut() {
        let Some(set) = rigid_body_sets.get(link.0).ok() else {
            continue;
        };
        let Some(sb) = set.soft_bodies.get(handle.0) else {
            continue;
        };
        let new_state = soft_body_state(sb);
        if let Some(mut state) = state {
            state.set_if_neq(new_state);
        }
        if let Some(mut transform) = transform {
            // Follow the whole-body proxy, so the colliders attached to it by children entities
            // stay where their entity is.
            let pose = set
                .entity2body
                .get(&entity)
                .and_then(|proxy| set.bodies.get(*proxy))
                .map(|rb| utils::iso_to_transform(rb.position()))
                .unwrap_or_else(|| {
                    #[cfg(feature = "dim2")]
                    let translation = new_state.center_of_mass.extend(0.0);
                    #[cfg(feature = "dim3")]
                    let translation = new_state.center_of_mass;
                    Transform::from_translation(translation)
                });
            let parent = child_of.and_then(|c| global_transforms.get(c.parent()).ok());
            set_world_transform(&mut transform, pose, parent);
        }
    }

    for (handle, link, mut transform, child_of) in clusters.iter_mut() {
        let Some(rb) = rigid_body_sets
            .get(link.0)
            .ok()
            .and_then(|set| set.bodies.get(handle.0))
        else {
            continue;
        };
        let parent = child_of.and_then(|c| global_transforms.get(c.parent()).ok());
        set_world_transform(
            &mut transform,
            utils::iso_to_transform(rb.position()),
            parent,
        );
    }
}

/// The components of a soft body entity referring to its particles by index.
#[derive(Clone, Default)]
struct IndexedComponents {
    pinned: Option<SoftBodyPinnedParticles>,
    targets: Option<SoftBodyKinematicTargets>,
    attachments: Option<SoftBodyAttachments>,
    force: Option<SoftBodyExternalForce>,
    impulse: Option<SoftBodyExternalImpulse>,
    /// The particles pinned by the [`SoftBody`] builder.
    builder_pinned: Option<Vec<u32>>,
}

/// The query reading the [`IndexedComponents`] of an entity.
type IndexedComponentsQuery<'a> = (
    Option<&'a SoftBodyPinnedParticles>,
    Option<&'a SoftBodyKinematicTargets>,
    Option<&'a SoftBodyAttachments>,
    Option<&'a SoftBodyExternalForce>,
    Option<&'a SoftBodyExternalImpulse>,
    Option<&'a SoftBody>,
);

/// The cluster components of a cluster entity, inherited by the clusters split off its cluster.
type ClusterComponentsQuery<'a> = (
    Has<SoftBodyClusterPinned>,
    Option<&'a SoftBodyClusterKinematicTarget>,
    Option<&'a SoftBodyClusterShapeMatching>,
    Option<&'a SoftBodyClusterMaterial>,
);

impl IndexedComponents {
    fn new(
        (pinned, targets, attachments, force, impulse, soft_body): QueryItem<
            IndexedComponentsQuery,
        >,
    ) -> Self {
        Self {
            pinned: pinned.cloned(),
            targets: targets.cloned(),
            attachments: attachments.cloned(),
            force: force.cloned(),
            impulse: impulse.cloned(),
            builder_pinned: soft_body
                .map(|sb| sb.builder.pinned.clone())
                .filter(|pinned| !pinned.is_empty()),
        }
    }

    fn is_empty(&self) -> bool {
        self.pinned.is_none()
            && self.targets.is_none()
            && self.attachments.is_none()
            && self.force.is_none()
            && self.impulse.is_none()
            && self.builder_pinned.is_none()
    }

    /// Splits these components between the pieces of a tear, following the particles they refer
    /// to (the entries of the particles that no longer exist are dropped).
    fn split(&self, event: &RapierSoftBodyTearEvent, pieces: &[SoftBodyHandle]) -> Vec<Self> {
        let destination = |particle: u32| {
            let (handle, index) = event.particle_destination(particle)?;
            Some((pieces.iter().position(|h| *h == handle)?, index))
        };
        let split_list = |list: &[u32]| {
            let mut result = vec![vec![]; pieces.len()];
            for particle in list {
                if let Some((k, index)) = destination(*particle) {
                    result[k].push(index);
                }
            }
            result
        };
        let mut result = vec![Self::default(); pieces.len()];
        if let Some(pinned) = &self.pinned {
            for (r, pinned) in result.iter_mut().zip(split_list(&pinned.0)) {
                r.pinned = Some(SoftBodyPinnedParticles(pinned));
            }
        }
        if let Some(pinned) = &self.builder_pinned {
            for (r, pinned) in result.iter_mut().zip(split_list(pinned)) {
                r.builder_pinned = Some(pinned);
            }
        }
        if let Some(targets) = &self.targets {
            result.iter_mut().for_each(|r| r.targets = Some(default()));
            for (particle, target) in &targets.0 {
                if let Some((k, index)) = destination(*particle) {
                    result[k].targets.as_mut().unwrap().0.push((index, *target));
                }
            }
        }
        if let Some(attachments) = &self.attachments {
            result
                .iter_mut()
                .for_each(|r| r.attachments = Some(default()));
            for attachment in &attachments.0 {
                if let Some((k, index)) = destination(attachment.particle) {
                    result[k]
                        .attachments
                        .as_mut()
                        .unwrap()
                        .0
                        .push(SoftBodyAttachment {
                            particle: index,
                            body: attachment.body,
                        });
                }
            }
        }
        if let Some(force) = &self.force {
            result.iter_mut().for_each(|r| {
                r.force = Some(SoftBodyExternalForce {
                    force: force.force,
                    particle_forces: vec![],
                })
            });
            for (particle, particle_force) in &force.particle_forces {
                if let Some((k, index)) = destination(*particle) {
                    let forces = &mut result[k].force.as_mut().unwrap().particle_forces;
                    forces.push((index, *particle_force));
                }
            }
        }
        if let Some(impulse) = &self.impulse {
            result.iter_mut().for_each(|r| {
                r.impulse = Some(SoftBodyExternalImpulse {
                    velocity_change: impulse.velocity_change,
                    particle_impulses: vec![],
                })
            });
            for (particle, particle_impulse) in &impulse.particle_impulses {
                if let Some((k, index)) = destination(*particle) {
                    let impulses = &mut result[k].impulse.as_mut().unwrap().particle_impulses;
                    impulses.push((index, *particle_impulse));
                }
            }
        }
        result
    }

    /// Inserts these components on `entity`.
    fn insert(self, entity_commands: &mut EntityCommands) {
        if let Some(pinned) = self.pinned {
            entity_commands.try_insert(pinned);
        }
        if let Some(targets) = self.targets {
            entity_commands.try_insert(targets);
        }
        if let Some(attachments) = self.attachments {
            entity_commands.try_insert(attachments);
        }
        if let Some(force) = self.force {
            entity_commands.try_insert(force);
        }
        if let Some(impulse) = self.impulse {
            entity_commands.try_insert(impulse);
        }
        if let Some(pinned) = self.builder_pinned {
            // The builder is kept for the removal of `SoftBodyPinnedParticles`: keep it in sync.
            entity_commands.queue_silenced(move |mut entity: EntityWorldMut| {
                if let Some(mut soft_body) = entity.get_mut::<SoftBody>() {
                    soft_body.bypass_change_detection().builder.pinned = pinned;
                }
            });
        }
    }
}

/// The mass-weighted rest centroid of the particles of the given clusters, as
/// `(soft body, cluster index)` pairs.
fn rest_centroid(
    rigidbody_set: &RapierRigidBodySet,
    clusters: impl IntoIterator<Item = (SoftBodyHandle, u32)>,
) -> crate::math::Vect {
    let mut com = crate::math::Vect::ZERO;
    let mut mass = 0.0;
    for (handle, index) in clusters {
        let Some(sb) = rigidbody_set.soft_bodies.get(handle) else {
            continue;
        };
        for &v in sb.cluster(index).map(|c| c.particles()).unwrap_or_default() {
            let particle = &sb.particles()[v as usize];
            com += particle.rest_position() * particle.mass();
            mass += particle.mass();
        }
    }
    if mass > 0.0 {
        com / mass
    } else {
        com
    }
}

/// The kinematic target of a cluster split off the source cluster of `split`, placing its
/// particles where `source_target` placed them in the source cluster.
fn inherited_kinematic_target(
    rigidbody_set: &RapierRigidBodySet,
    raw: &RapierSoftBodyTearEvent,
    split: &rapier::dynamics::SoftClusterSplit,
    source_target: &SoftBodyClusterKinematicTarget,
) -> SoftBodyClusterKinematicTarget {
    // A target places the rest shape centered on the cluster's rest centroid, which moved.
    let source_centroid = rest_centroid(
        rigidbody_set,
        raw.clusters
            .iter()
            .filter(|s| s.source_cluster == split.source_cluster)
            .map(|s| (s.soft_body, s.cluster)),
    );
    let centroid = rest_centroid(rigidbody_set, [(split.soft_body, split.cluster)]);
    let delta = centroid - source_centroid;
    #[cfg(feature = "dim2")]
    let delta = delta.extend(0.0);
    let mut target = *source_target;
    target.0.translation += target.0.rotation * delta;
    target
}

/// System responsible for spawning the entities of the soft bodies split off by tears, and for
/// sending the corresponding [`SoftBodyTearEvent`] messages.
pub fn handle_soft_body_tears(
    mut commands: Commands,
    mut contexts: Query<(
        Entity,
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    mut clusters: Query<&mut SoftBodyCluster>,
    cluster_components: Query<ClusterComponentsQuery>,
    indexed_components: Query<IndexedComponentsQuery>,
    mut tear_events: MessageWriter<SoftBodyTearEvent>,
) {
    // The index-based components of the torn soft bodies and of their pieces, remapped through
    // every tear of this frame.
    let mut remapped: HashMap<Entity, IndexedComponents> = HashMap::new();

    for (context_entity, mut simulation, mut colliders, joints, mut rigidbody_set) in
        contexts.iter_mut()
    {
        if simulation.pending_soft_body_tears.is_empty() {
            continue;
        }
        let colliders = &mut *colliders;
        let rigidbody_set = &mut *rigidbody_set;
        for pending in std::mem::take(&mut simulation.pending_soft_body_tears) {
            let raw = pending.raw;
            // Soft bodies inserted without entity are left alone.
            let Some(torn_entity) = rigidbody_set.soft_body_entity(raw.soft_body) else {
                continue;
            };

            let mut pieces = Vec::with_capacity(raw.pieces.len());
            let mut piece_entities = Vec::with_capacity(raw.pieces.len());
            for (k, piece) in raw.pieces.iter().enumerate() {
                // The entity already spawned for this piece by a manual tear, if any.
                let reserved = k
                    .checked_sub(1)
                    .and_then(|i| pending.piece_entities.get(i))
                    .copied();
                let piece_entity = if k == 0 {
                    torn_entity
                } else {
                    let Some(sb) = rigidbody_set.soft_bodies.get(piece.soft_body) else {
                        if let Some(reserved) = reserved {
                            commands.entity(reserved).despawn();
                        }
                        continue;
                    };
                    let state = soft_body_state(sb);
                    let mut torn_commands = commands.entity(torn_entity);
                    let deny = |builder: &mut bevy::ecs::entity::EntityClonerBuilder<
                        bevy::ecs::entity::OptOut,
                    >| {
                        builder.deny::<(
                            RapierSoftBodyHandle,
                            SoftBodyState,
                            SoftBodyPinnedParticles,
                            SoftBodyKinematicTargets,
                            SoftBodyAttachments,
                            SoftBodyExternalForce,
                            SoftBodyExternalImpulse,
                            Children,
                            RapierRigidBodyHandle,
                            RapierColliderHandle,
                        )>();
                        #[cfg(all(feature = "to-bevy-mesh", feature = "dim3"))]
                        builder.deny::<bevy::mesh::Mesh3d>();
                        #[cfg(all(feature = "to-bevy-mesh", feature = "dim2"))]
                        builder.deny::<bevy::mesh::Mesh2d>();
                    };
                    let piece_entity = match reserved {
                        Some(reserved) => {
                            torn_commands.clone_with_opt_out(reserved, deny);
                            reserved
                        }
                        None => torn_commands.clone_and_spawn_with_opt_out(deny).id(),
                    };
                    commands.entity(piece_entity).insert((
                        RapierSoftBodyHandle(piece.soft_body),
                        state,
                        RapierContextEntityLink(context_entity),
                    ));
                    rigidbody_set.map_soft_body_piece(colliders, piece.soft_body, piece_entity);
                    piece_entity
                };
                rigidbody_set.map_soft_body_collision_mesh(
                    colliders,
                    piece.soft_body,
                    piece_entity,
                );
                rigidbody_set.map_soft_body_whole_proxy(piece_entity, piece.soft_body);

                // The cluster entities moved to this piece now belong to its soft body.
                if let Some(sb) = rigidbody_set.soft_bodies.get(piece.soft_body) {
                    for [_, cluster] in &piece.clusters {
                        let Some(proxy_entity) = sb
                            .cluster_proxy(*cluster)
                            .and_then(|proxy| rigidbody_set.rigid_body_entity(proxy))
                        else {
                            continue;
                        };
                        if let Ok(mut cluster) = clusters.get_mut(proxy_entity) {
                            if cluster.soft_body != piece_entity {
                                cluster.soft_body = piece_entity;
                            }
                        }
                    }
                }

                piece_entities.push((piece.soft_body, piece_entity));
                pieces.push(SoftBodyTearPiece {
                    soft_body: piece_entity,
                    particles: piece.particles.clone(),
                });
            }
            if raw.pieces.is_empty() {
                rigidbody_set.map_soft_body_collision_mesh(colliders, raw.soft_body, torn_entity);
                rigidbody_set.map_soft_body_whole_proxy(torn_entity, raw.soft_body);
            } else {
                // The particles were renumbered: remap the components referring to them.
                let source = remapped.remove(&torn_entity).unwrap_or_else(|| {
                    indexed_components
                        .get(torn_entity)
                        .map(IndexedComponents::new)
                        .unwrap_or_default()
                });
                if !source.is_empty() {
                    let handles: Vec<_> = piece_entities.iter().map(|(h, _)| *h).collect();
                    let split = source.split(&raw, &handles);
                    for ((_, entity), components) in piece_entities.iter().zip(split) {
                        remapped.insert(*entity, components);
                    }
                }
            }

            let mut cluster_splits = Vec::with_capacity(raw.clusters.len());
            for split in &raw.clusters {
                let Some(soft_body) = rigidbody_set.soft_body_entity(split.soft_body) else {
                    continue;
                };
                let mut proxy = rigidbody_set.rigid_body_entity(split.proxy);
                if !split.keeps_proxy {
                    // The entity of the split cluster: a cluster entity or a soft body entity.
                    let source = raw
                        .clusters
                        .iter()
                        .find(|s| s.source_cluster == split.source_cluster && s.keeps_proxy)
                        .and_then(|s| rigidbody_set.rigid_body_entity(s.proxy));
                    let (pinned, target, shape_matching, material) = source
                        .and_then(|e| cluster_components.get(e).ok())
                        .unwrap_or_default();
                    let target = target.map(|target| {
                        inherited_kinematic_target(rigidbody_set, &raw, split, target)
                    });

                    if source.is_some_and(|e| clusters.contains(e)) {
                        // Give a fresh proxy its own entity if the split cluster had one.
                        let particles = rigidbody_set
                            .soft_bodies
                            .get(split.soft_body)
                            .and_then(|sb| sb.cluster(split.cluster))
                            .map(|c| c.particles().to_vec())
                            .unwrap_or_default();
                        let pose = rigidbody_set
                            .bodies
                            .get(split.proxy)
                            .map(|rb| utils::iso_to_transform(rb.position()))
                            .unwrap_or_default();
                        let mut cluster_commands = commands.spawn((
                            SoftBodyCluster {
                                soft_body,
                                particles,
                            },
                            RapierRigidBodyHandle(split.proxy),
                            RapierContextEntityLink(context_entity),
                            pose,
                        ));
                        // The piece inherits the components driving the split cluster.
                        if pinned {
                            cluster_commands.insert(SoftBodyClusterPinned);
                        }
                        if let Some(target) = target {
                            cluster_commands.insert(target);
                        }
                        if let Some(shape_matching) = shape_matching {
                            cluster_commands.insert(*shape_matching);
                        }
                        if let Some(material) = material {
                            cluster_commands.insert(*material);
                        }
                        let cluster_entity = cluster_commands.id();
                        if let Some(rb) = rigidbody_set.bodies.get_mut(split.proxy) {
                            rb.user_data = cluster_entity.to_bits() as u128;
                        }
                        rigidbody_set
                            .entity2body
                            .insert(cluster_entity, split.proxy);
                        proxy = Some(cluster_entity);
                    } else if let (Some(target), Some(piece)) = (target, proxy) {
                        // A piece's whole-body cluster: its entity was cloned with the target of
                        // the torn soft body, which must be shifted too.
                        commands.entity(piece).try_insert(target);
                    }
                }
                cluster_splits.push(SoftBodyClusterSplit {
                    source_cluster: split.source_cluster,
                    soft_body,
                    cluster: split.cluster,
                    proxy,
                    keeps_proxy: split.keeps_proxy,
                });
            }

            let moved_joints = raw
                .moved_joints
                .iter()
                .map(|moved| SoftBodyJointMove {
                    joint: joints.impulse_joint_entity(moved.joint),
                    from: rigidbody_set.rigid_body_entity(moved.from),
                    to: rigidbody_set.rigid_body_entity(moved.to),
                })
                .collect();

            tear_events.write(SoftBodyTearEvent {
                context: context_entity,
                soft_body: torn_entity,
                pieces,
                cluster_splits,
                moved_joints,
                raw,
            });
        }
    }

    for (entity, components) in remapped {
        components.insert(&mut commands.entity(entity));
    }
}

/// The mesh component a soft-body mesh is rendered with.
#[cfg(all(feature = "to-bevy-mesh", feature = "dim3"))]
type SoftBodyMeshComponent = bevy::mesh::Mesh3d;
/// The mesh component a soft-body mesh is rendered with.
#[cfg(all(feature = "to-bevy-mesh", feature = "dim2"))]
type SoftBodyMeshComponent = bevy::mesh::Mesh2d;

/// System responsible for updating the meshes of the soft bodies with a
/// [`SoftBodyMeshSync`](crate::dynamics::SoftBodyMeshSync) component.
#[cfg(feature = "to-bevy-mesh")]
pub fn sync_soft_body_meshes(
    mut commands: Commands,
    meshes: Option<ResMut<Assets<bevy::mesh::Mesh>>>,
    rigid_body_sets: Query<(&RapierRigidBodySet, &RapierContextColliders)>,
    global_transforms: Query<&GlobalTransform>,
    mut soft_bodies: Query<(
        Entity,
        &RapierSoftBodyHandle,
        &RapierContextEntityLink,
        &mut crate::dynamics::SoftBodyMeshSync,
        Option<&Transform>,
        Option<&ChildOf>,
        Option<&SoftBodyMeshComponent>,
    )>,
) {
    use crate::dynamics::{SoftBodyMeshSignature, SoftBodyRenderGeometry};
    use bevy::mesh::{Indices, Mesh};

    let Some(mut meshes) = meshes else {
        return;
    };

    for (entity, handle, link, mut sync, transform, child_of, mesh_component) in
        soft_bodies.iter_mut()
    {
        let Ok((set, colliders)) = rigid_body_sets.get(link.0) else {
            continue;
        };
        let Some(sb) = set.soft_bodies.get(handle.0) else {
            continue;
        };
        let geometry = SoftBodyRenderGeometry::new(sb, &colliders.colliders);
        // The global transform the entity will have after this frame's transform propagation.
        let global = match (transform, child_of) {
            (Some(transform), Some(child_of)) => global_transforms
                .get(child_of.parent())
                .map(|parent| parent.mul_transform(*transform))
                .unwrap_or_else(|_| GlobalTransform::from(*transform)),
            (Some(transform), None) => GlobalTransform::from(*transform),
            (None, _) => GlobalTransform::IDENTITY,
        };
        let world_to_local = global.affine().inverse();
        let signature = SoftBodyMeshSignature {
            handle: handle.0,
            topology_version: sb.topology_version(),
            num_vertices: geometry.vertices.len(),
            num_indices: geometry.indices.len(),
        };

        if sync.built == Some(signature) {
            if let Some(mut mesh) = mesh_component.and_then(|m| meshes.get_mut(&m.0)) {
                mesh.insert_attribute(
                    Mesh::ATTRIBUTE_POSITION,
                    geometry.positions(&world_to_local),
                );
                if geometry.triangles
                    && !geometry.indices.is_empty()
                    && matches!(mesh.indices(), Some(Indices::U32(_)))
                {
                    mesh.compute_smooth_normals();
                }
                continue;
            }
        }

        // First sync or topology change: (re)build the whole mesh.
        let mesh = geometry.to_mesh(&world_to_local);
        match mesh_component {
            Some(component) if meshes.contains(&component.0) => {
                let _ = meshes.insert(&component.0, mesh);
            }
            _ => {
                let mesh_handle = meshes.add(mesh);
                commands
                    .entity(entity)
                    .insert(SoftBodyMeshComponent::from(mesh_handle));
            }
        }
        sync.built = Some(signature);
    }
}
