use super::{CollisionEvent, Cuboid};
use crate::dynamics::{
    CoefficientCombineRule, ImpulseJointSet, MultibodyJointSet, RigidBodyDominance,
    RigidBodyHandle, RigidBodySet, RigidBodyType,
};
use crate::geometry::{ColliderChanges, ColliderHandle, ColliderSet, Contact, ContactManifold};
use crate::math::{Point, Real, TangentImpulse, Vector};
use crate::pipeline::{
    ActiveEvents, ActiveHooks, ContactModificationContext, EventHandler, PairFilterContext,
    PhysicsHooks,
};
use crate::prelude::CollisionEventFlags;
use parry::bounding_volume::BoundingVolume;
use parry::query::{ContactManifoldsWorkspace, PersistentQueryDispatcher};
use parry::utils::IsometryOpt;
use std::any::Any;

#[cfg(doc)]
use super::Collider;

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct SolverFlags: u32 {
        /// The constraint solver will take this contact manifold into
        /// account for force computation.
        const COMPUTE_IMPULSES = 0b001;
    }
}

impl Default for SolverFlags {
    fn default() -> Self {
        SolverFlags::COMPUTE_IMPULSES
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A single contact between two collider.
pub struct ContactData {
    /// The impulse, along the contact normal, applied by this contact to the first collider's rigid-body.
    ///
    /// The impulse applied to the second collider's rigid-body is given by `-impulse`.
    pub impulse: Real,
    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    pub tangent_impulse: TangentImpulse<Real>,
    /// The impulse retained for warmstarting the next simulation step.
    pub warmstart_impulse: Real,
    /// The friction impulse retained for warmstarting the next simulation step.
    pub warmstart_tangent_impulse: TangentImpulse<Real>,
}

impl Default for ContactData {
    fn default() -> Self {
        Self {
            impulse: 0.0,
            tangent_impulse: na::zero(),
            warmstart_impulse: 0.0,
            warmstart_tangent_impulse: na::zero(),
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
/// The description of all the contacts between a pair of colliders.
pub struct IntersectionPair {
    /// Are the colliders intersecting?
    pub intersecting: bool,
    /// Was a `CollisionEvent::Started` emitted for this collider?
    pub(crate) start_event_emitted: bool,
}

impl IntersectionPair {
    pub(crate) fn new() -> Self {
        Self {
            intersecting: false,
            start_event_emitted: false,
        }
    }

    pub(crate) fn emit_start_event(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        events: &dyn EventHandler,
    ) {
        self.start_event_emitted = true;
        events.handle_collision_event(
            bodies,
            colliders,
            CollisionEvent::Started(collider1, collider2, CollisionEventFlags::SENSOR),
            None,
        );
    }

    pub(crate) fn emit_stop_event(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
        events: &dyn EventHandler,
    ) {
        self.start_event_emitted = false;
        events.handle_collision_event(
            bodies,
            colliders,
            CollisionEvent::Stopped(collider1, collider2, CollisionEventFlags::SENSOR),
            None,
        );
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// The description of all the contacts between a pair of colliders.
pub struct ContactPair {
    /// The first collider involved in the contact pair.
    pub collider1: ColliderHandle,
    /// The second collider involved in the contact pair.
    pub collider2: ColliderHandle,
    /// The set of contact manifolds between the two colliders.
    ///
    /// All contact manifold contain themselves contact points between the colliders.
    /// Note that contact points in the contact manifold do not take into account the
    /// [`Collider::contact_skin`] which only affects the constraint solver and the
    /// [`SolverContact`].
    pub manifolds: Vec<ContactManifold>,
    /// Is there any active contact in this contact pair?
    pub has_any_active_contact: bool,
    /// Was a `CollisionEvent::Started` emitted for this collider?
    pub(crate) start_event_emitted: bool,
    pub(crate) workspace: Option<ContactManifoldsWorkspace>,
}

impl ContactPair {
    pub(crate) fn new(collider1: ColliderHandle, collider2: ColliderHandle) -> Self {
        Self {
            collider1,
            collider2,
            has_any_active_contact: false,
            manifolds: Vec::new(),
            start_event_emitted: false,
            workspace: None,
        }
    }

    /// Clears all the contacts of this contact pair.
    pub fn clear(&mut self) {
        self.manifolds.clear();
        self.has_any_active_contact = false;
        self.workspace = None;
    }

    /// The sum of all the impulses applied by contacts on this contact pair.
    pub fn total_impulse(&self) -> Vector<Real> {
        self.manifolds
            .iter()
            .map(|m| m.total_impulse() * m.data.normal)
            .sum()
    }

    /// The sum of the magnitudes of the contacts on this contact pair.
    pub fn total_impulse_magnitude(&self) -> Real {
        self.manifolds
            .iter()
            .fold(0.0, |a, m| a + m.total_impulse())
    }

    /// The magnitude and (unit) direction of the maximum impulse on this contact pair.
    pub fn max_impulse(&self) -> (Real, Vector<Real>) {
        let mut result = (0.0, Vector::zeros());

        for m in &self.manifolds {
            let impulse = m.total_impulse();

            if impulse > result.0 {
                result = (impulse, m.data.normal);
            }
        }

        result
    }

    /// Finds the contact with the smallest signed distance.
    ///
    /// If the colliders involved in this contact pair are penetrating, then
    /// this returns the contact with the largest penetration depth.
    ///
    /// Returns a reference to the contact, as well as the contact manifold
    /// it is part of.
    #[profiling::function]
    pub fn find_deepest_contact(&self) -> Option<(&ContactManifold, &Contact)> {
        let mut deepest = None;

        for m2 in &self.manifolds {
            let deepest_candidate = m2.find_deepest_contact();

            deepest = match (deepest, deepest_candidate) {
                (_, None) => deepest,
                (None, Some(c2)) => Some((m2, c2)),
                (Some((m1, c1)), Some(c2)) => {
                    if c1.dist <= c2.dist {
                        Some((m1, c1))
                    } else {
                        Some((m2, c2))
                    }
                }
            }
        }

        deepest
    }

    pub(crate) fn emit_start_event(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) {
        self.start_event_emitted = true;

        events.handle_collision_event(
            bodies,
            colliders,
            CollisionEvent::Started(self.collider1, self.collider2, CollisionEventFlags::empty()),
            Some(self),
        );
    }

    pub(crate) fn emit_stop_event(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) {
        self.start_event_emitted = false;

        events.handle_collision_event(
            bodies,
            colliders,
            CollisionEvent::Stopped(self.collider1, self.collider2, CollisionEventFlags::empty()),
            Some(self),
        );
    }

    pub(crate) fn update(
        &mut self,
        prediction_distance: Real,
        dt: Real,
        query_dispatcher: &dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        // TODO PERF: a **lot** of what’s happening in here are done at every frame
        //            despite that fact that they are checks depending of objects properties
        //            that are very rarely changed by the user. These tests should be done only
        //            once and way earlier so we can even not include them in the graph if the
        //            pair doesn’t work out.
        //            Then we have the user-changes detection update the pair if something changed
        //            that warrants a re-evaluation of these checks.
        let had_any_active_contact = self.has_any_active_contact;
        let co1 = &colliders[self.collider1];
        let co2 = &colliders[self.collider2];

        'emit_events: {
            if !co1.changes.needs_narrow_phase_update() && !co2.changes.needs_narrow_phase_update()
            {
                // No update needed for these colliders.
                return;
            }
            if co1.parent.map(|p| p.handle) == co2.parent.map(|p| p.handle) && co1.parent.is_some()
            {
                // Same parents. Ignore collisions.
                self.clear();
                break 'emit_events;
            }

            let rb1 = co1.parent.map(|co_parent1| &bodies[co_parent1.handle]);
            let rb2 = co2.parent.map(|co_parent2| &bodies[co_parent2.handle]);

            let rb_type1 = rb1.map(|rb| rb.body_type).unwrap_or(RigidBodyType::Fixed);
            let rb_type2 = rb2.map(|rb| rb.body_type).unwrap_or(RigidBodyType::Fixed);

            // Deal with contacts disabled between bodies attached by joints.
            if let (Some(co_parent1), Some(co_parent2)) = (&co1.parent, &co2.parent) {
                for (_, joint) in
                    impulse_joints.joints_between(co_parent1.handle, co_parent2.handle)
                {
                    if !joint.data.contacts_enabled {
                        self.clear();
                        break 'emit_events;
                    }
                }

                let link1 = multibody_joints.rigid_body_link(co_parent1.handle);
                let link2 = multibody_joints.rigid_body_link(co_parent2.handle);

                if let (Some(link1), Some(link2)) = (link1, link2) {
                    // If both bodies belong to the same multibody, apply some additional built-in
                    // contact filtering rules.
                    if link1.multibody == link2.multibody {
                        // 1) check if self-contacts is enabled.
                        if let Some(mb) = multibody_joints.get_multibody(link1.multibody) {
                            if !mb.self_contacts_enabled() {
                                self.clear();
                                break 'emit_events;
                            }
                        }

                        // 2) if they are attached by a joint, check if  contacts is disabled.
                        if let Some((_, _, mb_link)) =
                            multibody_joints.joint_between(co_parent1.handle, co_parent2.handle)
                        {
                            if !mb_link.joint.data.contacts_enabled {
                                self.clear();
                                break 'emit_events;
                            }
                        }
                    }
                }
            }

            // Filter based on the rigid-body types.
            if !co1.flags.active_collision_types.test(rb_type1, rb_type2)
                && !co2.flags.active_collision_types.test(rb_type1, rb_type2)
            {
                self.clear();
                break 'emit_events;
            }

            // Filter based on collision groups.
            if !co1.flags.collision_groups.test(co2.flags.collision_groups) {
                self.clear();
                break 'emit_events;
            }

            let active_hooks = co1.flags.active_hooks | co2.flags.active_hooks;

            let mut solver_flags = if active_hooks.contains(ActiveHooks::FILTER_CONTACT_PAIRS) {
                let context = PairFilterContext {
                    bodies,
                    colliders,
                    rigid_body1: co1.parent.map(|p| p.handle),
                    rigid_body2: co2.parent.map(|p| p.handle),
                    collider1: self.collider1,
                    collider2: self.collider2,
                };

                if let Some(solver_flags) = hooks.filter_contact_pair(&context) {
                    solver_flags
                } else {
                    // No contact allowed.
                    self.clear();
                    break 'emit_events;
                }
            } else {
                SolverFlags::default()
            };

            if !co1.flags.solver_groups.test(co2.flags.solver_groups) {
                solver_flags.remove(SolverFlags::COMPUTE_IMPULSES);
            }

            if co1.changes.contains(ColliderChanges::SHAPE)
                || co2.changes.contains(ColliderChanges::SHAPE)
            {
                // The shape changed so the workspace is no longer valid.
                self.workspace = None;
            }

            let pos12 = co1.pos.inv_mul(&co2.pos);

            let contact_skin_sum = co1.contact_skin() + co2.contact_skin();
            let soft_ccd_prediction1 = rb1.map(|rb| rb.soft_ccd_prediction()).unwrap_or(0.0);
            let soft_ccd_prediction2 = rb2.map(|rb| rb.soft_ccd_prediction()).unwrap_or(0.0);
            let effective_prediction_distance =
                if soft_ccd_prediction1 > 0.0 || soft_ccd_prediction2 > 0.0 {
                    let aabb1 = co1.compute_collision_aabb(0.0);
                    let aabb2 = co2.compute_collision_aabb(0.0);
                    let inv_dt = crate::utils::inv(dt);

                    let linvel1 = rb1
                        .map(|rb| rb.linvel().cap_magnitude(soft_ccd_prediction1 * inv_dt))
                        .unwrap_or_default();
                    let linvel2 = rb2
                        .map(|rb| rb.linvel().cap_magnitude(soft_ccd_prediction2 * inv_dt))
                        .unwrap_or_default();

                    if !aabb1.intersects(&aabb2)
                        && !aabb1.intersects_moving_aabb(&aabb2, linvel2 - linvel1)
                    {
                        self.clear();
                        break 'emit_events;
                    }

                    prediction_distance.max(dt * (linvel1 - linvel2).norm()) + contact_skin_sum
                } else {
                    prediction_distance + contact_skin_sum
                };

            if self.manifolds.is_empty() {
                self.manifolds.push(ContactManifold::new());
            }

            // use unsafe_any::UnsafeAny;
            // let cuboid1: &Cuboid =
            //     unsafe { (&*co1.shape.0 as &dyn UnsafeAny).downcast_ref_unchecked() };
            // let cuboid2: &Cuboid =
            //     unsafe { (&*co2.shape.0 as &dyn UnsafeAny).downcast_ref_unchecked() };

            // NOTE: this experiments shows that 10% of the time is spent on dispatch.
            parry::query::details::contact_manifold_cuboid_cuboid_shapes(
                &pos12,
                &*co1.shape.0,
                &*co2.shape.0,
                effective_prediction_distance,
                &mut self.manifolds[0],
            );
            // let _ = query_dispatcher.contact_manifolds(
            //     &pos12,
            //     &*co1.shape,
            //     &*co2.shape,
            //     effective_prediction_distance,
            //     &mut self.manifolds,
            //     &mut self.workspace,
            // );

            let friction = CoefficientCombineRule::combine(
                co1.material.friction,
                co2.material.friction,
                co1.material.friction_combine_rule,
                co2.material.friction_combine_rule,
            );
            let restitution = CoefficientCombineRule::combine(
                co1.material.restitution,
                co2.material.restitution,
                co1.material.restitution_combine_rule,
                co2.material.restitution_combine_rule,
            );

            let zero = RigidBodyDominance(0); // The value doesn't matter, it will be MAX because of the effective groups.
            let dominance1 = rb1.map(|rb| rb.dominance).unwrap_or(zero);
            let dominance2 = rb2.map(|rb| rb.dominance).unwrap_or(zero);

            self.has_any_active_contact = false;

            for manifold in &mut self.manifolds {
                let world_pos1 = manifold.subshape_pos1.prepend_to(&co1.pos);
                let world_pos2 = manifold.subshape_pos2.prepend_to(&co2.pos);
                manifold.data.solver_contacts.clear();
                manifold.data.rigid_body1 = co1.parent.map(|p| p.handle);
                manifold.data.rigid_body2 = co2.parent.map(|p| p.handle);
                manifold.data.solver_flags = solver_flags;
                manifold.data.relative_dominance =
                    dominance1.effective_group(&rb_type1) - dominance2.effective_group(&rb_type2);
                manifold.data.normal = world_pos1 * manifold.local_n1;

                // Generate solver contacts.
                for (contact_id, contact) in manifold.points.iter().enumerate() {
                    if contact_id > u8::MAX as usize {
                        log::warn!(
                            "A contact manifold cannot contain more than 255 contacts currently, dropping contact in excess."
                        );
                        break;
                    }

                    let effective_contact_dist =
                        contact.dist - co1.contact_skin() - co2.contact_skin();

                    let keep_solver_contact = effective_contact_dist < prediction_distance || {
                        let world_pt1 = world_pos1 * contact.local_p1;
                        let world_pt2 = world_pos2 * contact.local_p2;
                        let vel1 = rb1
                            .map(|rb| rb.velocity_at_point(&world_pt1))
                            .unwrap_or_default();
                        let vel2 = rb2
                            .map(|rb| rb.velocity_at_point(&world_pt2))
                            .unwrap_or_default();
                        effective_contact_dist + (vel2 - vel1).dot(&manifold.data.normal) * dt
                            < prediction_distance
                    };

                    if keep_solver_contact {
                        // Generate the solver contact.
                        let world_pt1 = world_pos1 * contact.local_p1;
                        let world_pt2 = world_pos2 * contact.local_p2;
                        let effective_point = na::center(&world_pt1, &world_pt2);

                        let solver_contact = SolverContact {
                            contact_id: contact_id as u8,
                            point: effective_point,
                            dist: effective_contact_dist,
                            friction,
                            restitution,
                            tangent_velocity: Vector::zeros(),
                            is_new: contact.data.impulse == 0.0,
                            warmstart_impulse: contact.data.warmstart_impulse,
                            warmstart_tangent_impulse: contact.data.warmstart_tangent_impulse,
                        };

                        manifold.data.solver_contacts.push(solver_contact);
                        self.has_any_active_contact = true;
                    }
                }

                // Apply the user-defined contact modification.
                if active_hooks.contains(ActiveHooks::MODIFY_SOLVER_CONTACTS) {
                    let mut modifiable_solver_contacts =
                        std::mem::take(&mut manifold.data.solver_contacts);
                    let mut modifiable_user_data = manifold.data.user_data;
                    let mut modifiable_normal = manifold.data.normal;

                    let mut context = ContactModificationContext {
                        bodies,
                        colliders,
                        rigid_body1: co1.parent.map(|p| p.handle),
                        rigid_body2: co2.parent.map(|p| p.handle),
                        collider1: self.collider1,
                        collider2: self.collider2,
                        manifold,
                        solver_contacts: &mut modifiable_solver_contacts,
                        normal: &mut modifiable_normal,
                        user_data: &mut modifiable_user_data,
                    };

                    hooks.modify_solver_contacts(&mut context);

                    manifold.data.solver_contacts = modifiable_solver_contacts;
                    manifold.data.normal = modifiable_normal;
                    manifold.data.user_data = modifiable_user_data;
                }

                /*
                 * TODO: When using the block solver in 3D, I’d expect this sort to help, but
                 *       it makes the domino demo worse. Needs more investigation.
                fn sort_solver_contacts(mut contacts: &mut [SolverContact]) {
                    while contacts.len() > 2 {
                        let first = contacts[0];
                        let mut furthest_id = 1;
                        let mut furthest_dist = na::distance(&first.point, &contacts[1].point);

                        for (candidate_id, candidate) in contacts.iter().enumerate().skip(2) {
                            let candidate_dist = na::distance(&first.point, &candidate.point);

                            if candidate_dist > furthest_dist {
                                furthest_dist = candidate_dist;
                                furthest_id = candidate_id;
                            }
                        }

                        if furthest_id > 1 {
                            contacts.swap(1, furthest_id);
                        }

                        contacts = &mut contacts[2..];
                    }
                }

                sort_solver_contacts(&mut manifold.data.solver_contacts);
                */
            }
        }

        let active_events = co1.flags.active_events | co2.flags.active_events;

        if self.has_any_active_contact != had_any_active_contact
            && active_events.contains(ActiveEvents::COLLISION_EVENTS)
        {
            if self.has_any_active_contact {
                self.emit_start_event(bodies, colliders, events);
            } else {
                self.emit_stop_event(bodies, colliders, events);
            }
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A contact manifold between two colliders.
///
/// A contact manifold describes a set of contacts between two colliders. All the contact
/// part of the same contact manifold share the same contact normal and contact kinematics.
pub struct ContactManifoldData {
    // The following are set by the narrow-phase.
    /// The first rigid-body involved in this contact manifold.
    pub rigid_body1: Option<RigidBodyHandle>,
    /// The second rigid-body involved in this contact manifold.
    pub rigid_body2: Option<RigidBodyHandle>,
    // We put the following fields here to avoids reading the colliders inside of the
    // contact preparation method.
    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub solver_flags: SolverFlags,
    /// The world-space contact normal shared by all the contact in this contact manifold.
    // NOTE: read the comment of `solver_contacts` regarding serialization. It applies
    // to this field as well.
    pub normal: Vector<Real>,
    /// The contacts that will be seen by the constraints solver for computing forces.
    // NOTE: unfortunately, we can't ignore this field when serialize
    // the contact manifold data. The reason is that the solver contacts
    // won't be updated for sleeping bodies. So it means that for one
    // frame, we won't have any solver contacts when waking up an island
    // after a deserialization. Not only does this break post-snapshot
    // determinism, but it will also skip constraint resolution for these
    // contacts during one frame.
    //
    // An alternative would be to skip the serialization of `solver_contacts` and
    // find a way to recompute them right after the deserialization process completes.
    // However, this would be an expensive operation. And doing this efficiently as part
    // of the narrow-phase update or the contact manifold collect will likely lead to tricky
    // bugs too.
    //
    // So right now it is best to just serialize this field and keep it that way until it
    // is proven to be actually problematic in real applications (in terms of snapshot size for example).
    pub solver_contacts: Vec<SolverContact>,
    /// The relative dominance of the bodies involved in this contact manifold.
    pub relative_dominance: i16,
    /// A user-defined piece of data.
    pub user_data: u32,
}

/// A contact seen by the constraints solver for computing forces.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SolverContact {
    /// The index of the manifold contact used to generate this solver contact.
    pub(crate) contact_id: u8,
    /// The contact point in world-space.
    pub point: Point<Real>,
    /// The distance between the two original contacts points along the contact normal.
    /// If negative, this is measures the penetration depth.
    pub dist: Real,
    /// The effective friction coefficient at this contact point.
    pub friction: Real,
    /// The effective restitution coefficient at this contact point.
    pub restitution: Real,
    /// The desired tangent relative velocity at the contact point.
    ///
    /// This is set to zero by default. Set to a non-zero value to
    /// simulate, e.g., conveyor belts.
    pub tangent_velocity: Vector<Real>,
    /// Whether or not this contact existed during the last timestep.
    pub is_new: bool,
    /// Impulse used to warmstart the solve for the normal constraint.
    pub warmstart_impulse: Real,
    /// Impulse used to warmstart the solve for the friction constraints.
    pub warmstart_tangent_impulse: TangentImpulse<Real>,
}

impl SolverContact {
    /// Should we treat this contact as a bouncy contact?
    /// If `true`, use [`Self::restitution`].
    pub fn is_bouncy(&self) -> bool {
        if self.is_new {
            // Treat new collisions as bouncing at first, unless we have zero restitution.
            self.restitution > 0.0
        } else {
            // If the contact is still here one step later, it is now a resting contact.
            // The exception is very high restitutions, which can never rest
            self.restitution >= 1.0
        }
    }
}

impl Default for ContactManifoldData {
    fn default() -> Self {
        Self::new(None, None, SolverFlags::empty())
    }
}

impl ContactManifoldData {
    pub(crate) fn new(
        rigid_body1: Option<RigidBodyHandle>,
        rigid_body2: Option<RigidBodyHandle>,
        solver_flags: SolverFlags,
    ) -> ContactManifoldData {
        Self {
            rigid_body1,
            rigid_body2,
            solver_flags,
            normal: Vector::zeros(),
            solver_contacts: Vec::new(),
            relative_dominance: 0,
            user_data: 0,
        }
    }

    /// Number of actives contacts, i.e., contacts that will be seen by
    /// the constraints solver.
    #[inline]
    pub fn num_active_contacts(&self) -> usize {
        self.solver_contacts.len()
    }
}

/// Additional methods for the contact manifold.
pub trait ContactManifoldExt {
    /// Computes the sum of all the impulses applied by contacts from this contact manifold.
    fn total_impulse(&self) -> Real;
}

impl ContactManifoldExt for ContactManifold {
    fn total_impulse(&self) -> Real {
        self.points.iter().map(|pt| pt.data.impulse).sum()
    }
}
