use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderHandle, ColliderSet, Contact, ContactManifold};
use crate::math::{Point, Real, TangentImpulse, Vector};
use crate::pipeline::EventHandler;
use crate::prelude::CollisionEventFlags;
use parry::query::ContactManifoldsWorkspace;

use super::CollisionEvent;

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

    /// The index of the manifold contact used to generate this solver contact.
    pub fn contact_id(&self) -> u8 {
        self.contact_id
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
