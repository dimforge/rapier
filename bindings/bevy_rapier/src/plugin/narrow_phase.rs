use crate::math::{Real, Vect};
use crate::plugin::context::{RapierContextColliders, RapierContextSimulation, RapierRigidBodySet};
use crate::utils::iso_to_transform;
use bevy::prelude::*;
use rapier::geometry::{
    ColliderHandle, Contact, ContactManifold, ContactManifoldData, ContactPair, InteractionGraph,
    IntersectionPair, PairContacts, SoftPairContacts, SolverContact, SolverFlags, NEW_CONTACT_BIT,
};

impl RapierContextSimulation {
    /// All the contact pairs involving the non-sensor collider attached to the given entity.
    ///
    /// The returned contact pairs identify pairs of colliders with intersecting bounding-volumes.
    /// To check if any geometric contact happened between the collider shapes, check
    /// [`ContactPairView::has_any_active_contact`].
    pub fn contact_pairs_with<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
        collider: Entity,
    ) -> impl Iterator<Item = ContactPairView<'a>> {
        context_colliders
            .entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .contact_pairs_with(*h)
                    .map(|raw| ContactPairView {
                        context_colliders,
                        rigidbody_set,
                        raw,
                    })
            })
    }

    /// All the intersection pairs involving the collider attached to the given entity, where at least one collider
    /// involved in the intersection is a sensor.
    ///
    /// The returned contact pairs identify pairs of colliders (where at least one is a sensor) with
    /// intersecting bounding-volumes. To check if any geometric overlap happened between the collider shapes, check
    /// the returned boolean.
    pub fn intersection_pairs_with<'a, 'b: 'a>(
        &'a self,
        rapier_colliders: &'b RapierContextColliders,
        collider: Entity,
    ) -> impl Iterator<Item = (Entity, Entity, bool)> + 'a {
        rapier_colliders
            .entity2collider
            .get(&collider)
            .into_iter()
            .flat_map(|h| {
                self.narrow_phase
                    .intersection_pairs_with(*h)
                    .filter_map(|(h1, h2, inter)| {
                        let e1 = rapier_colliders.collider_entity(h1);
                        let e2 = rapier_colliders.collider_entity(h2);
                        match (e1, e2) {
                            (Some(e1), Some(e2)) => Some((e1, e2, inter)),
                            _ => None,
                        }
                    })
            })
    }

    /// The contact pair involving two specific colliders.
    ///
    /// If this returns `None`, there is no contact between the two colliders.
    /// If this returns `Some`, then there may be a contact between the two colliders. Check the
    /// result [`ContactPairView::has_any_active_contact`] method to see if there is an actual contact.
    pub fn contact_pair<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
        collider1: Entity,
        collider2: Entity,
    ) -> Option<ContactPairView<'a>> {
        let h1 = context_colliders.entity2collider.get(&collider1)?;
        let h2 = context_colliders.entity2collider.get(&collider2)?;
        self.narrow_phase
            .contact_pair(*h1, *h2)
            .map(|raw| ContactPairView {
                context_colliders,
                rigidbody_set,
                raw,
            })
    }

    /// The intersection pair involving two specific colliders (at least one being a sensor).
    ///
    /// If this returns `None` or `Some(false)`, then there is no intersection between the two colliders.
    /// If this returns `Some(true)`, then there may be an intersection between the two colliders.
    pub fn intersection_pair(
        &self,
        rapier_colliders: &RapierContextColliders,
        collider1: Entity,
        collider2: Entity,
    ) -> Option<bool> {
        let h1 = rapier_colliders.entity2collider.get(&collider1)?;
        let h2 = rapier_colliders.entity2collider.get(&collider2)?;
        self.narrow_phase.intersection_pair(*h1, *h2)
    }

    /// All the contact pairs detected during the last timestep.
    pub fn contact_pairs<'a, 'b: 'a>(
        &'a self,
        context_colliders: &'b RapierContextColliders,
        rigidbody_set: &'b RapierRigidBodySet,
    ) -> impl Iterator<Item = ContactPairView<'a>> {
        self.narrow_phase
            .contact_pairs()
            .map(|raw| ContactPairView {
                context_colliders,
                rigidbody_set,
                raw,
            })
    }

    /// The raw contact graph of the narrow-phase.
    ///
    /// Its nodes are the Rapier handles of the colliders, and its edges the raw contact pairs
    /// between non-sensor colliders with intersecting bounding volumes. Prefer
    /// [`Self::contact_pairs`] and [`Self::contact_pairs_with`] which yield Bevy entities.
    pub fn contact_graph(&self) -> &InteractionGraph<ColliderHandle, ContactPair> {
        self.narrow_phase.contact_graph()
    }

    /// The raw intersection graph of the narrow-phase.
    ///
    /// Its nodes are the Rapier handles of the colliders, and its edges the raw intersection
    /// pairs involving at least one sensor. Prefer [`Self::intersection_pairs`] and
    /// [`Self::intersection_pairs_with`] which yield Bevy entities.
    pub fn intersection_graph(&self) -> &InteractionGraph<ColliderHandle, IntersectionPair> {
        self.narrow_phase.intersection_graph()
    }

    /// All the intersection pairs detected during the last timestep.
    pub fn intersection_pairs<'a, 'b: 'a>(
        &'a self,
        rapier_colliders: &'b RapierContextColliders,
    ) -> impl Iterator<Item = (Entity, Entity, bool)> + 'a {
        self.narrow_phase
            .intersection_pairs()
            .filter_map(|(h1, h2, inter)| {
                let e1 = rapier_colliders.collider_entity(h1);
                let e2 = rapier_colliders.collider_entity(h2);
                match (e1, e2) {
                    (Some(e1), Some(e2)) => Some((e1, e2, inter)),
                    _ => None,
                }
            })
    }
}

/// Read-only access to the properties of a contact manifold.
pub struct ContactManifoldView<'a> {
    rigidbody_set: &'a RapierRigidBodySet,
    /// The raw contact manifold from Rapier.
    pub raw: &'a ContactManifold,
}

impl ContactManifoldView<'_> {
    /// The number of points on this contact manifold.
    pub fn num_points(&self) -> usize {
        self.raw.points.len()
    }

    /// Retrieves the i-th point of this contact manifold.
    pub fn point(&self, i: usize) -> Option<ContactView<'_>> {
        self.raw.points.get(i).map(|raw| ContactView { raw })
    }

    /// The contacts points.
    pub fn points(&self) -> impl ExactSizeIterator<Item = ContactView<'_>> {
        self.raw.points.iter().map(|raw| ContactView { raw })
    }

    /// The contact normal of all the contacts of this manifold, expressed in the local space of the first shape.
    pub fn local_n1(&self) -> Vect {
        self.raw.local_n1
    }

    /// The contact normal of all the contacts of this manifold, expressed in the local space of the second shape.
    pub fn local_n2(&self) -> Vect {
        self.raw.local_n2
    }

    /// The first subshape involved in this contact manifold.
    ///
    /// This is zero if the first shape is not a composite shape.
    pub fn subshape1(&self) -> u32 {
        self.raw.subshape1
    }

    /// The second subshape involved in this contact manifold.
    ///
    /// This is zero if the second shape is not a composite shape.
    pub fn subshape2(&self) -> u32 {
        self.raw.subshape2
    }

    /// The first rigid-body involved in this contact manifold.
    pub fn rigid_body1(&self) -> Option<Entity> {
        self.raw
            .data
            .rigid_body1
            .and_then(|h| self.rigidbody_set.rigid_body_entity(h))
    }

    /// The second rigid-body involved in this contact manifold.
    pub fn rigid_body2(&self) -> Option<Entity> {
        self.raw
            .data
            .rigid_body2
            .and_then(|h| self.rigidbody_set.rigid_body_entity(h))
    }

    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub fn solver_flags(&self) -> SolverFlags {
        self.raw.data.solver_flags
    }

    /// The world-space contact normal shared by all the contact in this contact manifold.
    pub fn normal(&self) -> Vect {
        self.raw.data.normal
    }

    /// The contacts that will be seen by the constraints solver for computing forces.
    pub fn num_solver_contacts(&self) -> usize {
        self.raw.data.solver_contacts.len()
    }

    /// Gets the i-th solver contact.
    pub fn solver_contact(&self, i: usize) -> Option<SolverContactView<'_>> {
        self.raw
            .data
            .solver_contacts
            .get(i)
            .map(|raw| SolverContactView {
                rigidbody_set: self.rigidbody_set,
                manifold_data: &self.raw.data,
                raw,
            })
    }

    /// The contacts that will be seen by the constraints solver for computing forces.
    pub fn solver_contacts(&self) -> impl ExactSizeIterator<Item = SolverContactView<'_>> {
        self.raw
            .data
            .solver_contacts
            .iter()
            .map(|raw| SolverContactView {
                rigidbody_set: self.rigidbody_set,
                manifold_data: &self.raw.data,
                raw,
            })
    }

    /// The relative dominance of the bodies involved in this contact manifold.
    pub fn relative_dominance(&self) -> i16 {
        self.raw.data.relative_dominance
    }

    /// A user-defined piece of data.
    pub fn user_data(&self) -> u32 {
        self.raw.data.user_data
    }

    /// The effective friction coefficient of this manifold's contacts, combined from both
    /// colliders' materials (or as set by a contact modification hook).
    pub fn friction(&self) -> Real {
        self.raw.data.friction
    }

    /// The effective restitution coefficient of this manifold's contacts, combined from both
    /// colliders' materials (or as set by a contact modification hook).
    pub fn restitution(&self) -> Real {
        self.raw.data.restitution
    }

    /// The pose of the first shape's subshape involved in this manifold, relative to the first
    /// collider, if the first shape is a composite shape.
    pub fn subshape_pose1(&self) -> Option<Transform> {
        self.raw.subshape_pos1().map(iso_to_transform)
    }

    /// The pose of the second shape's subshape involved in this manifold, relative to the second
    /// collider, if the second shape is a composite shape.
    pub fn subshape_pose2(&self) -> Option<Transform> {
        self.raw.subshape_pos2().map(iso_to_transform)
    }
}

impl ContactManifoldView<'_> {
    /// Returns the contact with the smallest distance (i.e. the largest penetration depth).
    pub fn find_deepest_contact(&self) -> Option<ContactView<'_>> {
        self.raw
            .find_deepest_contact()
            .map(|raw| ContactView { raw })
    }
}

/// Read-only access to the properties of a single contact.
pub struct ContactView<'a> {
    /// The raw contact from Rapier.
    pub raw: &'a Contact,
}

impl ContactView<'_> {
    /// The contact point in the local-space of the first shape.
    pub fn local_p1(&self) -> Vect {
        self.raw.local_p1
    }

    /// The contact point in the local-space of the second shape.
    pub fn local_p2(&self) -> Vect {
        self.raw.local_p2
    }

    /// The distance between the two contact points.
    pub fn dist(&self) -> Real {
        self.raw.dist
    }

    /// The feature ID of the first shape involved in the contact.
    pub fn fid1(&self) -> u32 {
        self.raw.fid1.0
    }

    /// The feature ID of the second shape involved in the contact.
    pub fn fid2(&self) -> u32 {
        self.raw.fid2.0
    }

    /// The impulse, along the contact normal, applied by this contact to the first collider's rigid-body.
    ///
    /// The impulse applied to the second collider's rigid-body is given by `-impulse`.
    pub fn impulse(&self) -> Real {
        self.raw.data.impulse
    }

    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim2")]
    pub fn tangent_impulse(&self) -> Real {
        self.raw.data.tangent_impulse.x
    }

    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim3")]
    pub fn tangent_impulse(&self) -> [Real; 2] {
        self.raw.data.tangent_impulse.into()
    }

    /// The normal impulse retained for warmstarting the next simulation step.
    pub fn warmstart_impulse(&self) -> Real {
        self.raw.data.warmstart_impulse
    }

    /// The friction impulse retained for warmstarting the next simulation step.
    #[cfg(feature = "dim2")]
    pub fn warmstart_tangent_impulse(&self) -> Real {
        self.raw.data.warmstart_tangent_impulse.x
    }

    /// The friction impulse retained for warmstarting the next simulation step.
    #[cfg(feature = "dim3")]
    pub fn warmstart_tangent_impulse(&self) -> [Real; 2] {
        self.raw.data.warmstart_tangent_impulse.into()
    }

    /// The twist (rotational friction) impulse retained for warmstarting the next simulation step.
    #[cfg(feature = "dim3")]
    pub fn warmstart_twist_impulse(&self) -> Real {
        self.raw.data.warmstart_twist_impulse
    }
}

/// Read-only access to the properties of a single solver contact.
pub struct SolverContactView<'a> {
    rigidbody_set: &'a RapierRigidBodySet,
    /// The data of the contact manifold this solver contact is part of.
    pub manifold_data: &'a ContactManifoldData,
    /// The raw solver contact from Rapier.
    pub raw: &'a SolverContact,
}

impl SolverContactView<'_> {
    /// The world-space contact point.
    pub fn point(&self) -> Vect {
        let (p1, p2) = self
            .manifold_data
            .solver_contact_world_points(self.raw, &self.rigidbody_set.bodies);
        (p1 + p2) / 2.0
    }
    /// The distance between the two original contacts points along the contact normal.
    /// If negative, this is measures the penetration depth.
    pub fn dist(&self) -> Real {
        self.raw.dist
    }
    /// The effective friction coefficient at this contact point.
    pub fn friction(&self) -> Real {
        self.manifold_data.friction
    }
    /// The effective restitution coefficient at this contact point.
    pub fn restitution(&self) -> Real {
        self.manifold_data.restitution
    }
    /// The desired tangent relative velocity at the contact point.
    ///
    /// This is set to zero by default. Set to a non-zero value to
    /// simulate, e.g., conveyor belts.
    pub fn tangent_velocity(&self) -> Vect {
        self.raw.tangent_velocity
    }
    /// Whether or not this contact existed during the last timestep.
    pub fn is_new(&self) -> bool {
        self.raw.contact_id[0] & NEW_CONTACT_BIT != 0
    }
    /// The index, in its manifold's contact points, of the contact this solver contact was
    /// generated from.
    ///
    /// This index is only valid for the timestep that produced this solver contact.
    pub fn contact_id(&self) -> u32 {
        self.raw.contact_indices()[0]
    }
    /// The raw contact point on the first body's surface, as stored for the solver.
    ///
    /// This is expressed in the center-of-mass-centered local frame of the first rigid-body, or in
    /// world-space if that side is attached to the world (no rigid-body, or a fixed one). Use
    /// [`Self::world_point1`] for a world-space point.
    pub fn anchor1(&self) -> Vect {
        self.raw.anchor1
    }
    /// The raw contact point on the second body's surface, as stored for the solver.
    ///
    /// See [`Self::anchor1`] for the frame it is expressed in.
    pub fn anchor2(&self) -> Vect {
        self.raw.anchor2
    }
    /// The world-space contact point on the first body's surface.
    pub fn world_point1(&self) -> Vect {
        self.manifold_data
            .solver_contact_world_points(self.raw, &self.rigidbody_set.bodies)
            .0
    }
    /// The world-space contact point on the second body's surface.
    pub fn world_point2(&self) -> Vect {
        self.manifold_data
            .solver_contact_world_points(self.raw, &self.rigidbody_set.bodies)
            .1
    }
}

/// Read-only access to the properties of a contact pair.
pub struct ContactPairView<'a> {
    context_colliders: &'a RapierContextColliders,
    rigidbody_set: &'a RapierRigidBodySet,
    /// The raw contact pair from Rapier.
    pub raw: &'a ContactPair,
}

impl ContactPairView<'_> {
    /// The first collider involved in this contact pair.
    pub fn collider1(&self) -> Option<Entity> {
        self.context_colliders.collider_entity(self.raw.collider1)
    }

    /// The second collider involved in this contact pair.
    pub fn collider2(&self) -> Option<Entity> {
        self.context_colliders.collider_entity(self.raw.collider2)
    }

    /// The number of contact manifolds detected for this contact pair.
    pub fn manifolds_len(&self) -> usize {
        self.raw.manifolds().len()
    }

    /// Gets the i-th contact manifold.
    pub fn manifold(&self, i: usize) -> Option<ContactManifoldView<'_>> {
        self.raw.manifolds().get(i).map(|raw| ContactManifoldView {
            rigidbody_set: self.rigidbody_set,
            raw,
        })
    }

    /// Iterate through all the contact manifolds of this contact pair.
    pub fn manifolds(&self) -> impl ExactSizeIterator<Item = ContactManifoldView<'_>> {
        self.raw.manifolds().iter().map(|raw| ContactManifoldView {
            rigidbody_set: self.rigidbody_set,
            raw,
        })
    }

    /// Is there any active contact in this contact pair?
    pub fn has_any_active_contact(&self) -> bool {
        self.raw.has_any_active_contact()
    }

    /// Does this pair hold contact manifolds (every pair except two soft surfaces)?
    pub fn is_rigid(&self) -> bool {
        self.raw.rigid().is_some()
    }

    /// Is this a pair of two soft surfaces, whose contacts are candidates instead of manifolds?
    ///
    /// Such pairs have no manifolds; their candidates can be read through
    /// [`Self::soft_contacts`].
    pub fn is_soft(&self) -> bool {
        self.raw.soft().is_some()
    }

    /// The contacts of this pair: manifolds, or the contact candidates of two soft surfaces.
    pub fn contacts(&self) -> &PairContacts {
        &self.raw.contacts
    }

    /// The contact candidates of a pair of two soft surfaces (`None` for other pairs).
    pub fn soft_contacts(&self) -> Option<&SoftPairContacts> {
        self.raw.soft()
    }

    /// Whether the two soft surfaces of this pair touch: some feature of one surface is within
    /// contact reach of the other, or the two surfaces cross (`None` for other pairs).
    pub fn soft_touching(&self) -> Option<bool> {
        match &self.raw.contacts {
            PairContacts::Soft { touching, .. } => Some(*touching),
            PairContacts::Rigid(_) => None,
        }
    }

    /// The number of manifolds seen by the constraints solver, see [`Self::solver_manifolds`].
    pub fn solver_manifolds_len(&self) -> usize {
        self.raw.solver_manifolds().len()
    }

    /// Iterate through the contact manifolds actually seen by the constraints solver.
    ///
    /// These are the contact clusters if contact clustering applied to this pair (merging the
    /// manifolds sharing nearly the same normal), and the plain [`Self::manifolds`] otherwise.
    /// The contact impulses applied by the solver are stored in these manifolds.
    pub fn solver_manifolds(&self) -> impl ExactSizeIterator<Item = ContactManifoldView<'_>> {
        self.raw
            .solver_manifolds()
            .iter()
            .map(|raw| ContactManifoldView {
                rigidbody_set: self.rigidbody_set,
                raw,
            })
    }

    /// The sum of all the contact impulses applied by the solver to the first collider during
    /// the last timestep.
    ///
    /// Divide by the timestep length to obtain a force.
    pub fn total_impulse(&self) -> Vect {
        self.raw.total_impulse()
    }

    /// The sum of the magnitudes of all the contact impulses applied during the last timestep.
    ///
    /// This is not the magnitude of [`Self::total_impulse`]; it is the quantity compared against
    /// the contact force event threshold (after conversion to a force).
    pub fn total_impulse_magnitude(&self) -> Real {
        self.raw.total_impulse_magnitude()
    }

    /// The magnitude and world-space direction of the strongest contact impulse applied during
    /// the last timestep.
    pub fn max_impulse(&self) -> (Real, Vect) {
        self.raw.max_impulse()
    }

    /// Finds the contact with the smallest signed distance.
    ///
    /// If the colliders involved in this contact pair are penetrating, then
    /// this returns the contact with the largest penetration depth.
    ///
    /// Returns a reference to the contact, as well as the contact manifold
    /// it is part of.
    pub fn find_deepest_contact(&self) -> Option<(ContactManifoldView<'_>, ContactView<'_>)> {
        self.raw.find_deepest_contact().map(|(manifold, contact)| {
            (
                ContactManifoldView {
                    rigidbody_set: self.rigidbody_set,
                    raw: manifold,
                },
                ContactView { raw: contact },
            )
        })
    }
}

#[cfg(test)]
mod test {
    use crate::math::Real;
    use crate::prelude::*;
    use bevy::{
        prelude::*,
        time::{TimePlugin, TimeUpdateStrategy},
    };

    #[cfg(feature = "dim3")]
    fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
        Collider::cuboid(hx, hy, hz)
    }
    #[cfg(feature = "dim2")]
    fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
        Collider::cuboid(hx, hy)
    }

    #[test]
    fn resting_box_contact_impulses() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));
        let ground = app
            .world_mut()
            .spawn((Transform::from_xyz(0.0, -1.0, 0.0), cuboid(4.0, 1.0, 4.0)))
            .id();
        let body = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 0.6, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                // Keep the box awake so the impulses are refreshed at every step.
                Sleeping::disabled(),
            ))
            .id();
        app.finish();
        for _ in 0..60 {
            app.update();
        }

        let mut query = app.world_mut().query::<(
            &RapierContextSimulation,
            &RapierContextColliders,
            &RapierRigidBodySet,
        )>();
        let (simulation, colliders, bodies) = query.single(app.world()).unwrap();
        let pair = simulation
            .contact_pair(colliders, bodies, ground, body)
            .unwrap();
        assert!(pair.is_rigid());
        assert!(!pair.is_soft());
        assert!(pair.has_any_active_contact());
        assert!(pair.solver_manifolds_len() > 0);
        assert!(pair.total_impulse_magnitude() > 0.0);
        assert!(pair.total_impulse().length() > 0.0);
        let (max_impulse, max_dir) = pair.max_impulse();
        assert!(max_impulse > 0.0);
        assert!(max_dir.y.abs() > 0.9);

        let manifold = pair.solver_manifolds().next().unwrap();
        assert!(manifold.friction() > 0.0);
        assert!(manifold.subshape_pose1().is_none());
        assert!(manifold.points().any(|c| c.warmstart_impulse() > 0.0));
        for solver_contact in manifold.solver_contacts() {
            assert!((solver_contact.contact_id() as usize) < manifold.num_points());
            let midpoint = (solver_contact.world_point1() + solver_contact.world_point2()) / 2.0;
            assert!((midpoint - solver_contact.point()).length() < 1.0e-5);
        }

        assert!(simulation.contact_graph().interactions().count() > 0);
        assert_eq!(simulation.intersection_graph().interactions().count(), 0);
    }
}
