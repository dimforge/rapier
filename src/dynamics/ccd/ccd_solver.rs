use super::TOIEntry;
use crate::data::{BundleSet, ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{IslandManager, RigidBodyColliders, RigidBodyForces};
use crate::dynamics::{
    RigidBodyCcd, RigidBodyHandle, RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::geometry::{
    ColliderParent, ColliderPosition, ColliderShape, ColliderType, IntersectionEvent, NarrowPhase,
};
use crate::math::Real;
use crate::parry::utils::SortedPair;
use crate::pipeline::{EventHandler, QueryPipeline, QueryPipelineMode};
use crate::prelude::{ActiveEvents, ColliderFlags};
use parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use parry::utils::hashmap::HashMap;
use std::collections::BinaryHeap;

pub enum PredictedImpacts {
    Impacts(HashMap<RigidBodyHandle, Real>),
    ImpactsAfterEndTime(Real),
    NoImpacts,
}

/// Solver responsible for performing motion-clamping on fast-moving bodies.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct CCDSolver {
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    query_pipeline: QueryPipeline,
}

impl CCDSolver {
    /// Initializes a new CCD solver
    pub fn new() -> Self {
        Self::with_query_dispatcher(DefaultQueryDispatcher)
    }

    /// Initializes a CCD solver with a custom `QueryDispatcher` used for computing time-of-impacts.
    ///
    /// Use this constructor in order to use a custom `QueryDispatcher` that is aware of your own
    /// user-defined shapes.
    pub fn with_query_dispatcher<D>(d: D) -> Self
    where
        D: 'static + QueryDispatcher,
    {
        CCDSolver {
            query_pipeline: QueryPipeline::with_query_dispatcher(d),
        }
    }

    /// Apply motion-clamping to the bodies affected by the given `impacts`.
    ///
    /// The `impacts` should be the result of a previous call to `self.predict_next_impacts`.
    pub fn clamp_motions<Bodies>(&self, dt: Real, bodies: &mut Bodies, impacts: &PredictedImpacts)
    where
        Bodies: ComponentSet<RigidBodyCcd>
            + ComponentSetMut<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>,
    {
        match impacts {
            PredictedImpacts::Impacts(tois) => {
                for (handle, toi) in tois {
                    let (rb_poss, vels, ccd, mprops): (
                        &RigidBodyPosition,
                        &RigidBodyVelocity,
                        &RigidBodyCcd,
                        &RigidBodyMassProps,
                    ) = bodies.index_bundle(handle.0);
                    let local_com = &mprops.local_mprops.local_com;

                    let min_toi = (ccd.ccd_thickness
                        * 0.15
                        * crate::utils::inv(ccd.max_point_velocity(vels)))
                    .min(dt);
                    // println!("Min toi: {}, Toi: {}", min_toi, toi);
                    let new_pos = vels.integrate(toi.max(min_toi), &rb_poss.position, &local_com);
                    bodies.map_mut_internal(handle.0, |rb_poss| {
                        rb_poss.next_position = new_pos;
                    });
                }
            }
            _ => {}
        }
    }

    /// Updates the set of bodies that needs CCD to be resolved.
    ///
    /// Returns `true` if any rigid-body must have CCD resolved.
    pub fn update_ccd_active_flags<Bodies>(
        &self,
        islands: &IslandManager,
        bodies: &mut Bodies,
        dt: Real,
        include_forces: bool,
    ) -> bool
    where
        Bodies: ComponentSetMut<RigidBodyCcd>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyForces>,
    {
        let mut ccd_active = false;

        // println!("Checking CCD activation");
        for handle in islands.active_dynamic_bodies() {
            let (ccd, vels, forces): (&RigidBodyCcd, &RigidBodyVelocity, &RigidBodyForces) =
                bodies.index_bundle(handle.0);

            if ccd.ccd_enabled {
                let forces = if include_forces { Some(forces) } else { None };
                let moving_fast = ccd.is_moving_fast(dt, vels, forces);

                bodies.map_mut_internal(handle.0, |ccd| {
                    ccd.ccd_active = moving_fast;
                });

                ccd_active = ccd_active || moving_fast;
            }
        }

        ccd_active
    }

    /// Find the first time a CCD-enabled body has a non-sensor collider hitting another non-sensor collider.
    pub fn find_first_impact<Bodies, Colliders>(
        &mut self,
        dt: Real,
        islands: &IslandManager,
        bodies: &Bodies,
        colliders: &Colliders,
        narrow_phase: &NarrowPhase,
    ) -> Option<Real>
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyCcd>
            + ComponentSet<RigidBodyColliders>
            + ComponentSet<RigidBodyForces>
            + ComponentSet<RigidBodyMassProps>,
        Colliders: ComponentSetOption<ColliderParent>
            + ComponentSet<ColliderPosition>
            + ComponentSet<ColliderShape>
            + ComponentSet<ColliderType>
            + ComponentSet<ColliderFlags>,
    {
        // Update the query pipeline.
        self.query_pipeline.update_with_mode(
            islands,
            bodies,
            colliders,
            QueryPipelineMode::SweepTestWithPredictedPosition { dt },
        );

        let mut pairs_seen = HashMap::default();
        let mut min_toi = dt;

        for handle in islands.active_dynamic_bodies() {
            let rb_ccd1: &RigidBodyCcd = bodies.index(handle.0);

            if rb_ccd1.ccd_active {
                let (rb_pos1, rb_vels1, forces1, rb_mprops1, rb_colliders1): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyForces,
                    &RigidBodyMassProps,
                    &RigidBodyColliders,
                ) = bodies.index_bundle(handle.0);

                let predicted_body_pos1 =
                    rb_pos1.integrate_forces_and_velocities(dt, forces1, rb_vels1, rb_mprops1);

                for ch1 in &rb_colliders1.0 {
                    let co_parent1: &ColliderParent = colliders
                        .get(ch1.0)
                        .expect("Could not find the ColliderParent component.");
                    let (co_shape1, co_pos1, co_type1): (
                        &ColliderShape,
                        &ColliderPosition,
                        &ColliderType,
                    ) = colliders.index_bundle(ch1.0);

                    if co_type1.is_sensor() {
                        continue; // Ignore sensors.
                    }

                    let predicted_collider_pos1 = predicted_body_pos1 * co_parent1.pos_wrt_parent;
                    let aabb1 = co_shape1.compute_swept_aabb(&co_pos1, &predicted_collider_pos1);

                    self.query_pipeline
                        .colliders_with_aabb_intersecting_aabb(&aabb1, |ch2| {
                            if *ch1 == *ch2 {
                                // Ignore self-intersection.
                                return true;
                            }

                            if pairs_seen
                                .insert(
                                    SortedPair::new(ch1.into_raw_parts().0, ch2.into_raw_parts().0),
                                    (),
                                )
                                .is_none()
                            {
                                let co_parent1: Option<&ColliderParent> = colliders.get(ch1.0);
                                let co_parent2: Option<&ColliderParent> = colliders.get(ch2.0);
                                let c1: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch1.0);
                                let c2: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch2.0);
                                let co_type1: &ColliderType = colliders.index(ch1.0);
                                let co_type2: &ColliderType = colliders.index(ch1.0);

                                let bh1 = co_parent1.map(|p| p.handle);
                                let bh2 = co_parent2.map(|p| p.handle);

                                // Ignore self-intersection and sensors and apply collision groups filter.
                                if bh1 == bh2                                                 // Ignore self-intersection.
                                    || (co_type1.is_sensor() || co_type2.is_sensor())         // Ignore sensors.
                                    || !c1.3.collision_groups.test(c2.3.collision_groups) // Apply collision groups.
                                    || !c1.3.solver_groups.test(c2.3.solver_groups)
                                // Apply solver groups.
                                {
                                    return true;
                                }

                                let smallest_dist = narrow_phase
                                    .contact_pair(*ch1, *ch2)
                                    .and_then(|p| p.find_deepest_contact())
                                    .map(|c| c.1.dist)
                                    .unwrap_or(0.0);

                                let b2 = bh2.map(|h| bodies.index_bundle(h.0));

                                if let Some(toi) = TOIEntry::try_from_colliders(
                                    self.query_pipeline.query_dispatcher(),
                                    *ch1,
                                    *ch2,
                                    (c1.0, c1.1, c1.2, c1.3, co_parent1),
                                    (c2.0, c2.1, c2.2, c2.3, co_parent2),
                                    Some((rb_pos1, rb_vels1, rb_mprops1, rb_ccd1)),
                                    b2,
                                    None,
                                    None,
                                    0.0,
                                    min_toi,
                                    smallest_dist,
                                ) {
                                    min_toi = min_toi.min(toi.toi);
                                }
                            }

                            true
                        });
                }
            }
        }

        if min_toi < dt {
            Some(min_toi)
        } else {
            None
        }
    }

    /// Outputs the set of bodies as well as their first time-of-impact event.
    pub fn predict_impacts_at_next_positions<Bodies, Colliders>(
        &mut self,
        dt: Real,
        islands: &IslandManager,
        bodies: &Bodies,
        colliders: &Colliders,
        narrow_phase: &NarrowPhase,
        events: &dyn EventHandler,
    ) -> PredictedImpacts
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyCcd>
            + ComponentSet<RigidBodyColliders>
            + ComponentSet<RigidBodyForces>
            + ComponentSet<RigidBodyMassProps>,
        Colliders: ComponentSetOption<ColliderParent>
            + ComponentSet<ColliderPosition>
            + ComponentSet<ColliderShape>
            + ComponentSet<ColliderType>
            + ComponentSet<ColliderFlags>
            + ComponentSet<ColliderFlags>,
    {
        let mut frozen = HashMap::<_, Real>::default();
        let mut all_toi = BinaryHeap::new();
        let mut pairs_seen = HashMap::default();
        let mut min_overstep = dt;

        // Update the query pipeline.
        self.query_pipeline.update_with_mode(
            islands,
            bodies,
            colliders,
            QueryPipelineMode::SweepTestWithNextPosition,
        );

        /*
         *
         * First, collect all TOIs.
         *
         */
        // TODO: don't iterate through all the colliders.
        for handle in islands.active_dynamic_bodies() {
            let rb_ccd1: &RigidBodyCcd = bodies.index(handle.0);

            if rb_ccd1.ccd_active {
                let (rb_pos1, rb_vels1, forces1, rb_mprops1, rb_colliders1): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyForces,
                    &RigidBodyMassProps,
                    &RigidBodyColliders,
                ) = bodies.index_bundle(handle.0);

                let predicted_body_pos1 =
                    rb_pos1.integrate_forces_and_velocities(dt, forces1, rb_vels1, rb_mprops1);

                for ch1 in &rb_colliders1.0 {
                    let co_parent1: &ColliderParent = colliders
                        .get(ch1.0)
                        .expect("Could not find the ColliderParent component.");
                    let (co_shape1, co_pos1): (&ColliderShape, &ColliderPosition) =
                        colliders.index_bundle(ch1.0);

                    let predicted_collider_pos1 = predicted_body_pos1 * co_parent1.pos_wrt_parent;
                    let aabb1 = co_shape1.compute_swept_aabb(&co_pos1, &predicted_collider_pos1);

                    self.query_pipeline
                        .colliders_with_aabb_intersecting_aabb(&aabb1, |ch2| {
                            if *ch1 == *ch2 {
                                // Ignore self-intersection.
                                return true;
                            }

                            if pairs_seen
                                .insert(
                                    SortedPair::new(ch1.into_raw_parts().0, ch2.into_raw_parts().0),
                                    (),
                                )
                                .is_none()
                            {
                                let co_parent1: Option<&ColliderParent> = colliders.get(ch1.0);
                                let co_parent2: Option<&ColliderParent> = colliders.get(ch2.0);
                                let c1: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch1.0);
                                let c2: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch2.0);

                                let bh1 = co_parent1.map(|p| p.handle);
                                let bh2 = co_parent2.map(|p| p.handle);

                                // Ignore self-intersections and apply groups filter.
                                if bh1 == bh2 || !c1.3.collision_groups.test(c2.3.collision_groups)
                                {
                                    return true;
                                }

                                let smallest_dist = narrow_phase
                                    .contact_pair(*ch1, *ch2)
                                    .and_then(|p| p.find_deepest_contact())
                                    .map(|c| c.1.dist)
                                    .unwrap_or(0.0);

                                let b1 = bh1.map(|h| bodies.index_bundle(h.0));
                                let b2 = bh2.map(|h| bodies.index_bundle(h.0));

                                if let Some(toi) = TOIEntry::try_from_colliders(
                                    self.query_pipeline.query_dispatcher(),
                                    *ch1,
                                    *ch2,
                                    (c1.0, c1.1, c1.2, c1.3, co_parent1),
                                    (c2.0, c2.1, c2.2, c2.3, co_parent2),
                                    b1,
                                    b2,
                                    None,
                                    None,
                                    0.0,
                                    // NOTE: we use dt here only once we know that
                                    // there is at least one TOI before dt.
                                    min_overstep,
                                    smallest_dist,
                                ) {
                                    if toi.toi > dt {
                                        min_overstep = min_overstep.min(toi.toi);
                                    } else {
                                        min_overstep = dt;
                                        all_toi.push(toi);
                                    }
                                }
                            }

                            true
                        });
                }
            }
        }

        /*
         *
         * If the smallest TOI is outside of the time interval, return.
         *
         */
        if min_overstep == dt && all_toi.is_empty() {
            return PredictedImpacts::NoImpacts;
        } else if min_overstep > dt {
            return PredictedImpacts::ImpactsAfterEndTime(min_overstep);
        }

        // NOTE: all static bodies (and kinematic bodies?) should be considered as "frozen", this
        // may avoid some resweeps.
        let mut pseudo_intersections_to_check = vec![];

        while let Some(toi) = all_toi.pop() {
            assert!(toi.toi <= dt);

            let rb1: Option<(&RigidBodyCcd, &RigidBodyColliders)> =
                toi.b1.map(|b| bodies.index_bundle(b.0));
            let rb2: Option<(&RigidBodyCcd, &RigidBodyColliders)> =
                toi.b2.map(|b| bodies.index_bundle(b.0));

            let mut colliders_to_check = Vec::new();
            let should_freeze1 = rb1.is_some()
                && rb1.unwrap().0.ccd_active
                && !frozen.contains_key(&toi.b1.unwrap());
            let should_freeze2 = rb2.is_some()
                && rb2.unwrap().0.ccd_active
                && !frozen.contains_key(&toi.b2.unwrap());

            if !should_freeze1 && !should_freeze2 {
                continue;
            }

            if toi.is_pseudo_intersection_test {
                // NOTE: this test is redundant with the previous `if !should_freeze && ...`
                //       but let's keep it to avoid tricky regressions if we end up swapping both
                //       `if` for some reasons in the future.
                if should_freeze1 || should_freeze2 {
                    // This is only an intersection so we don't have to freeze and there is no
                    // need to resweep. However we will need to see if we have to generate
                    // intersection events, so push the TOI for further testing.
                    pseudo_intersections_to_check.push(toi);
                }
                continue;
            }

            if should_freeze1 {
                let _ = frozen.insert(toi.b1.unwrap(), toi.toi);
                colliders_to_check.extend_from_slice(&rb1.unwrap().1 .0);
            }

            if should_freeze2 {
                let _ = frozen.insert(toi.b2.unwrap(), toi.toi);
                colliders_to_check.extend_from_slice(&rb2.unwrap().1 .0);
            }

            let start_time = toi.toi;

            // NOTE: the 1 and 2 indices (e.g., `ch1`, `ch2`) bellow are unrelated to the
            //       ones we used above.
            for ch1 in &colliders_to_check {
                let co_parent1: &ColliderParent = colliders.get(ch1.0).unwrap();
                let (co_shape1, co_pos1): (&ColliderShape, &ColliderPosition) =
                    colliders.index_bundle(ch1.0);

                let rb_pos1: &RigidBodyPosition = bodies.index(co_parent1.handle.0);
                let co_next_pos1 = rb_pos1.next_position * co_parent1.pos_wrt_parent;
                let aabb = co_shape1.compute_swept_aabb(&co_pos1, &co_next_pos1);

                self.query_pipeline
                    .colliders_with_aabb_intersecting_aabb(&aabb, |ch2| {
                        let co_parent1: Option<&ColliderParent> = colliders.get(ch1.0);
                        let co_parent2: Option<&ColliderParent> = colliders.get(ch2.0);
                        let c1: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch1.0);
                        let c2: (_, _, _, &ColliderFlags) = colliders.index_bundle(ch2.0);

                        let bh1 = co_parent1.map(|p| p.handle);
                        let bh2 = co_parent2.map(|p| p.handle);

                        // Ignore self-intersection and apply groups filter.
                        if bh1 == bh2 || !c1.3.collision_groups.test(c2.3.collision_groups) {
                            return true;
                        }

                        let frozen1 = bh1.and_then(|h| frozen.get(&h));
                        let frozen2 = bh2.and_then(|h| frozen.get(&h));

                        let b1: Option<(_, _, _, &RigidBodyCcd)> =
                            bh1.map(|h| bodies.index_bundle(h.0));
                        let b2: Option<(_, _, _, &RigidBodyCcd)> =
                            bh2.map(|h| bodies.index_bundle(h.0));

                        if (frozen1.is_some() || !b1.map(|b| b.3.ccd_active).unwrap_or(false))
                            && (frozen2.is_some() || !b2.map(|b| b.3.ccd_active).unwrap_or(false))
                        {
                            // We already did a resweep.
                            return true;
                        }

                        let smallest_dist = narrow_phase
                            .contact_pair(*ch1, *ch2)
                            .and_then(|p| p.find_deepest_contact())
                            .map(|c| c.1.dist)
                            .unwrap_or(0.0);

                        if let Some(toi) = TOIEntry::try_from_colliders(
                            self.query_pipeline.query_dispatcher(),
                            *ch1,
                            *ch2,
                            (c1.0, c1.1, c1.2, c1.3, co_parent1),
                            (c2.0, c2.1, c2.2, c2.3, co_parent2),
                            b1,
                            b2,
                            frozen1.copied(),
                            frozen2.copied(),
                            start_time,
                            dt,
                            smallest_dist,
                        ) {
                            all_toi.push(toi);
                        }

                        true
                    });
            }
        }

        for toi in pseudo_intersections_to_check {
            // See if the intersection is still active once the bodies
            // reach their final positions.
            // - If the intersection is still active, don't report it yet. It will be
            //   reported by the narrow-phase at the next timestep/substep.
            // - If the intersection isn't active anymore, and it wasn't intersecting
            //   before, then we need to generate one interaction-start and one interaction-stop
            //   events because it will never be detected by the narrow-phase because of tunneling.
            let (co_type1, co_pos1, co_shape1, co_flags1): (
                &ColliderType,
                &ColliderPosition,
                &ColliderShape,
                &ColliderFlags,
            ) = colliders.index_bundle(toi.c1.0);
            let (co_type2, co_pos2, co_shape2, co_flags2): (
                &ColliderType,
                &ColliderPosition,
                &ColliderShape,
                &ColliderFlags,
            ) = colliders.index_bundle(toi.c2.0);

            if !co_type1.is_sensor() && !co_type2.is_sensor() {
                // TODO: this happens if we found a TOI between two non-sensor
                //       colliders with mismatching solver_flags. It is not clear
                //       what we should do in this case: we could report a
                //       contact started/contact stopped event for example. But in
                //       that case, what contact pair should be pass to these events?
                // For now we just ignore this special case. Let's wait for an actual
                // use-case to come up before we determine what we want to do here.
                continue;
            }

            let co_next_pos1 = if let Some(b1) = toi.b1 {
                let co_parent1: &ColliderParent = colliders.get(toi.c1.0).unwrap();
                let (rb_pos1, rb_vels1, rb_mprops1): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyMassProps,
                ) = bodies.index_bundle(b1.0);

                let local_com1 = &rb_mprops1.local_mprops.local_com;
                let frozen1 = frozen.get(&b1);
                let pos1 = frozen1
                    .map(|t| rb_vels1.integrate(*t, &rb_pos1.position, local_com1))
                    .unwrap_or(rb_pos1.next_position);
                pos1 * co_parent1.pos_wrt_parent
            } else {
                co_pos1.0
            };

            let co_next_pos2 = if let Some(b2) = toi.b2 {
                let co_parent2: &ColliderParent = colliders.get(toi.c2.0).unwrap();
                let (rb_pos2, rb_vels2, rb_mprops2): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyMassProps,
                ) = bodies.index_bundle(b2.0);

                let local_com2 = &rb_mprops2.local_mprops.local_com;
                let frozen2 = frozen.get(&b2);
                let pos2 = frozen2
                    .map(|t| rb_vels2.integrate(*t, &rb_pos2.position, local_com2))
                    .unwrap_or(rb_pos2.next_position);
                pos2 * co_parent2.pos_wrt_parent
            } else {
                co_pos2.0
            };

            let prev_coll_pos12 = co_pos1.inv_mul(&co_pos2);
            let next_coll_pos12 = co_next_pos1.inv_mul(&co_next_pos2);

            let query_dispatcher = self.query_pipeline.query_dispatcher();
            let intersect_before = query_dispatcher
                .intersection_test(&prev_coll_pos12, co_shape1.as_ref(), co_shape2.as_ref())
                .unwrap_or(false);

            let intersect_after = query_dispatcher
                .intersection_test(&next_coll_pos12, co_shape1.as_ref(), co_shape2.as_ref())
                .unwrap_or(false);

            if !intersect_before
                && !intersect_after
                && (co_flags1.active_events | co_flags2.active_events)
                    .contains(ActiveEvents::INTERSECTION_EVENTS)
            {
                // Emit one intersection-started and one intersection-stopped event.
                events.handle_intersection_event(IntersectionEvent::new(toi.c1, toi.c2, true));
                events.handle_intersection_event(IntersectionEvent::new(toi.c1, toi.c2, false));
            }
        }

        PredictedImpacts::Impacts(frozen)
    }
}
