use super::TOIEntry;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::{ColliderSet, IntersectionEvent};
use crate::math::Real;
use crate::parry::utils::SortedPair;
use crate::pipeline::{EventHandler, QueryPipeline, QueryPipelineMode};
use parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use parry::utils::hashmap::HashMap;
use std::collections::BinaryHeap;

pub enum PredictedImpacts {
    Impacts(HashMap<RigidBodyHandle, Real>),
    ImpactsAfterEndTime(Real),
    NoImpacts,
}

/// Solver responsible for performing motion-clamping on fast-moving bodies.
pub struct CCDSolver {
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
    pub fn clamp_motions(&self, dt: Real, bodies: &mut RigidBodySet, impacts: &PredictedImpacts) {
        match impacts {
            PredictedImpacts::Impacts(tois) => {
                for (handle, toi) in tois {
                    if let Some(body) = bodies.get_mut_internal(*handle) {
                        let min_toi =
                            (body.ccd_thickness * 0.15 * crate::utils::inv(body.linvel.norm()))
                                .min(dt);
                        // println!("Min toi: {}, Toi: {}", min_toi, toi);
                        body.integrate_next_position(toi.max(min_toi), false);
                    }
                }
            }
            _ => {}
        }
    }

    /// Updates the set of bodies that needs CCD to be resolved.
    ///
    /// Returns `true` if any rigid-body must have CCD resolved.
    pub fn update_ccd_active_flags(&self, bodies: &mut RigidBodySet, dt: Real) -> bool {
        let mut ccd_active = false;

        bodies.foreach_active_dynamic_body_mut_internal(|_, body| {
            body.update_ccd_active_flag(dt);
            ccd_active = ccd_active || body.is_ccd_active();
        });

        ccd_active
    }

    /// Find the first time a CCD-enabled body has a non-sensor collider hitting another non-sensor collider.
    pub fn find_first_impact(
        &mut self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) -> Option<Real> {
        // Update the query pipeline.
        self.query_pipeline.update_with_mode(
            bodies,
            colliders,
            QueryPipelineMode::SweepTestWithPredictedPosition { dt },
        );

        let mut pairs_seen = HashMap::default();
        let mut min_toi = dt;

        for (_, rb1) in bodies.iter_active_dynamic() {
            if rb1.is_ccd_active() {
                let predicted_body_pos1 = rb1.predict_position_using_velocity_and_forces(dt);

                for ch1 in &rb1.colliders {
                    let co1 = &colliders[*ch1];

                    if co1.is_sensor() {
                        continue; // Ignore sensors.
                    }

                    let aabb1 =
                        co1.compute_swept_aabb(&(predicted_body_pos1 * co1.position_wrt_parent()));

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
                                let c1 = colliders.get(*ch1).unwrap();
                                let c2 = colliders.get(*ch2).unwrap();
                                let bh1 = c1.parent();
                                let bh2 = c2.parent();

                                if bh1 == bh2 || (c1.is_sensor() || c2.is_sensor()) {
                                    // Ignore self-intersection and sensors.
                                    return true;
                                }

                                let b1 = bodies.get(bh1).unwrap();
                                let b2 = bodies.get(bh2).unwrap();

                                if let Some(toi) = TOIEntry::try_from_colliders(
                                    self.query_pipeline.query_dispatcher(),
                                    *ch1,
                                    *ch2,
                                    c1,
                                    c2,
                                    b1,
                                    b2,
                                    None,
                                    None,
                                    0.0,
                                    min_toi,
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
    pub fn predict_impacts_at_next_positions(
        &mut self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) -> PredictedImpacts {
        let mut frozen = HashMap::<_, Real>::default();
        let mut all_toi = BinaryHeap::new();
        let mut pairs_seen = HashMap::default();
        let mut min_overstep = dt;

        // Update the query pipeline.
        self.query_pipeline.update_with_mode(
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
        for (ch1, co1) in colliders.iter() {
            let rb1 = &bodies[co1.parent()];
            if rb1.is_ccd_active() {
                let aabb = co1.compute_swept_aabb(&(rb1.next_position * co1.position_wrt_parent()));

                self.query_pipeline
                    .colliders_with_aabb_intersecting_aabb(&aabb, |ch2| {
                        if ch1 == *ch2 {
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
                            let c1 = colliders.get(ch1).unwrap();
                            let c2 = colliders.get(*ch2).unwrap();
                            let bh1 = c1.parent();
                            let bh2 = c2.parent();

                            if bh1 == bh2 {
                                // Ignore self-intersection.
                                return true;
                            }

                            let b1 = bodies.get(bh1).unwrap();
                            let b2 = bodies.get(bh2).unwrap();

                            if let Some(toi) = TOIEntry::try_from_colliders(
                                self.query_pipeline.query_dispatcher(),
                                ch1,
                                *ch2,
                                c1,
                                c2,
                                b1,
                                b2,
                                None,
                                None,
                                0.0,
                                // NOTE: we use dt here only once we know that
                                // there is at least one TOI before dt.
                                min_overstep,
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
        let mut intersections_to_check = vec![];

        while let Some(toi) = all_toi.pop() {
            assert!(toi.toi <= dt);

            let body1 = bodies.get(toi.b1).unwrap();
            let body2 = bodies.get(toi.b2).unwrap();

            let mut colliders_to_check = Vec::new();
            let should_freeze1 = body1.is_ccd_active() && !frozen.contains_key(&toi.b1);
            let should_freeze2 = body2.is_ccd_active() && !frozen.contains_key(&toi.b2);

            if !should_freeze1 && !should_freeze2 {
                continue;
            }

            if toi.is_intersection_test {
                // NOTE: this test is rendundant with the previous `if !should_freeze && ...`
                //       but let's keep it to avoid tricky regressions if we end up swapping both
                //       `if` for some reasons in the future.
                if should_freeze1 || should_freeze2 {
                    // This is only an intersection so we don't have to freeze and there is no
                    // need to resweep. However we will need to see if we have to generate
                    // intersection events, so push the TOI for further testing.
                    intersections_to_check.push(toi);
                }
                continue;
            }

            if should_freeze1 {
                let _ = frozen.insert(toi.b1, toi.toi);
                colliders_to_check.extend_from_slice(&body1.colliders);
            }

            if should_freeze2 {
                let _ = frozen.insert(toi.b2, toi.toi);
                colliders_to_check.extend_from_slice(&body2.colliders);
            }

            let start_time = toi.toi;

            for ch1 in &colliders_to_check {
                let co1 = &colliders[*ch1];
                let rb1 = &bodies[co1.parent];
                let aabb = co1.compute_swept_aabb(&(rb1.next_position * co1.position_wrt_parent()));

                self.query_pipeline
                    .colliders_with_aabb_intersecting_aabb(&aabb, |ch2| {
                        let c1 = colliders.get(*ch1).unwrap();
                        let c2 = colliders.get(*ch2).unwrap();
                        let bh1 = c1.parent();
                        let bh2 = c2.parent();

                        if bh1 == bh2 {
                            // Ignore self-intersection.
                            return true;
                        }

                        let frozen1 = frozen.get(&bh1);
                        let frozen2 = frozen.get(&bh2);

                        let b1 = bodies.get(bh1).unwrap();
                        let b2 = bodies.get(bh2).unwrap();

                        if (frozen1.is_some() || !b1.is_ccd_active())
                            && (frozen2.is_some() || !b2.is_ccd_active())
                        {
                            // We already did a resweep.
                            return true;
                        }

                        if let Some(toi) = TOIEntry::try_from_colliders(
                            self.query_pipeline.query_dispatcher(),
                            *ch1,
                            *ch2,
                            c1,
                            c2,
                            b1,
                            b2,
                            frozen1.copied(),
                            frozen2.copied(),
                            start_time,
                            dt,
                        ) {
                            all_toi.push(toi);
                        }

                        true
                    });
            }
        }

        for toi in intersections_to_check {
            // See if the intersection is still active once the bodies
            // reach their final positions.
            // - If the intersection is still active, don't report it yet. It will be
            //   reported by the narrow-phase at the next timestep/substep.
            // - If the intersection isn't active anymore, and it wasn't intersecting
            //   before, then we need to generate one interaction-start and one interaction-stop
            //   events because it will never be detected by the narrow-phase because of tunneling.
            let body1 = &bodies[toi.b1];
            let body2 = &bodies[toi.b2];
            let co1 = &colliders[toi.c1];
            let co2 = &colliders[toi.c2];
            let frozen1 = frozen.get(&toi.b1);
            let frozen2 = frozen.get(&toi.b2);
            let pos1 = frozen1
                .map(|t| body1.integrate_velocity(*t))
                .unwrap_or(body1.next_position);
            let pos2 = frozen2
                .map(|t| body2.integrate_velocity(*t))
                .unwrap_or(body2.next_position);

            let prev_coll_pos12 = co1.position.inv_mul(&co2.position);
            let next_coll_pos12 =
                (pos1 * co1.position_wrt_parent()).inverse() * (pos2 * co2.position_wrt_parent());

            let query_dispatcher = self.query_pipeline.query_dispatcher();
            let intersect_before = query_dispatcher
                .intersection_test(&prev_coll_pos12, co1.shape(), co2.shape())
                .unwrap_or(false);

            let intersect_after = query_dispatcher
                .intersection_test(&next_coll_pos12, co1.shape(), co2.shape())
                .unwrap_or(false);

            if !intersect_before && !intersect_after {
                // Emit one intersection-started and one intersection-stopped event.
                events.handle_intersection_event(IntersectionEvent::new(toi.c1, toi.c2, true));
                events.handle_intersection_event(IntersectionEvent::new(toi.c1, toi.c2, false));
            }
        }

        PredictedImpacts::Impacts(frozen)
    }
}
