//! Physics pipeline structures.

use crate::counters::Counters;
#[cfg(not(feature = "parallel"))]
use crate::dynamics::IslandSolver;
use crate::dynamics::{CCDSolver, IntegrationParameters, JointSet, RigidBodySet};
#[cfg(feature = "parallel")]
use crate::dynamics::{JointGraphEdge, ParallelIslandSolver as IslandSolver};
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderPair, ColliderSet, ContactManifoldIndex, NarrowPhase,
};
use crate::math::{Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks};

/// The physics pipeline, responsible for stepping the whole physics simulation.
///
/// This structure only contains temporary data buffers. It can be dropped and replaced by a fresh
/// copy at any time. For performance reasons it is recommended to reuse the same physics pipeline
/// instance to benefit from the cached data.
///
/// Rapier relies on a time-stepping scheme. Its force computations
/// uses two solvers:
/// - A velocity based solver based on PGS which computes forces for contact and joint constraints.
/// - A position based solver based on non-linear PGS which performs constraint stabilization (i.e. correction of errors like penetrations).
// NOTE: this contains only workspace data, so there is no point in making this serializable.
pub struct PhysicsPipeline {
    /// Counters used for benchmarking only.
    pub counters: Counters,
    manifold_indices: Vec<Vec<ContactManifoldIndex>>,
    joint_constraint_indices: Vec<Vec<ContactManifoldIndex>>,
    broadphase_collider_pairs: Vec<ColliderPair>,
    broad_phase_events: Vec<BroadPhasePairEvent>,
    solvers: Vec<IslandSolver>,
}

impl Default for PhysicsPipeline {
    fn default() -> Self {
        PhysicsPipeline::new()
    }
}

#[allow(dead_code)]
fn check_pipeline_send_sync() {
    fn do_test<T: Sync>() {}
    do_test::<PhysicsPipeline>();
}

impl PhysicsPipeline {
    /// Initializes a new physics pipeline.
    pub fn new() -> PhysicsPipeline {
        PhysicsPipeline {
            counters: Counters::new(false),
            solvers: Vec::new(),
            manifold_indices: Vec::new(),
            joint_constraint_indices: Vec::new(),
            broadphase_collider_pairs: Vec::new(),
            broad_phase_events: Vec::new(),
        }
    }

    fn detect_collisions(
        &mut self,
        integration_parameters: &IntegrationParameters,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
        handle_user_changes: bool,
    ) {
        self.counters.stages.collision_detection_time.resume();
        self.counters.cd.broad_phase_time.resume();

        // Update broad-phase.
        self.broad_phase_events.clear();
        self.broadphase_collider_pairs.clear();
        broad_phase.update(
            integration_parameters.prediction_distance,
            colliders,
            &mut self.broad_phase_events,
        );

        self.counters.cd.broad_phase_time.pause();
        self.counters.cd.narrow_phase_time.resume();

        // Update narrow-phase.
        if handle_user_changes {
            narrow_phase.handle_user_changes(colliders, bodies, events);
        }
        narrow_phase.register_pairs(colliders, bodies, &self.broad_phase_events, events);
        narrow_phase.compute_contacts(
            integration_parameters.prediction_distance,
            bodies,
            colliders,
            hooks,
            events,
        );
        narrow_phase.compute_intersections(bodies, colliders, hooks, events);

        // Clear colliders modification flags.
        colliders.clear_modified_colliders();

        self.counters.cd.narrow_phase_time.pause();
        self.counters.stages.collision_detection_time.pause();
    }

    fn solve_position_constraints(
        &mut self,
        integration_parameters: &IntegrationParameters,
        bodies: &mut RigidBodySet,
    ) {
        #[cfg(not(feature = "parallel"))]
        {
            enable_flush_to_zero!();

            for island_id in 0..bodies.num_islands() {
                self.solvers[island_id].solve_position_constraints(
                    island_id,
                    &mut self.counters,
                    integration_parameters,
                    bodies,
                )
            }
        }

        #[cfg(feature = "parallel")]
        {
            use crate::geometry::ContactManifold;
            use rayon::prelude::*;
            use std::sync::atomic::Ordering;

            let num_islands = bodies.num_islands();
            let solvers = &mut self.solvers[..num_islands];
            let bodies = &std::sync::atomic::AtomicPtr::new(bodies as *mut _);

            rayon::scope(|scope| {
                enable_flush_to_zero!();

                solvers
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(island_id, solver)| {
                        let bodies: &mut RigidBodySet =
                            unsafe { std::mem::transmute(bodies.load(Ordering::Relaxed)) };

                        solver.solve_position_constraints(
                            scope,
                            island_id,
                            integration_parameters,
                            bodies,
                        )
                    });
            });
        }
    }

    fn build_islands_and_solve_velocity_constraints(
        &mut self,
        gravity: &Vector<Real>,
        integration_parameters: &IntegrationParameters,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut JointSet,
    ) {
        self.counters.stages.island_construction_time.resume();
        bodies.update_active_set_with_contacts(
            colliders,
            narrow_phase,
            joints.joint_graph(),
            integration_parameters.min_island_size,
        );
        self.counters.stages.island_construction_time.pause();

        if self.manifold_indices.len() < bodies.num_islands() {
            self.manifold_indices
                .resize(bodies.num_islands(), Vec::new());
        }

        if self.joint_constraint_indices.len() < bodies.num_islands() {
            self.joint_constraint_indices
                .resize(bodies.num_islands(), Vec::new());
        }

        let mut manifolds = Vec::new();
        narrow_phase.select_active_contacts(bodies, &mut manifolds, &mut self.manifold_indices);
        joints.select_active_interactions(bodies, &mut self.joint_constraint_indices);

        self.counters.stages.update_time.resume();
        bodies.foreach_active_dynamic_body_mut_internal(|_, b| {
            b.update_world_mass_properties();
            b.add_gravity(*gravity)
        });
        self.counters.stages.update_time.pause();

        self.counters.stages.solver_time.resume();
        if self.solvers.len() < bodies.num_islands() {
            self.solvers
                .resize_with(bodies.num_islands(), IslandSolver::new);
        }

        #[cfg(not(feature = "parallel"))]
        {
            enable_flush_to_zero!();

            for island_id in 0..bodies.num_islands() {
                self.solvers[island_id].init_constraints_and_solve_velocity_constraints(
                    island_id,
                    &mut self.counters,
                    integration_parameters,
                    bodies,
                    &mut manifolds[..],
                    &self.manifold_indices[island_id],
                    joints.joints_mut(),
                    &self.joint_constraint_indices[island_id],
                )
            }
        }

        #[cfg(feature = "parallel")]
        {
            use crate::geometry::ContactManifold;
            use rayon::prelude::*;
            use std::sync::atomic::Ordering;

            let num_islands = bodies.num_islands();
            let solvers = &mut self.solvers[..num_islands];
            let bodies = &std::sync::atomic::AtomicPtr::new(bodies as *mut _);
            let manifolds = &std::sync::atomic::AtomicPtr::new(&mut manifolds as *mut _);
            let joints = &std::sync::atomic::AtomicPtr::new(joints.joints_vec_mut() as *mut _);
            let manifold_indices = &self.manifold_indices[..];
            let joint_constraint_indices = &self.joint_constraint_indices[..];

            rayon::scope(|scope| {
                enable_flush_to_zero!();

                solvers
                    .par_iter_mut()
                    .enumerate()
                    .for_each(|(island_id, solver)| {
                        let bodies: &mut RigidBodySet =
                            unsafe { std::mem::transmute(bodies.load(Ordering::Relaxed)) };
                        let manifolds: &mut Vec<&mut ContactManifold> =
                            unsafe { std::mem::transmute(manifolds.load(Ordering::Relaxed)) };
                        let joints: &mut Vec<JointGraphEdge> =
                            unsafe { std::mem::transmute(joints.load(Ordering::Relaxed)) };

                        solver.init_constraints_and_solve_velocity_constraints(
                            scope,
                            island_id,
                            integration_parameters,
                            bodies,
                            manifolds,
                            &manifold_indices[island_id],
                            joints,
                            &joint_constraint_indices[island_id],
                        )
                    });
            });
        }
        self.counters.stages.solver_time.pause();
    }

    fn run_ccd_motion_clamping(
        &mut self,
        integration_parameters: &IntegrationParameters,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        narrow_phase: &NarrowPhase,
        ccd_solver: &mut CCDSolver,
        events: &dyn EventHandler,
    ) {
        self.counters.ccd.toi_computation_time.start();
        // Handle CCD
        let impacts = ccd_solver.predict_impacts_at_next_positions(
            integration_parameters.dt,
            bodies,
            colliders,
            narrow_phase,
            events,
        );
        ccd_solver.clamp_motions(integration_parameters.dt, bodies, &impacts);
        self.counters.ccd.toi_computation_time.pause();
    }

    fn advance_to_final_positions(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        clear_forces: bool,
    ) {
        // Set the rigid-bodies and kinematic bodies to their final position.
        bodies.foreach_active_body_mut_internal(|_, rb| {
            if rb.is_kinematic() {
                rb.linvel = na::zero();
                rb.angvel = na::zero();
            }

            if clear_forces {
                rb.force = na::zero();
                rb.torque = na::zero();
            }

            rb.position = rb.next_position;
            rb.update_colliders_positions(colliders);
        });
    }

    fn interpolate_kinematic_velocities(
        &mut self,
        integration_parameters: &IntegrationParameters,
        bodies: &mut RigidBodySet,
    ) {
        // Update kinematic bodies velocities.
        // TODO: what is the best place for this? It should at least be
        // located before the island computation because we test the velocity
        // there to determine if this kinematic body should wake-up dynamic
        // bodies it is touching.
        bodies.foreach_active_kinematic_body_mut_internal(|_, body| {
            body.compute_velocity_from_next_position(integration_parameters.inv_dt());
        });
    }

    /// Executes one timestep of the physics simulation.
    pub fn step(
        &mut self,
        gravity: &Vector<Real>,
        integration_parameters: &IntegrationParameters,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut JointSet,
        ccd_solver: &mut CCDSolver,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        self.counters.reset();
        self.counters.step_started();
        colliders.handle_user_changes(bodies);
        bodies.handle_user_changes(colliders);

        self.detect_collisions(
            integration_parameters,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            hooks,
            events,
            true,
        );

        let mut remaining_time = integration_parameters.dt;
        let mut integration_parameters = *integration_parameters;

        let (ccd_is_enabled, mut remaining_substeps) =
            if integration_parameters.max_ccd_substeps == 0 {
                (false, 1)
            } else {
                (true, integration_parameters.max_ccd_substeps)
            };

        while remaining_substeps > 0 {
            // If there are more than one CCD substep, we need to split
            // the timestep into multiple intervals. First, estimate the
            // size of the time slice we will integrate for this substep.
            //
            // Note that we must do this now, before the constrains resolution
            // because we need to use the correct timestep length for the
            // integration of external forces.
            //
            // If there is only one or zero CCD substep, there is no need
            // to split the timetsep interval. So we can just skip this part.
            if ccd_is_enabled && remaining_substeps > 1 {
                // NOTE: Take forces into account when updating the bodies CCD activation flags
                //       these forces have not been integrated to the body's velocity yet.
                let ccd_active = ccd_solver.update_ccd_active_flags(bodies, remaining_time, true);
                let first_impact = if ccd_active {
                    ccd_solver.find_first_impact(remaining_time, bodies, colliders, narrow_phase)
                } else {
                    None
                };

                if let Some(toi) = first_impact {
                    let original_interval = remaining_time / (remaining_substeps as Real);

                    if toi < original_interval {
                        integration_parameters.dt = original_interval;
                    } else {
                        integration_parameters.dt =
                            toi + (remaining_time - toi) / (remaining_substeps as Real);
                    }

                    remaining_substeps -= 1;
                } else {
                    // No impact, don't do any other substep after this one.
                    integration_parameters.dt = remaining_time;
                    remaining_substeps = 0;
                }

                remaining_time -= integration_parameters.dt;

                // Avoid substep length that are too small.
                if remaining_time <= integration_parameters.min_ccd_dt {
                    integration_parameters.dt += remaining_time;
                    remaining_substeps = 0;
                }
            } else {
                integration_parameters.dt = remaining_time;
                remaining_time = 0.0;
                remaining_substeps = 0;
            }

            self.counters.ccd.num_substeps += 1;

            self.interpolate_kinematic_velocities(&integration_parameters, bodies);
            self.build_islands_and_solve_velocity_constraints(
                gravity,
                &integration_parameters,
                narrow_phase,
                bodies,
                colliders,
                joints,
            );

            // If CCD is enabled, execute the CCD motion clamping.
            if ccd_is_enabled {
                // NOTE: don't the forces into account when updating the CCD active flags because
                //       they have already been integrated into the velocities by the solver.
                let ccd_active =
                    ccd_solver.update_ccd_active_flags(bodies, integration_parameters.dt, false);
                if ccd_active {
                    self.run_ccd_motion_clamping(
                        &integration_parameters,
                        bodies,
                        colliders,
                        narrow_phase,
                        ccd_solver,
                        events,
                    );
                }
            }

            // NOTE: we need to run the position solver **after** the
            //       CCD motion clamping because otherwise the clamping
            //       would undo the depenetration done by the position
            //       solver.
            //       This happens because our CCD use the real rigid-body
            //       velocities instead of just interpolating between
            //       isometries.
            self.solve_position_constraints(&integration_parameters, bodies);

            let clear_forces = remaining_substeps == 0;
            self.advance_to_final_positions(bodies, colliders, clear_forces);
            self.detect_collisions(
                &integration_parameters,
                broad_phase,
                narrow_phase,
                bodies,
                colliders,
                hooks,
                events,
                false,
            );

            bodies.modified_inactive_set.clear();
        }

        self.counters.step_completed();
    }
}

#[cfg(test)]
mod test {
    use crate::dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet};
    use crate::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
    use crate::math::Vector;
    use crate::pipeline::PhysicsPipeline;

    #[test]
    fn kinematic_and_static_contact_crash() {
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhase::new();
        let mut nf = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();

        let rb = RigidBodyBuilder::new_static().build();
        let h1 = bodies.insert(rb.clone());
        let co = ColliderBuilder::ball(10.0).build();
        colliders.insert(co.clone(), h1, &mut bodies);

        // The same but with a kinematic body.
        let rb = RigidBodyBuilder::new_kinematic().build();
        let h2 = bodies.insert(rb.clone());
        colliders.insert(co, h2, &mut bodies);

        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &(),
            &(),
        );
    }

    #[test]
    fn rigid_body_removal_before_step() {
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhase::new();
        let mut nf = NarrowPhase::new();

        let mut bodies = RigidBodySet::new();

        // Check that removing the body right after inserting it works.
        // We add two dynamic bodies, one kinematic body and one static body before removing
        // them. This include a non-regression test where deleting a kimenatic body crashes.
        let rb = RigidBodyBuilder::new_dynamic().build();
        let h1 = bodies.insert(rb.clone());
        let h2 = bodies.insert(rb.clone());

        // The same but with a kinematic body.
        let rb = RigidBodyBuilder::new_kinematic().build();
        let h3 = bodies.insert(rb.clone());

        // The same but with a static body.
        let rb = RigidBodyBuilder::new_static().build();
        let h4 = bodies.insert(rb.clone());

        let to_delete = [h1, h2, h3, h4];
        for h in &to_delete {
            bodies.remove(*h, &mut colliders, &mut joints);
        }

        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &(),
            &(),
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn rigid_body_removal_snapshot_handle_determinism() {
        let mut colliders = ColliderSet::new();
        let mut joints = JointSet::new();

        let mut bodies = RigidBodySet::new();
        let rb = RigidBodyBuilder::new_dynamic().build();
        let h1 = bodies.insert(rb.clone());
        let h2 = bodies.insert(rb.clone());
        let h3 = bodies.insert(rb.clone());

        bodies.remove(h1, &mut colliders, &mut joints);
        bodies.remove(h3, &mut colliders, &mut joints);
        bodies.remove(h2, &mut colliders, &mut joints);

        let ser_bodies = bincode::serialize(&bodies).unwrap();
        let mut bodies2: RigidBodySet = bincode::deserialize(&ser_bodies).unwrap();

        let h1a = bodies.insert(rb.clone());
        let h2a = bodies.insert(rb.clone());
        let h3a = bodies.insert(rb.clone());

        let h1b = bodies2.insert(rb.clone());
        let h2b = bodies2.insert(rb.clone());
        let h3b = bodies2.insert(rb.clone());

        assert_eq!(h1a, h1b);
        assert_eq!(h2a, h2b);
        assert_eq!(h3a, h3b);
    }
}
