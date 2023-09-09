//! Physics pipeline structures.

use crate::counters::Counters;
#[cfg(not(feature = "parallel"))]
use crate::dynamics::IslandSolver;
use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodyChanges, RigidBodyHandle, RigidBodyPosition, RigidBodyType,
};
#[cfg(feature = "parallel")]
use crate::dynamics::{JointGraphEdge, ParallelIslandSolver as IslandSolver};
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderChanges, ColliderHandle, ColliderPair,
    ContactManifoldIndex, NarrowPhase, TemporaryInteractionIndex,
};
use crate::math::{Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks, QueryPipeline};
use {crate::dynamics::RigidBodySet, crate::geometry::ColliderSet};

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
    contact_pair_indices: Vec<TemporaryInteractionIndex>,
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
            counters: Counters::new(true),
            solvers: vec![],
            contact_pair_indices: vec![],
            manifold_indices: vec![],
            joint_constraint_indices: vec![],
            broadphase_collider_pairs: vec![],
            broad_phase_events: vec![],
        }
    }

    fn clear_modified_colliders(
        &mut self,
        colliders: &mut ColliderSet,
        modified_colliders: &mut Vec<ColliderHandle>,
    ) {
        for handle in modified_colliders.drain(..) {
            if let Some(co) = colliders.get_mut_internal(handle) {
                co.changes = ColliderChanges::empty();
            }
        }
    }

    fn clear_modified_bodies(
        &mut self,
        bodies: &mut RigidBodySet,
        modified_bodies: &mut Vec<RigidBodyHandle>,
    ) {
        for handle in modified_bodies.drain(..) {
            if let Some(rb) = bodies.get_mut_internal(handle) {
                rb.changes = RigidBodyChanges::empty();
            }
        }
    }

    fn detect_collisions(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
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
            modified_colliders,
            removed_colliders,
            &mut self.broad_phase_events,
        );

        self.counters.cd.broad_phase_time.pause();
        self.counters.cd.narrow_phase_time.resume();

        // Update narrow-phase.
        if handle_user_changes {
            narrow_phase.handle_user_changes(
                Some(islands),
                modified_colliders,
                removed_colliders,
                colliders,
                bodies,
                events,
            );
        }
        narrow_phase.register_pairs(
            Some(islands),
            colliders,
            bodies,
            &self.broad_phase_events,
            events,
        );
        narrow_phase.compute_contacts(
            integration_parameters.prediction_distance,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            modified_colliders,
            hooks,
            events,
        );
        narrow_phase.compute_intersections(bodies, colliders, modified_colliders, hooks, events);

        self.counters.cd.narrow_phase_time.pause();
        self.counters.stages.collision_detection_time.pause();
    }

    fn build_islands_and_solve_velocity_constraints(
        &mut self,
        gravity: &Vector<Real>,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        events: &dyn EventHandler,
    ) {
        self.counters.stages.island_construction_time.resume();
        islands.update_active_set_with_contacts(
            integration_parameters.dt,
            bodies,
            colliders,
            narrow_phase,
            impulse_joints,
            multibody_joints,
            integration_parameters.min_island_size,
        );
        self.counters.stages.island_construction_time.pause();

        if self.manifold_indices.len() < islands.num_islands() {
            self.manifold_indices
                .resize(islands.num_islands(), Vec::new());
        }

        if self.joint_constraint_indices.len() < islands.num_islands() {
            self.joint_constraint_indices
                .resize(islands.num_islands(), Vec::new());
        }

        let mut manifolds = Vec::new();
        narrow_phase.select_active_contacts(
            islands,
            bodies,
            &mut self.contact_pair_indices,
            &mut manifolds,
            &mut self.manifold_indices,
        );
        impulse_joints.select_active_interactions(
            islands,
            bodies,
            &mut self.joint_constraint_indices,
        );

        self.counters.stages.update_time.resume();
        for handle in islands.active_dynamic_bodies() {
            let rb = bodies.index_mut_internal(*handle);
            rb.mprops.update_world_mass_properties(&rb.pos.position);
            let effective_mass = rb.mprops.effective_mass();
            rb.forces
                .compute_effective_force_and_torque(&gravity, &effective_mass);
        }

        for multibody in &mut multibody_joints.multibodies {
            multibody
                .1
                .update_dynamics(integration_parameters.dt, bodies);
            multibody.1.update_acceleration(bodies);
        }
        self.counters.stages.update_time.pause();

        self.counters.stages.solver_time.resume();
        if self.solvers.len() < islands.num_islands() {
            self.solvers
                .resize_with(islands.num_islands(), IslandSolver::new);
        }

        #[cfg(not(feature = "parallel"))]
        {
            enable_flush_to_zero!();

            for island_id in 0..islands.num_islands() {
                self.solvers[island_id].init_and_solve(
                    island_id,
                    &mut self.counters,
                    integration_parameters,
                    islands,
                    bodies,
                    &mut manifolds[..],
                    &self.manifold_indices[island_id],
                    impulse_joints.joints_mut(),
                    &self.joint_constraint_indices[island_id],
                    multibody_joints,
                )
            }
        }

        #[cfg(feature = "parallel")]
        {
            use crate::geometry::ContactManifold;
            use rayon::prelude::*;
            use std::sync::atomic::Ordering;

            let num_islands = islands.num_islands();
            let solvers = &mut self.solvers[..num_islands];
            let bodies = &std::sync::atomic::AtomicPtr::new(bodies as *mut _);
            let manifolds = &std::sync::atomic::AtomicPtr::new(&mut manifolds as *mut _);
            let impulse_joints =
                &std::sync::atomic::AtomicPtr::new(impulse_joints.joints_vec_mut() as *mut _);
            let multibody_joints = &std::sync::atomic::AtomicPtr::new(multibody_joints as *mut _);
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
                        let impulse_joints: &mut Vec<JointGraphEdge> =
                            unsafe { std::mem::transmute(impulse_joints.load(Ordering::Relaxed)) };
                        let multibody_joints: &mut MultibodyJointSet = unsafe {
                            std::mem::transmute(multibody_joints.load(Ordering::Relaxed))
                        };

                        solver.init_and_solve(
                            scope,
                            island_id,
                            islands,
                            integration_parameters,
                            bodies,
                            manifolds,
                            &manifold_indices[island_id],
                            impulse_joints,
                            &joint_constraint_indices[island_id],
                            multibody_joints,
                        )
                    });
            });
        }

        // Generate contact force events if needed.
        let inv_dt = crate::utils::inv(integration_parameters.dt);
        for pair_id in self.contact_pair_indices.drain(..) {
            let pair = narrow_phase.contact_pair_at_index(pair_id);
            let co1 = &colliders[pair.collider1];
            let co2 = &colliders[pair.collider2];
            let threshold = co1
                .effective_contact_force_event_threshold()
                .min(co2.effective_contact_force_event_threshold());

            if threshold < Real::MAX {
                let total_magnitude = pair.total_impulse_magnitude() * inv_dt;

                // NOTE: the strict inequality is important here, so we don’t
                //       trigger an event if the force is 0.0 and the threshold is 0.0.
                if total_magnitude > threshold {
                    events.handle_contact_force_event(
                        integration_parameters.dt,
                        bodies,
                        colliders,
                        pair,
                        total_magnitude,
                    );
                }
            }
        }

        self.counters.stages.solver_time.pause();
    }

    fn run_ccd_motion_clamping(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &IslandManager,
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
            islands,
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
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        modified_colliders: &mut Vec<ColliderHandle>,
    ) {
        // Set the rigid-bodies and kinematic bodies to their final position.
        for handle in islands.iter_active_bodies() {
            let rb = bodies.index_mut_internal(handle);
            rb.pos.position = rb.pos.next_position;
            rb.colliders
                .update_positions(colliders, modified_colliders, &rb.pos.position);
        }
    }

    fn interpolate_kinematic_velocities(
        &mut self,
        integration_parameters: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
    ) {
        // Update kinematic bodies velocities.
        // TODO: what is the best place for this? It should at least be
        // located before the island computation because we test the velocity
        // there to determine if this kinematic body should wake-up dynamic
        // bodies it is touching.
        for handle in islands.active_kinematic_bodies() {
            let rb = bodies.index_mut_internal(*handle);

            match rb.body_type {
                RigidBodyType::KinematicPositionBased => {
                    rb.vels = rb.pos.interpolate_velocity(
                        integration_parameters.inv_dt(),
                        &rb.mprops.local_mprops.local_com,
                    );
                }
                RigidBodyType::KinematicVelocityBased => {
                    let new_pos = rb.vels.integrate(
                        integration_parameters.dt,
                        &rb.pos.position,
                        &rb.mprops.local_mprops.local_com,
                    );
                    rb.pos = RigidBodyPosition::from(new_pos);
                }
                _ => {}
            }
        }
    }

    /// Executes one timestep of the physics simulation.
    pub fn step(
        &mut self,
        gravity: &Vector<Real>,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        ccd_solver: &mut CCDSolver,
        mut query_pipeline: Option<&mut QueryPipeline>,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        self.counters.reset();
        self.counters.step_started();

        // Apply some of delayed wake-ups.
        for handle in impulse_joints
            .to_wake_up
            .drain(..)
            .chain(multibody_joints.to_wake_up.drain(..))
        {
            islands.wake_up(bodies, handle, true);
        }

        // Apply modifications.
        let mut modified_colliders = colliders.take_modified();
        let mut removed_colliders = colliders.take_removed();

        super::user_changes::handle_user_changes_to_colliders(
            bodies,
            colliders,
            &modified_colliders[..],
        );

        let mut modified_bodies = bodies.take_modified();
        super::user_changes::handle_user_changes_to_rigid_bodies(
            Some(islands),
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            &modified_bodies,
            &mut modified_colliders,
        );

        // Disabled colliders are treated as if they were removed.
        // NOTE: this must be called here, after handle_user_changes_to_rigid_bodies to take into
        //       account colliders disabled because of their parent rigid-body.
        removed_colliders.extend(
            modified_colliders
                .iter()
                .copied()
                .filter(|h| colliders.get(*h).map(|c| !c.is_enabled()).unwrap_or(false)),
        );

        // TODO: do this only on user-change.
        // TODO: do we want some kind of automatic inverse kinematics?
        for multibody in &mut multibody_joints.multibodies {
            multibody.1.update_root_type(bodies);
            // FIXME: what should we do here? We should not
            //        rely on the next state here.
            multibody.1.forward_kinematics(bodies, true);
        }

        self.detect_collisions(
            integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            &modified_colliders,
            &removed_colliders,
            hooks,
            events,
            true,
        );

        if let Some(queries) = query_pipeline.as_deref_mut() {
            queries.update_incremental(colliders, &modified_colliders, &removed_colliders, false);
        }

        self.clear_modified_colliders(colliders, &mut modified_colliders);
        self.clear_modified_bodies(bodies, &mut modified_bodies);
        removed_colliders.clear();

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
                let ccd_active =
                    ccd_solver.update_ccd_active_flags(islands, bodies, remaining_time, true);
                let first_impact = if ccd_active {
                    ccd_solver.find_first_impact(
                        remaining_time,
                        islands,
                        bodies,
                        colliders,
                        narrow_phase,
                    )
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

            self.interpolate_kinematic_velocities(&integration_parameters, islands, bodies);
            self.build_islands_and_solve_velocity_constraints(
                gravity,
                &integration_parameters,
                islands,
                narrow_phase,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                events,
            );

            // If CCD is enabled, execute the CCD motion clamping.
            if ccd_is_enabled {
                // NOTE: don't the forces into account when updating the CCD active flags because
                //       they have already been integrated into the velocities by the solver.
                let ccd_active = ccd_solver.update_ccd_active_flags(
                    islands,
                    bodies,
                    integration_parameters.dt,
                    false,
                );
                if ccd_active {
                    self.run_ccd_motion_clamping(
                        &integration_parameters,
                        islands,
                        bodies,
                        colliders,
                        narrow_phase,
                        ccd_solver,
                        events,
                    );
                }
            }

            self.advance_to_final_positions(islands, bodies, colliders, &mut modified_colliders);

            self.detect_collisions(
                &integration_parameters,
                islands,
                broad_phase,
                narrow_phase,
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                &modified_colliders,
                &[],
                hooks,
                events,
                false,
            );

            if let Some(queries) = query_pipeline.as_deref_mut() {
                queries.update_incremental(
                    colliders,
                    &modified_colliders,
                    &[],
                    remaining_substeps == 0,
                );
            }

            self.clear_modified_colliders(colliders, &mut modified_colliders);
        }

        // Finally, make sure we update the world mass-properties of the rigid-bodies
        // that moved. Otherwise, users may end up applying forces with respect to an
        // outdated center of mass.
        // TODO: avoid updating the world mass properties twice (here, and
        //       at the beginning of the next timestep) for bodies that were
        //       not modified by the user in the mean time.
        for handle in islands.active_dynamic_bodies() {
            let rb = bodies.index_mut_internal(*handle);
            rb.mprops.update_world_mass_properties(&rb.pos.position);
        }

        self.counters.step_completed();
    }
}

#[cfg(test)]
mod test {
    use crate::dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, RigidBodyBuilder,
        RigidBodySet,
    };
    use crate::geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase};
    use crate::math::Vector;
    use crate::pipeline::PhysicsPipeline;
    use crate::prelude::MultibodyJointSet;

    #[test]
    fn kinematic_and_fixed_contact_crash() {
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhase::new();
        let mut nf = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut islands = IslandManager::new();

        let rb = RigidBodyBuilder::fixed().build();
        let h1 = bodies.insert(rb.clone());
        let co = ColliderBuilder::ball(10.0).build();
        colliders.insert_with_parent(co.clone(), h1, &mut bodies);

        // The same but with a kinematic body.
        let rb = RigidBodyBuilder::kinematic_position_based().build();
        let h2 = bodies.insert(rb.clone());
        colliders.insert_with_parent(co, h2, &mut bodies);

        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut CCDSolver::new(),
            None,
            &(),
            &(),
        );
    }

    #[test]
    fn rigid_body_removal_before_step() {
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhase::new();
        let mut nf = NarrowPhase::new();
        let mut islands = IslandManager::new();

        let mut bodies = RigidBodySet::new();

        // Check that removing the body right after inserting it works.
        // We add two dynamic bodies, one kinematic body and one fixed body before removing
        // them. This include a non-regression test where deleting a kimenatic body crashes.
        let rb = RigidBodyBuilder::dynamic().build();
        let h1 = bodies.insert(rb.clone());
        let h2 = bodies.insert(rb.clone());

        // The same but with a kinematic body.
        let rb = RigidBodyBuilder::kinematic_position_based().build();
        let h3 = bodies.insert(rb.clone());

        // The same but with a fixed body.
        let rb = RigidBodyBuilder::fixed().build();
        let h4 = bodies.insert(rb.clone());

        let to_delete = [h1, h2, h3, h4];
        for h in &to_delete {
            bodies.remove(
                *h,
                &mut islands,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                true,
            );
        }

        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut CCDSolver::new(),
            None,
            &(),
            &(),
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn rigid_body_removal_snapshot_handle_determinism() {
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut islands = IslandManager::new();

        let mut bodies = RigidBodySet::new();
        let rb = RigidBodyBuilder::dynamic().build();
        let h1 = bodies.insert(rb.clone());
        let h2 = bodies.insert(rb.clone());
        let h3 = bodies.insert(rb.clone());

        bodies.remove(
            h1,
            &mut islands,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            true,
        );
        bodies.remove(
            h3,
            &mut islands,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            true,
        );
        bodies.remove(
            h2,
            &mut islands,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            true,
        );

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

    #[test]
    fn collider_removal_before_step() {
        let mut pipeline = PhysicsPipeline::new();
        let gravity = Vector::y() * -9.81;
        let integration_parameters = IntegrationParameters::default();
        let mut broad_phase = BroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut ccd = CCDSolver::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut islands = IslandManager::new();
        let physics_hooks = ();
        let event_handler = ();

        let body = RigidBodyBuilder::dynamic().build();
        let b_handle = bodies.insert(body);
        let collider = ColliderBuilder::ball(1.0).build();
        let c_handle = colliders.insert_with_parent(collider, b_handle, &mut bodies);
        colliders.remove(c_handle, &mut islands, &mut bodies, true);
        bodies.remove(
            b_handle,
            &mut islands,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            true,
        );

        for _ in 0..10 {
            pipeline.step(
                &gravity,
                &integration_parameters,
                &mut islands,
                &mut broad_phase,
                &mut narrow_phase,
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut ccd,
                None,
                &physics_hooks,
                &event_handler,
            );
        }
    }
}
