//! Physics pipeline structures.

use crate::counters::Counters;
#[cfg(not(feature = "parallel"))]
use crate::dynamics::IslandSolver;
use crate::dynamics::{IntegrationParameters, JointSet, RigidBodySet};
#[cfg(feature = "parallel")]
use crate::dynamics::{JointGraphEdge, ParallelIslandSolver as IslandSolver};
use crate::geometry::{
    BroadPhase, BroadPhasePairEvent, ColliderPair, ColliderSet, ContactManifoldIndex,
    ContactPairFilter, NarrowPhase, ProximityPairFilter,
};
use crate::math::Vector;
use crate::pipeline::EventHandler;

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

    /// Executes one timestep of the physics simulation.
    pub fn step(
        &mut self,
        gravity: &Vector<f32>,
        integration_parameters: &IntegrationParameters,
        broad_phase: &mut BroadPhase,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        joints: &mut JointSet,
        contact_pair_filter: Option<&dyn ContactPairFilter>,
        proximity_pair_filter: Option<&dyn ProximityPairFilter>,
        events: &dyn EventHandler,
    ) {
        self.counters.step_started();
        broad_phase.maintain(colliders);
        narrow_phase.maintain(colliders, bodies);
        bodies.maintain_active_set();

        // Update kinematic bodies velocities.
        // TODO: what is the best place for this? It should at least be
        // located before the island computation because we test the velocity
        // there to determine if this kinematic body should wake-up dynamic
        // bodies it is touching.
        bodies.foreach_active_kinematic_body_mut_internal(|_, body| {
            body.compute_velocity_from_predicted_position(integration_parameters.inv_dt());
        });

        self.counters.stages.collision_detection_time.start();
        self.counters.cd.broad_phase_time.start();
        self.broadphase_collider_pairs.clear();
        //        let t = instant::now();
        broad_phase.update_aabbs(
            integration_parameters.prediction_distance,
            bodies,
            colliders,
        );
        //        println!("Update AABBs time: {}", instant::now() - t);

        //        let t = instant::now();
        self.broad_phase_events.clear();
        broad_phase.find_pairs(&mut self.broad_phase_events);
        //        println!("Find pairs time: {}", instant::now() - t);

        narrow_phase.register_pairs(colliders, bodies, &self.broad_phase_events, events);
        self.counters.cd.broad_phase_time.pause();

        //        println!("Num contact pairs: {}", pairs.len());

        self.counters.cd.narrow_phase_time.start();

        //        let t = instant::now();
        narrow_phase.compute_contacts(
            integration_parameters.prediction_distance,
            bodies,
            colliders,
            contact_pair_filter,
            events,
        );
        narrow_phase.compute_proximities(
            integration_parameters.prediction_distance,
            bodies,
            colliders,
            proximity_pair_filter,
            events,
        );
        //        println!("Compute contact time: {}", instant::now() - t);

        self.counters.stages.island_construction_time.start();
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
        narrow_phase.sort_and_select_active_contacts(
            bodies,
            &mut manifolds,
            &mut self.manifold_indices,
        );
        joints.select_active_interactions(bodies, &mut self.joint_constraint_indices);

        self.counters.cd.narrow_phase_time.pause();
        self.counters.stages.collision_detection_time.pause();

        self.counters.stages.update_time.start();
        bodies.foreach_active_dynamic_body_mut_internal(|_, b| {
            b.update_world_mass_properties();
            b.integrate_accelerations(integration_parameters.dt(), *gravity)
        });
        self.counters.stages.update_time.pause();

        self.counters.solver.reset();
        self.counters.stages.solver_time.start();
        if self.solvers.len() < bodies.num_islands() {
            self.solvers
                .resize_with(bodies.num_islands(), || IslandSolver::new());
        }

        #[cfg(not(feature = "parallel"))]
        {
            enable_flush_to_zero!();

            for island_id in 0..bodies.num_islands() {
                self.solvers[island_id].solve_island(
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

                        solver.solve_island(
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

        // Update colliders positions and kinematic bodies positions.
        // FIXME: do this in the solver?
        bodies.foreach_active_body_mut_internal(|_, rb| {
            if rb.is_kinematic() {
                rb.position = rb.predicted_position;
                rb.linvel = na::zero();
                rb.angvel = na::zero();
            } else {
                rb.update_predicted_position(integration_parameters.dt());
            }

            for handle in &rb.colliders {
                let collider = &mut colliders[*handle];
                collider.position = rb.position * collider.delta;
                collider.predicted_position = rb.predicted_position * collider.delta;
            }
        });

        self.counters.stages.solver_time.pause();

        bodies.modified_inactive_set.clear();
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
            None,
            None,
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
            None,
            None,
            &(),
        );
    }

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
