use super::{JointConstraintTypes, SolverConstraintsSet};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    MultibodyLinkId, RigidBodySet,
    solver::{ContactConstraintTypes, SolverVel},
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::Real;
use crate::prelude::RigidBodyVelocity;
use crate::utils::SimdAngularInertia;
use na::DVector;

pub(crate) struct VelocitySolver {
    pub solver_bodies: Vec<SolverBody>,
    pub solver_vels: Vec<SolverVel<Real>>,
    pub solver_vels_increment: Vec<SolverVel<Real>>,
    pub generic_solver_vels: DVector<Real>,
    pub generic_solver_vels_increment: DVector<Real>,
    pub multibody_roots: Vec<MultibodyLinkId>,
}

impl VelocitySolver {
    pub fn new() -> Self {
        Self {
            solver_bodies: Vec::new(),
            solver_vels: Vec::new(),
            solver_vels_increment: Vec::new(),
            generic_solver_vels: DVector::zeros(0),
            generic_solver_vels_increment: DVector::zeros(0),
            multibody_roots: Vec::new(),
        }
    }

    pub fn init_constraints(
        &self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        manifolds_all: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        joints_all: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        contact_constraints: &mut SolverConstraintsSet<ContactConstraintTypes>,
        joint_constraints: &mut SolverConstraintsSet<JointConstraintTypes>,
    ) {
        contact_constraints.init(
            island_id,
            islands,
            bodies,
            multibodies,
            manifolds_all,
            manifold_indices,
        );

        joint_constraints.init(
            island_id,
            islands,
            bodies,
            multibodies,
            joints_all,
            joint_indices,
        );
    }

    pub fn init_solver_velocities_and_solver_bodies(
        &mut self,
        params: &IntegrationParameters,
        island_id: usize,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        self.multibody_roots.clear();
        self.solver_bodies.clear();
        self.solver_bodies.resize(
            islands.active_island(island_id).len(),
            SolverBody::default(),
        );

        self.solver_vels_increment.clear();
        self.solver_vels_increment
            .resize(islands.active_island(island_id).len(), SolverVel::zero());
        self.solver_vels.clear();
        self.solver_vels
            .resize(islands.active_island(island_id).len(), SolverVel::zero());

        /*
         * Initialize solver bodies and delta-velocities (`solver_vels_increment`) with external forces (gravity etc):
         * NOTE: we compute this only once by neglecting changes of mass matrices.
         */

        // Assign solver ids to multibodies, and collect the relevant roots.
        // And init solver_vels for rigidb-bodies.
        let mut multibody_solver_id = 0;
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    multibody.solver_id = multibody_solver_id;
                    multibody_solver_id += multibody.ndofs();
                    self.multibody_roots.push(link);
                }
            } else {
                let rb = &bodies[*handle];
                let solver_vel = &mut self.solver_vels[rb.ids.active_set_offset];
                let solver_vel_incr = &mut self.solver_vels_increment[rb.ids.active_set_offset];
                let solver_body = &mut self.solver_bodies[rb.ids.active_set_offset];
                solver_body.copy_from(rb);

                // NOTE: `dvel.angular` is actually storing angular velocity delta multiplied
                //       by the square root of the inertia tensor:
                solver_vel_incr.angular =
                    rb.mprops.effective_world_inv_inertia_sqrt * rb.forces.torque * params.dt;
                solver_vel_incr.linear =
                    rb.forces.force.component_mul(&rb.mprops.effective_inv_mass) * params.dt;

                solver_vel.linear = rb.vels.linvel;
                // PERF: can we avoid the call to effective_angular_inertia_sqrt?
                solver_vel.angular = rb.mprops.effective_angular_inertia_sqrt() * rb.vels.angvel;
            }
        }

        // PERF: don’t reallocate at each iteration.
        self.generic_solver_vels_increment = DVector::zeros(multibody_solver_id);
        self.generic_solver_vels = DVector::zeros(multibody_solver_id);

        // init solver_vels for multibodies.
        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            multibody.update_dynamics(params.dt, bodies);
            multibody.update_acceleration(bodies);

            let mut solver_vels_incr = self
                .generic_solver_vels_increment
                .rows_mut(multibody.solver_id, multibody.ndofs());
            let mut solver_vels = self
                .generic_solver_vels
                .rows_mut(multibody.solver_id, multibody.ndofs());

            solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            solver_vels.copy_from(&multibody.velocities);
        }
    }

    #[profiling::function]
    pub fn solve_constraints(
        &mut self,
        params: &IntegrationParameters,
        num_substeps: usize,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        contact_constraints: &mut SolverConstraintsSet<ContactConstraintTypes>,
        joint_constraints: &mut SolverConstraintsSet<JointConstraintTypes>,
    ) {
        for substep_id in 0..num_substeps {
            let is_last_substep = substep_id == num_substeps - 1;

            for (solver_vels, incr) in self
                .solver_vels
                .iter_mut()
                .zip(self.solver_vels_increment.iter())
            {
                solver_vels.linear += incr.linear;
                solver_vels.angular += incr.angular;
            }

            self.generic_solver_vels += &self.generic_solver_vels_increment;

            /*
             * Update & solve constraints.
             */
            joint_constraints.update(params, multibodies, &self.solver_bodies);
            contact_constraints.update(params, substep_id, multibodies, &self.solver_bodies);

            if params.warmstart_coefficient != 0.0 {
                contact_constraints.warmstart(&mut self.solver_vels, &mut self.generic_solver_vels);
            }

            for _ in 0..params.num_internal_pgs_iterations {
                joint_constraints.solve(&mut self.solver_vels, &mut self.generic_solver_vels);
                contact_constraints
                    .solve_restitution(&mut self.solver_vels, &mut self.generic_solver_vels);
                contact_constraints
                    .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
            }

            if is_last_substep {
                for _ in 0..params.num_additional_friction_iterations {
                    contact_constraints
                        .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
                }
            }

            /*
             * Integrate positions.
             */
            self.integrate_positions(params, is_last_substep, bodies, multibodies);

            /*
             * Resolution without bias.
             */
            if params.num_internal_stabilization_iterations > 0 {
                for _ in 0..params.num_internal_stabilization_iterations {
                    joint_constraints
                        .solve_wo_bias(&mut self.solver_vels, &mut self.generic_solver_vels);
                    contact_constraints.solve_restitution_wo_bias(
                        &mut self.solver_vels,
                        &mut self.generic_solver_vels,
                    );
                }

                contact_constraints
                    .solve_friction(&mut self.solver_vels, &mut self.generic_solver_vels);
            }
        }
    }

    #[profiling::function]
    pub fn integrate_positions(
        &mut self,
        params: &IntegrationParameters,
        is_last_substep: bool,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        // Integrate positions.
        for (solver_vels, solver_body) in self.solver_vels.iter().zip(self.solver_bodies.iter_mut())
        {
            let linvel = solver_vels.linear;
            let angvel = solver_body.sqrt_ii.transform_vector(solver_vels.angular);

            let mut new_vels = RigidBodyVelocity { linvel, angvel };
            new_vels = new_vels.apply_damping(params.dt, &solver_body.damping);
            let new_pos =
                new_vels.integrate(params.dt, &solver_body.position, &solver_body.local_com);
            solver_body.integrated_vels += new_vels;
            solver_body.position = new_pos;
            solver_body.world_com = new_pos * solver_body.local_com;
        }

        // Integrate multibody positions.
        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            let solver_vels = self
                .generic_solver_vels
                .rows(multibody.solver_id, multibody.ndofs());
            multibody.velocities.copy_from(&solver_vels);
            multibody.integrate(params.dt);
            // PERF: don’t write back to the rigid-body poses `bodies` before the last step?
            multibody.forward_kinematics(bodies, false);
            multibody.update_rigid_bodies_internal(bodies, !is_last_substep, true, false);

            if !is_last_substep {
                // These are very expensive and not needed if we don’t
                // have to run another step.
                multibody.update_dynamics(params.dt, bodies);
                multibody.update_acceleration(bodies);

                let mut solver_vels_incr = self
                    .generic_solver_vels_increment
                    .rows_mut(multibody.solver_id, multibody.ndofs());
                solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            }
        }
    }

    pub fn writeback_bodies(
        &mut self,
        params: &IntegrationParameters,
        num_substeps: usize,
        islands: &IslandManager,
        island_id: usize,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        for handle in islands.active_island(island_id) {
            let link = if self.multibody_roots.is_empty() {
                None
            } else {
                multibodies.rigid_body_link(*handle).copied()
            };

            if let Some(link) = link {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    let solver_vels = self
                        .generic_solver_vels
                        .rows(multibody.solver_id, multibody.ndofs());
                    multibody.velocities.copy_from(&solver_vels);
                }
            } else {
                let rb = bodies.index_mut_internal(*handle);
                let solver_body = &self.solver_bodies[rb.ids.active_set_offset];
                let solver_vels = &self.solver_vels[rb.ids.active_set_offset];

                let dangvel = solver_body.sqrt_ii.transform_vector(solver_vels.angular);

                let mut new_vels = RigidBodyVelocity {
                    linvel: solver_vels.linear,
                    angvel: dangvel,
                };
                new_vels = new_vels.apply_damping(params.dt, &solver_body.damping);

                // NOTE: using integrated_vels instead of interpolation is interesting for
                //       high angular velocities. However, it is a bit inexact due to the
                //       solver integrating at intermediate sub-steps. Should we just switch
                //       to interpolation?
                rb.integrated_vels.linvel =
                    solver_body.integrated_vels.linvel / num_substeps as Real;
                rb.integrated_vels.angvel =
                    solver_body.integrated_vels.angvel / num_substeps as Real;
                rb.vels = new_vels;
                rb.pos.next_position = solver_body.position;
            }
        }
    }
}
