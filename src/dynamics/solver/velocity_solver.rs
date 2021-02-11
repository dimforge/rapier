use super::AnyJointVelocityConstraint;
use crate::dynamics::{
    solver::{AnyVelocityConstraint, DeltaVel},
    IntegrationParameters, JointGraphEdge, RigidBodySet,
};
use crate::geometry::ContactManifold;
use crate::math::Real;
use crate::utils::WAngularInertia;

pub(crate) struct VelocitySolver {
    pub mj_lambdas: Vec<DeltaVel<Real>>,
}

impl VelocitySolver {
    pub fn new() -> Self {
        Self {
            mj_lambdas: Vec::new(),
        }
    }

    pub fn solve(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &mut RigidBodySet,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        contact_constraints: &mut [AnyVelocityConstraint],
        joint_constraints: &mut [AnyJointVelocityConstraint],
    ) {
        self.mj_lambdas.clear();
        self.mj_lambdas
            .resize(bodies.active_island(island_id).len(), DeltaVel::zero());

        // Initialize delta-velocities (`mj_lambdas`) with external forces (gravity etc):
        bodies.foreach_active_island_body_mut_internal(island_id, |_, rb| {
            let dvel = &mut self.mj_lambdas[rb.active_set_offset];

            dvel.linear += rb.force * (rb.effective_inv_mass * params.dt);
            rb.force = na::zero();

            // dvel.angular is actually storing angular velocity delta multiplied by the square root of the inertia tensor:
            dvel.angular += rb.effective_world_inv_inertia_sqrt * rb.torque * params.dt;
            rb.torque = na::zero();
        });

        /*
         * Warmstart constraints.
         */
        for constraint in &*joint_constraints {
            constraint.warmstart(&mut self.mj_lambdas[..]);
        }

        for constraint in &*contact_constraints {
            constraint.warmstart(&mut self.mj_lambdas[..]);
        }

        /*
         * Solve constraints.
         */
        for _ in 0..params.max_velocity_iterations {
            for constraint in &mut *joint_constraints {
                constraint.solve(&mut self.mj_lambdas[..]);
            }

            for constraint in &mut *contact_constraints {
                constraint.solve(&mut self.mj_lambdas[..]);
            }
        }

        // Update velocities.
        bodies.foreach_active_island_body_mut_internal(island_id, |_, rb| {
            let dvel = self.mj_lambdas[rb.active_set_offset];
            rb.linvel += dvel.linear;
            rb.angvel += rb
                .effective_world_inv_inertia_sqrt
                .transform_vector(dvel.angular);
        });

        // Write impulses back into the manifold structures.
        for constraint in &*joint_constraints {
            constraint.writeback_impulses(joints_all);
        }

        for constraint in &*contact_constraints {
            constraint.writeback_impulses(manifolds_all);
        }
    }
}
