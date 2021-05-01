use super::AnyJointVelocityConstraint;
use crate::data::{BundleSet, ComponentSet, ComponentSetMut};
use crate::dynamics::{
    solver::{AnyVelocityConstraint, DeltaVel},
    IntegrationParameters, JointGraphEdge, RigidBodyForces, RigidBodyVelocity,
};
use crate::dynamics::{IslandManager, RigidBodyIds, RigidBodyMassProps};
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

    pub fn solve<Bodies>(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut Bodies,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        contact_constraints: &mut [AnyVelocityConstraint],
        joint_constraints: &mut [AnyJointVelocityConstraint],
    ) where
        Bodies: ComponentSet<RigidBodyForces>
            + ComponentSet<RigidBodyIds>
            + ComponentSetMut<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>,
    {
        self.mj_lambdas.clear();
        self.mj_lambdas
            .resize(islands.active_island(island_id).len(), DeltaVel::zero());

        // Initialize delta-velocities (`mj_lambdas`) with external forces (gravity etc):
        for handle in islands.active_island(island_id) {
            let (ids, mprops, forces): (&RigidBodyIds, &RigidBodyMassProps, &RigidBodyForces) =
                bodies.index_bundle(handle.0);

            let dvel = &mut self.mj_lambdas[ids.active_set_offset];

            // NOTE: `dvel.angular` is actually storing angular velocity delta multiplied
            //       by the square root of the inertia tensor:
            dvel.angular += mprops.effective_world_inv_inertia_sqrt * forces.torque * params.dt;
            dvel.linear += forces.force * (mprops.effective_inv_mass * params.dt);
        }

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
        for handle in islands.active_island(island_id) {
            let (ids, mprops): (&RigidBodyIds, &RigidBodyMassProps) = bodies.index_bundle(handle.0);

            let dvel = self.mj_lambdas[ids.active_set_offset];
            let dangvel = mprops
                .effective_world_inv_inertia_sqrt
                .transform_vector(dvel.angular);

            bodies.map_mut_internal(handle.0, |vels| {
                vels.linvel += dvel.linear;
                vels.angvel += dangvel;
            });
        }

        // Write impulses back into the manifold structures.
        for constraint in &*joint_constraints {
            constraint.writeback_impulses(joints_all);
        }

        for constraint in &*contact_constraints {
            constraint.writeback_impulses(manifolds_all);
        }
    }
}
