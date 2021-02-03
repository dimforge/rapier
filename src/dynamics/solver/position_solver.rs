use super::AnyJointPositionConstraint;
use crate::dynamics::{
    solver::AnyPositionConstraint, IntegrationParameters, IslandSet, RigidBodySet,
};
use crate::math::{Isometry, Real};

pub(crate) struct PositionSolver {
    positions: Vec<Isometry<Real>>,
}

impl PositionSolver {
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
        }
    }

    pub fn solve(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandSet,
        bodies: &mut RigidBodySet,
        contact_constraints: &[AnyPositionConstraint],
        joint_constraints: &[AnyJointPositionConstraint],
    ) {
        self.positions.clear();
        self.positions.extend(
            islands
                .active_island(island_id)
                .bodies()
                .iter()
                .filter_map(|h| bodies.get(*h))
                .map(|b| b.position),
        );

        for _ in 0..params.max_position_iterations {
            for constraint in joint_constraints {
                constraint.solve(params, &mut self.positions)
            }

            for constraint in contact_constraints {
                constraint.solve(params, &mut self.positions)
            }
        }

        for handle in islands.active_island(island_id).bodies() {
            if let Some(rb) = bodies.get_mut(*handle) {
                rb.set_position_internal(self.positions[rb.island_offset])
            }
        }
    }
}
