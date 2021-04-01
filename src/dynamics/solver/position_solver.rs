use super::AnyJointPositionConstraint;
use crate::dynamics::{solver::AnyPositionConstraint, IntegrationParameters, RigidBodySet};
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
        bodies: &mut RigidBodySet,
        contact_constraints: &[AnyPositionConstraint],
        joint_constraints: &[AnyJointPositionConstraint],
    ) {
        if contact_constraints.is_empty() && joint_constraints.is_empty() {
            // Nothing to do.
            return;
        }

        self.positions.clear();
        self.positions.extend(
            bodies
                .iter_active_island(island_id)
                .map(|(_, b)| b.next_position),
        );

        for _ in 0..params.max_position_iterations {
            for constraint in joint_constraints {
                constraint.solve(params, &mut self.positions)
            }

            for constraint in contact_constraints {
                constraint.solve(params, &mut self.positions)
            }
        }

        bodies.foreach_active_island_body_mut_internal(island_id, |_, rb| {
            rb.set_next_position(self.positions[rb.active_set_offset])
        });
    }
}
