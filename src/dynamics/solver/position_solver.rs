use super::AnyJointPositionConstraint;
use crate::data::{ComponentSet, ComponentSetMut};
use crate::dynamics::{solver::AnyPositionConstraint, IntegrationParameters};
use crate::dynamics::{IslandManager, RigidBodyIds, RigidBodyPosition};
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

    pub fn solve<Bodies>(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut Bodies,
        contact_constraints: &[AnyPositionConstraint],
        joint_constraints: &[AnyJointPositionConstraint],
    ) where
        Bodies: ComponentSet<RigidBodyIds> + ComponentSetMut<RigidBodyPosition>,
    {
        if contact_constraints.is_empty() && joint_constraints.is_empty() {
            // Nothing to do.
            return;
        }

        self.positions.clear();
        self.positions
            .extend(islands.active_island(island_id).iter().map(|h| {
                let poss: &RigidBodyPosition = bodies.index(h.0);
                poss.next_position
            }));

        for _ in 0..params.max_position_iterations {
            for constraint in joint_constraints {
                constraint.solve(params, &mut self.positions)
            }

            for constraint in contact_constraints {
                constraint.solve(params, &mut self.positions)
            }
        }

        for handle in islands.active_island(island_id) {
            let ids: &RigidBodyIds = bodies.index(handle.0);
            let next_pos = &self.positions[ids.active_set_offset];
            bodies.map_mut_internal(handle.0, |poss| poss.next_position = *next_pos);
        }
    }
}
