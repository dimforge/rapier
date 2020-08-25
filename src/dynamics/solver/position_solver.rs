use super::{
    AnyJointPositionConstraint, InteractionGroups, PositionConstraint, PositionGroundConstraint,
};
#[cfg(feature = "simd-is-enabled")]
use super::{WPositionConstraint, WPositionGroundConstraint};
use crate::dynamics::solver::categorization::{categorize_joints, categorize_position_contacts};
use crate::dynamics::{
    solver::AnyPositionConstraint, IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::Isometry;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;

pub(crate) struct PositionSolverJointPart {
    pub nonground_joints: Vec<JointIndex>,
    pub ground_joints: Vec<JointIndex>,
    pub nonground_joint_groups: InteractionGroups,
    pub ground_joint_groups: InteractionGroups,
    pub constraints: Vec<AnyJointPositionConstraint>,
}

impl PositionSolverJointPart {
    pub fn new() -> Self {
        Self {
            nonground_joints: Vec::new(),
            ground_joints: Vec::new(),
            nonground_joint_groups: InteractionGroups::new(),
            ground_joint_groups: InteractionGroups::new(),
            constraints: Vec::new(),
        }
    }
}

pub(crate) struct PositionSolverPart {
    pub point_point_manifolds: Vec<ContactManifoldIndex>,
    pub plane_point_manifolds: Vec<ContactManifoldIndex>,
    pub point_point_ground_manifolds: Vec<ContactManifoldIndex>,
    pub plane_point_ground_manifolds: Vec<ContactManifoldIndex>,
    pub point_point_groups: InteractionGroups,
    pub plane_point_groups: InteractionGroups,
    pub point_point_ground_groups: InteractionGroups,
    pub plane_point_ground_groups: InteractionGroups,
    pub constraints: Vec<AnyPositionConstraint>,
}

impl PositionSolverPart {
    pub fn new() -> Self {
        Self {
            point_point_manifolds: Vec::new(),
            plane_point_manifolds: Vec::new(),
            point_point_ground_manifolds: Vec::new(),
            plane_point_ground_manifolds: Vec::new(),
            point_point_groups: InteractionGroups::new(),
            plane_point_groups: InteractionGroups::new(),
            point_point_ground_groups: InteractionGroups::new(),
            plane_point_ground_groups: InteractionGroups::new(),
            constraints: Vec::new(),
        }
    }
}

pub(crate) struct PositionSolver {
    positions: Vec<Isometry<f32>>,
    part: PositionSolverPart,
    joint_part: PositionSolverJointPart,
}

impl PositionSolver {
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
            part: PositionSolverPart::new(),
            joint_part: PositionSolverJointPart::new(),
        }
    }

    pub fn init_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) {
        self.part
            .init_constraints(island_id, params, bodies, manifolds, manifold_indices);
        self.joint_part.init_constraints(
            island_id,
            params,
            bodies,
            joints,
            joint_constraint_indices,
        );
    }

    pub fn solve_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &mut RigidBodySet,
    ) {
        self.positions.clear();
        self.positions.extend(
            bodies
                .iter_active_island(island_id)
                .map(|(_, b)| b.position),
        );

        for _ in 0..params.max_position_iterations {
            for constraint in &self.joint_part.constraints {
                constraint.solve(params, &mut self.positions)
            }

            for constraint in &self.part.constraints {
                constraint.solve(params, &mut self.positions)
            }
        }

        bodies.foreach_active_island_body_mut_internal(island_id, |_, rb| {
            rb.set_position(self.positions[rb.active_set_offset])
        });
    }
}

impl PositionSolverPart {
    pub fn init_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) {
        self.point_point_ground_manifolds.clear();
        self.plane_point_ground_manifolds.clear();
        self.point_point_manifolds.clear();
        self.plane_point_manifolds.clear();
        categorize_position_contacts(
            bodies,
            manifolds_all,
            manifold_indices,
            &mut self.point_point_ground_manifolds,
            &mut self.plane_point_ground_manifolds,
            &mut self.point_point_manifolds,
            &mut self.plane_point_manifolds,
        );

        self.point_point_groups.clear_groups();
        self.point_point_groups.group_manifolds(
            island_id,
            bodies,
            manifolds_all,
            &self.point_point_manifolds,
        );
        self.plane_point_groups.clear_groups();
        self.plane_point_groups.group_manifolds(
            island_id,
            bodies,
            manifolds_all,
            &self.plane_point_manifolds,
        );

        self.point_point_ground_groups.clear_groups();
        self.point_point_ground_groups.group_manifolds(
            island_id,
            bodies,
            manifolds_all,
            &self.point_point_ground_manifolds,
        );
        self.plane_point_ground_groups.clear_groups();
        self.plane_point_ground_groups.group_manifolds(
            island_id,
            bodies,
            manifolds_all,
            &self.plane_point_ground_manifolds,
        );

        self.constraints.clear();

        /*
         * Init non-ground contact constraints.
         */
        #[cfg(feature = "simd-is-enabled")]
        {
            compute_grouped_constraints(
                params,
                bodies,
                manifolds_all,
                &self.point_point_groups.grouped_interactions,
                &mut self.constraints,
            );
            compute_grouped_constraints(
                params,
                bodies,
                manifolds_all,
                &self.plane_point_groups.grouped_interactions,
                &mut self.constraints,
            );
        }
        compute_nongrouped_constraints(
            params,
            bodies,
            manifolds_all,
            &self.point_point_groups.nongrouped_interactions,
            &mut self.constraints,
        );
        compute_nongrouped_constraints(
            params,
            bodies,
            manifolds_all,
            &self.plane_point_groups.nongrouped_interactions,
            &mut self.constraints,
        );

        /*
         * Init ground contact constraints.
         */
        #[cfg(feature = "simd-is-enabled")]
        {
            compute_grouped_ground_constraints(
                params,
                bodies,
                manifolds_all,
                &self.point_point_ground_groups.grouped_interactions,
                &mut self.constraints,
            );
            compute_grouped_ground_constraints(
                params,
                bodies,
                manifolds_all,
                &self.plane_point_ground_groups.grouped_interactions,
                &mut self.constraints,
            );
        }
        compute_nongrouped_ground_constraints(
            params,
            bodies,
            manifolds_all,
            &self.point_point_ground_groups.nongrouped_interactions,
            &mut self.constraints,
        );
        compute_nongrouped_ground_constraints(
            params,
            bodies,
            manifolds_all,
            &self.plane_point_ground_groups.nongrouped_interactions,
            &mut self.constraints,
        );
    }
}

impl PositionSolverJointPart {
    pub fn init_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) {
        self.ground_joints.clear();
        self.nonground_joints.clear();
        categorize_joints(
            bodies,
            joints,
            joint_constraint_indices,
            &mut self.ground_joints,
            &mut self.nonground_joints,
        );

        self.nonground_joint_groups.clear_groups();
        self.nonground_joint_groups
            .group_joints(island_id, bodies, joints, &self.nonground_joints);

        self.ground_joint_groups.clear_groups();
        self.ground_joint_groups
            .group_joints(island_id, bodies, joints, &self.ground_joints);

        /*
         * Init ground joint constraints.
         */
        self.constraints.clear();
        compute_nongrouped_joint_ground_constraints(
            params,
            bodies,
            joints,
            &self.ground_joint_groups.nongrouped_interactions,
            &mut self.constraints,
        );

        #[cfg(feature = "simd-is-enabled")]
        {
            compute_grouped_joint_ground_constraints(
                params,
                bodies,
                joints,
                &self.ground_joint_groups.grouped_interactions,
                &mut self.constraints,
            );
        }

        /*
         * Init non-ground joint constraints.
         */
        compute_nongrouped_joint_constraints(
            params,
            bodies,
            joints,
            &self.nonground_joint_groups.nongrouped_interactions,
            &mut self.constraints,
        );

        #[cfg(feature = "simd-is-enabled")]
        {
            compute_grouped_joint_constraints(
                params,
                bodies,
                joints,
                &self.nonground_joint_groups.grouped_interactions,
                &mut self.constraints,
            );
        }
    }
}

fn compute_nongrouped_constraints(
    params: &IntegrationParameters,
    bodies: &RigidBodySet,
    manifolds_all: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    output: &mut Vec<AnyPositionConstraint>,
) {
    for manifold in manifold_indices.iter().map(|i| &manifolds_all[*i]) {
        PositionConstraint::generate(params, manifold, bodies, output, true)
    }
}

#[cfg(feature = "simd-is-enabled")]
fn compute_grouped_constraints(
    params: &IntegrationParameters,
    bodies: &RigidBodySet,
    manifolds_all: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    output: &mut Vec<AnyPositionConstraint>,
) {
    for manifolds_i in manifold_indices.chunks_exact(SIMD_WIDTH) {
        let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
        WPositionConstraint::generate(params, manifolds, bodies, output, true)
    }
}

fn compute_nongrouped_ground_constraints(
    params: &IntegrationParameters,
    bodies: &RigidBodySet,
    manifolds_all: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    output: &mut Vec<AnyPositionConstraint>,
) {
    for manifold in manifold_indices.iter().map(|i| &manifolds_all[*i]) {
        PositionGroundConstraint::generate(params, manifold, bodies, output, true)
    }
}

#[cfg(feature = "simd-is-enabled")]
fn compute_grouped_ground_constraints(
    params: &IntegrationParameters,
    bodies: &RigidBodySet,
    manifolds_all: &[&mut ContactManifold],
    manifold_indices: &[ContactManifoldIndex],
    output: &mut Vec<AnyPositionConstraint>,
) {
    for manifolds_i in manifold_indices.chunks_exact(SIMD_WIDTH) {
        let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
        WPositionGroundConstraint::generate(params, manifolds, bodies, output, true);
    }
}

fn compute_nongrouped_joint_ground_constraints(
    _params: &IntegrationParameters,
    bodies: &RigidBodySet,
    joints_all: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    output: &mut Vec<AnyJointPositionConstraint>,
) {
    for joint_i in joint_indices {
        let joint = &joints_all[*joint_i].weight;
        let pos_constraint = AnyJointPositionConstraint::from_joint_ground(joint, bodies);
        output.push(pos_constraint);
    }
}

#[cfg(feature = "simd-is-enabled")]
fn compute_grouped_joint_ground_constraints(
    _params: &IntegrationParameters,
    bodies: &RigidBodySet,
    joints_all: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    output: &mut Vec<AnyJointPositionConstraint>,
) {
    for joint_i in joint_indices.chunks_exact(SIMD_WIDTH) {
        let joints = array![|ii| &joints_all[joint_i[ii]].weight; SIMD_WIDTH];
        if let Some(pos_constraint) =
            AnyJointPositionConstraint::from_wide_joint_ground(joints, bodies)
        {
            output.push(pos_constraint);
        } else {
            for joint in joints.iter() {
                output.push(AnyJointPositionConstraint::from_joint_ground(
                    *joint, bodies,
                ))
            }
        }
    }
}

fn compute_nongrouped_joint_constraints(
    _params: &IntegrationParameters,
    bodies: &RigidBodySet,
    joints_all: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    output: &mut Vec<AnyJointPositionConstraint>,
) {
    for joint_i in joint_indices {
        let joint = &joints_all[*joint_i];
        let pos_constraint = AnyJointPositionConstraint::from_joint(&joint.weight, bodies);
        output.push(pos_constraint);
    }
}

#[cfg(feature = "simd-is-enabled")]
fn compute_grouped_joint_constraints(
    _params: &IntegrationParameters,
    bodies: &RigidBodySet,
    joints_all: &[JointGraphEdge],
    joint_indices: &[JointIndex],
    output: &mut Vec<AnyJointPositionConstraint>,
) {
    for joint_i in joint_indices.chunks_exact(SIMD_WIDTH) {
        let joints = array![|ii| &joints_all[joint_i[ii]].weight; SIMD_WIDTH];
        if let Some(pos_constraint) = AnyJointPositionConstraint::from_wide_joint(joints, bodies) {
            output.push(pos_constraint);
        } else {
            for joint in joints.iter() {
                output.push(AnyJointPositionConstraint::from_joint(*joint, bodies))
            }
        }
    }
}
