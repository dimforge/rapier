use crate::dynamics::solver::PositionGroundConstraint;
#[cfg(feature = "simd-is-enabled")]
use crate::dynamics::solver::{WPositionConstraint, WPositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, KinematicsCategory};
use crate::math::{
    AngularInertia, Isometry, Point, Rotation, Translation, Vector, MAX_MANIFOLD_POINTS,
};
use crate::utils::{WAngularInertia, WCross, WDot};

pub(crate) enum AnyPositionConstraint {
    #[cfg(feature = "simd-is-enabled")]
    GroupedPointPointGround(WPositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedPlanePointGround(WPositionGroundConstraint),
    NongroupedPointPointGround(PositionGroundConstraint),
    NongroupedPlanePointGround(PositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedPointPoint(WPositionConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedPlanePoint(WPositionConstraint),
    NongroupedPointPoint(PositionConstraint),
    NongroupedPlanePoint(PositionConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyPositionConstraint {
    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<f32>]) {
        match self {
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedPointPointGround(c) => {
                c.solve_point_point(params, positions)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedPlanePointGround(c) => {
                c.solve_plane_point(params, positions)
            }
            AnyPositionConstraint::NongroupedPointPointGround(c) => {
                c.solve_point_point(params, positions)
            }
            AnyPositionConstraint::NongroupedPlanePointGround(c) => {
                c.solve_plane_point(params, positions)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedPointPoint(c) => c.solve_point_point(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedPlanePoint(c) => c.solve_plane_point(params, positions),
            AnyPositionConstraint::NongroupedPointPoint(c) => {
                c.solve_point_point(params, positions)
            }
            AnyPositionConstraint::NongroupedPlanePoint(c) => {
                c.solve_plane_point(params, positions)
            }
            AnyPositionConstraint::Empty => unreachable!(),
        }
    }
}

pub(crate) struct PositionConstraint {
    pub rb1: usize,
    pub rb2: usize,
    // NOTE: the points are relative to the center of masses.
    pub local_p1: [Point<f32>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<f32>; MAX_MANIFOLD_POINTS],
    pub local_n1: Vector<f32>,
    pub num_contacts: u8,
    pub radius: f32,
    pub im1: f32,
    pub im2: f32,
    pub ii1: AngularInertia<f32>,
    pub ii2: AngularInertia<f32>,
    pub erp: f32,
    pub max_linear_correction: f32,
}

impl PositionConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints(manifold: &ContactManifold) -> usize {
        let rest = manifold.num_active_contacts() % MAX_MANIFOLD_POINTS != 0;
        manifold.num_active_contacts() / MAX_MANIFOLD_POINTS + rest as usize
    }

    pub fn generate(
        params: &IntegrationParameters,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) {
        let rb1 = &bodies[manifold.body_pair.body1];
        let rb2 = &bodies[manifold.body_pair.body2];
        let shift1 = manifold.local_n1 * -manifold.kinematics.radius1;
        let shift2 = manifold.local_n2 * -manifold.kinematics.radius2;
        let radius =
            manifold.kinematics.radius1 + manifold.kinematics.radius2 /*- params.allowed_linear_error*/;
        let delta1 = rb1.position.inverse() * manifold.position1;
        let delta2 = rb2.position.inverse() * manifold.position2;

        for (l, manifold_points) in manifold
            .active_contacts()
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let mut local_p1 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut local_p2 = [Point::origin(); MAX_MANIFOLD_POINTS];

            for l in 0..manifold_points.len() {
                local_p1[l] = delta1 * (manifold_points[l].local_p1 + shift1);
                local_p2[l] = delta2 * (manifold_points[l].local_p2 + shift2);
            }

            let constraint = PositionConstraint {
                rb1: rb1.active_set_offset,
                rb2: rb2.active_set_offset,
                local_p1,
                local_p2,
                local_n1: manifold.local_n1,
                radius,
                im1: rb1.mass_properties.inv_mass,
                im2: rb2.mass_properties.inv_mass,
                ii1: rb1.world_inv_inertia_sqrt.squared(),
                ii2: rb2.world_inv_inertia_sqrt.squared(),
                num_contacts: manifold_points.len() as u8,
                erp: params.erp,
                max_linear_correction: params.max_linear_correction,
            };

            if push {
                if manifold.kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints.push(AnyPositionConstraint::NongroupedPointPoint(constraint));
                } else {
                    out_constraints.push(AnyPositionConstraint::NongroupedPlanePoint(constraint));
                }
            } else {
                if manifold.kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints[manifold.constraint_index + l] =
                        AnyPositionConstraint::NongroupedPointPoint(constraint);
                } else {
                    out_constraints[manifold.constraint_index + l] =
                        AnyPositionConstraint::NongroupedPlanePoint(constraint);
                }
            }
        }
    }

    pub fn solve_point_point(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<f32>],
    ) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos1 = positions[self.rb1];
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let p1 = pos1 * self.local_p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;

            let sqdist = dpos.norm_squared();

            // NOTE: only works for the point-point case.
            if sqdist < target_dist * target_dist {
                let dist = sqdist.sqrt();
                let n = dpos / dist;
                let err = ((dist - target_dist) * self.erp).max(-self.max_linear_correction);
                let dp1 = p1.coords - pos1.translation.vector;
                let dp2 = p2.coords - pos2.translation.vector;

                let gcross1 = dp1.gcross(n);
                let gcross2 = -dp2.gcross(n);
                let ii_gcross1 = self.ii1.transform_vector(gcross1);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r =
                    self.im1 + self.im2 + gcross1.gdot(ii_gcross1) + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                let tra1 = Translation::from(n * (impulse * self.im1));
                let tra2 = Translation::from(n * (-impulse * self.im2));
                let rot1 = Rotation::new(ii_gcross1 * impulse);
                let rot2 = Rotation::new(ii_gcross2 * impulse);

                pos1 = Isometry::from_parts(tra1 * pos1.translation, rot1 * pos1.rotation);
                pos2 = Isometry::from_parts(tra2 * pos2.translation, rot2 * pos2.rotation);
            }
        }

        positions[self.rb1] = pos1;
        positions[self.rb2] = pos2;
    }

    pub fn solve_plane_point(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<f32>],
    ) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos1 = positions[self.rb1];
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let n1 = pos1 * self.local_n1;
            let p1 = pos1 * self.local_p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;
            let dist = dpos.dot(&n1);

            if dist < target_dist {
                let p1 = p2 - n1 * dist;
                let err = ((dist - target_dist) * self.erp).max(-self.max_linear_correction);
                let dp1 = p1.coords - pos1.translation.vector;
                let dp2 = p2.coords - pos2.translation.vector;

                let gcross1 = dp1.gcross(n1);
                let gcross2 = -dp2.gcross(n1);
                let ii_gcross1 = self.ii1.transform_vector(gcross1);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r =
                    self.im1 + self.im2 + gcross1.gdot(ii_gcross1) + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                let tra1 = Translation::from(n1 * (impulse * self.im1));
                let tra2 = Translation::from(n1 * (-impulse * self.im2));
                let rot1 = Rotation::new(ii_gcross1 * impulse);
                let rot2 = Rotation::new(ii_gcross2 * impulse);

                pos1 = Isometry::from_parts(tra1 * pos1.translation, rot1 * pos1.rotation);
                pos2 = Isometry::from_parts(tra2 * pos2.translation, rot2 * pos2.rotation);
            }
        }

        positions[self.rb1] = pos1;
        positions[self.rb2] = pos2;
    }
}
