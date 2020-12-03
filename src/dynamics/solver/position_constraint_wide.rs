use super::AnyPositionConstraint;
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, KinematicsCategory};
use crate::math::{
    AngularInertia, Isometry, Point, Rotation, SimdFloat, Translation, Vector, MAX_MANIFOLD_POINTS,
    SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WDot};

use num::Zero;
use simba::simd::{SimdBool as _, SimdComplexField, SimdPartialOrd, SimdValue};

pub(crate) struct WPositionConstraint {
    pub rb1: [usize; SIMD_WIDTH],
    pub rb2: [usize; SIMD_WIDTH],
    // NOTE: the points are relative to the center of masses.
    pub local_p1: [Point<SimdFloat>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<SimdFloat>; MAX_MANIFOLD_POINTS],
    pub local_n1: Vector<SimdFloat>,
    pub radius: SimdFloat,
    pub im1: SimdFloat,
    pub im2: SimdFloat,
    pub ii1: AngularInertia<SimdFloat>,
    pub ii2: AngularInertia<SimdFloat>,
    pub erp: SimdFloat,
    pub max_linear_correction: SimdFloat,
    pub num_contacts: u8,
}

impl WPositionConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) {
        let rbs1 = array![|ii| bodies.get(manifolds[ii].body_pair.body1).unwrap(); SIMD_WIDTH];
        let rbs2 = array![|ii| bodies.get(manifolds[ii].body_pair.body2).unwrap(); SIMD_WIDTH];

        let im1 = SimdFloat::from(array![|ii| rbs1[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let sqrt_ii1: AngularInertia<SimdFloat> =
            AngularInertia::from(array![|ii| rbs1[ii].world_inv_inertia_sqrt; SIMD_WIDTH]);
        let rb_pos1 = Isometry::from(array![|ii| *rbs1[ii].position(); SIMD_WIDTH]);

        let im2 = SimdFloat::from(array![|ii| rbs2[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let sqrt_ii2: AngularInertia<SimdFloat> =
            AngularInertia::from(array![|ii| rbs2[ii].world_inv_inertia_sqrt; SIMD_WIDTH]);
        let rb_pos2 = Isometry::from(array![|ii| *rbs2[ii].position(); SIMD_WIDTH]);

        let local_n1 = Vector::from(array![|ii| manifolds[ii].local_n1; SIMD_WIDTH]);
        let local_n2 = Vector::from(array![|ii| manifolds[ii].local_n2; SIMD_WIDTH]);

        let radius1 = SimdFloat::from(array![|ii| manifolds[ii].kinematics.radius1; SIMD_WIDTH]);
        let radius2 = SimdFloat::from(array![|ii| manifolds[ii].kinematics.radius2; SIMD_WIDTH]);

        let coll_pos1 = Isometry::from(array![|ii| manifolds[ii].position1; SIMD_WIDTH]);
        let coll_pos2 = Isometry::from(array![|ii| manifolds[ii].position2; SIMD_WIDTH]);

        let delta1 = rb_pos1.inverse() * coll_pos1;
        let delta2 = rb_pos2.inverse() * coll_pos2;

        let rb1 = array![|ii| rbs1[ii].active_set_offset; SIMD_WIDTH];
        let rb2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let radius = radius1 + radius2 /*- SimdFloat::splat(params.allowed_linear_error)*/;

        for l in (0..manifolds[0].num_active_contacts()).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = array![|ii| &manifolds[ii].active_contacts()[l..]; SIMD_WIDTH];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WPositionConstraint {
                rb1,
                rb2,
                local_p1: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_p2: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_n1,
                radius,
                im1,
                im2,
                ii1: sqrt_ii1.squared(),
                ii2: sqrt_ii2.squared(),
                erp: SimdFloat::splat(params.erp),
                max_linear_correction: SimdFloat::splat(params.max_linear_correction),
                num_contacts: num_points as u8,
            };

            let shift1 = local_n1 * -radius1;
            let shift2 = local_n2 * -radius2;

            for i in 0..num_points {
                let local_p1 =
                    Point::from(array![|ii| manifold_points[ii][i].local_p1; SIMD_WIDTH]);
                let local_p2 =
                    Point::from(array![|ii| manifold_points[ii][i].local_p2; SIMD_WIDTH]);

                constraint.local_p1[i] = delta1 * (local_p1 + shift1);
                constraint.local_p2[i] = delta2 * (local_p2 + shift2);
            }

            if push {
                if manifolds[0].kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints.push(AnyPositionConstraint::GroupedPointPoint(constraint));
                } else {
                    out_constraints.push(AnyPositionConstraint::GroupedPlanePoint(constraint));
                }
            } else {
                if manifolds[0].kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints[manifolds[0].constraint_index + l / MAX_MANIFOLD_POINTS] =
                        AnyPositionConstraint::GroupedPointPoint(constraint);
                } else {
                    out_constraints[manifolds[0].constraint_index + l / MAX_MANIFOLD_POINTS] =
                        AnyPositionConstraint::GroupedPlanePoint(constraint);
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
        let mut pos1 = Isometry::from(array![|ii| positions[self.rb1[ii]]; SIMD_WIDTH]);
        let mut pos2 = Isometry::from(array![|ii| positions[self.rb2[ii]]; SIMD_WIDTH]);
        let allowed_err = SimdFloat::splat(params.allowed_linear_error);
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let p1 = pos1 * self.local_p1[k];
            let p2 = pos2 * self.local_p2[k];

            let dpos = p2 - p1;
            let sqdist = dpos.norm_squared();

            if sqdist.simd_lt(target_dist * target_dist).any() {
                let dist = sqdist.simd_sqrt();
                let n = dpos / dist;
                let err = ((dist - target_dist) * self.erp)
                    .simd_clamp(-self.max_linear_correction, SimdFloat::zero());
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
                pos1.translation = Translation::from(n * (impulse * self.im1)) * pos1.translation;
                pos1.rotation = Rotation::new(ii_gcross1 * impulse) * pos1.rotation;
                pos2.translation = Translation::from(n * (-impulse * self.im2)) * pos2.translation;
                pos2.rotation = Rotation::new(ii_gcross2 * impulse) * pos2.rotation;
            }
        }

        for ii in 0..SIMD_WIDTH {
            positions[self.rb1[ii]] = pos1.extract(ii);
        }

        for ii in 0..SIMD_WIDTH {
            positions[self.rb2[ii]] = pos2.extract(ii);
        }
    }

    pub fn solve_plane_point(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<f32>],
    ) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos1 = Isometry::from(array![|ii| positions[self.rb1[ii]]; SIMD_WIDTH]);
        let mut pos2 = Isometry::from(array![|ii| positions[self.rb2[ii]]; SIMD_WIDTH]);
        let allowed_err = SimdFloat::splat(params.allowed_linear_error);
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let n1 = pos1 * self.local_n1;
            let p1 = pos1 * self.local_p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;
            let dist = dpos.dot(&n1);

            // NOTE: this condition does not seem to be useful perfomancewise?
            if dist.simd_lt(target_dist).any() {
                // NOTE: only works for the point-point case.
                let p1 = p2 - n1 * dist;
                let err = ((dist - target_dist) * self.erp)
                    .simd_clamp(-self.max_linear_correction, SimdFloat::zero());
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
                pos1.translation = Translation::from(n1 * (impulse * self.im1)) * pos1.translation;
                pos1.rotation = Rotation::new(ii_gcross1 * impulse) * pos1.rotation;
                pos2.translation = Translation::from(n1 * (-impulse * self.im2)) * pos2.translation;
                pos2.rotation = Rotation::new(ii_gcross2 * impulse) * pos2.rotation;
            }
        }

        for ii in 0..SIMD_WIDTH {
            positions[self.rb1[ii]] = pos1.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            positions[self.rb2[ii]] = pos2.extract(ii);
        }
    }
}
