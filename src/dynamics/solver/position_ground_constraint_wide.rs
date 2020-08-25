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

pub(crate) struct WPositionGroundConstraint {
    pub rb2: [usize; SIMD_WIDTH],
    // NOTE: the points are relative to the center of masses.
    pub p1: [Point<SimdFloat>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<SimdFloat>; MAX_MANIFOLD_POINTS],
    pub n1: Vector<SimdFloat>,
    pub radius: SimdFloat,
    pub im2: SimdFloat,
    pub ii2: AngularInertia<SimdFloat>,
    pub erp: SimdFloat,
    pub max_linear_correction: SimdFloat,
    pub num_contacts: u8,
}

impl WPositionGroundConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) {
        let mut rbs1 = array![|ii| bodies.get(manifolds[ii].body_pair.body1).unwrap(); SIMD_WIDTH];
        let mut rbs2 = array![|ii| bodies.get(manifolds[ii].body_pair.body2).unwrap(); SIMD_WIDTH];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if !rbs2[ii].is_dynamic() {
                flipped[ii] = true;
                std::mem::swap(&mut rbs1[ii], &mut rbs2[ii]);
            }
        }

        let im2 = SimdFloat::from(array![|ii| rbs2[ii].mass_properties.inv_mass; SIMD_WIDTH]);
        let sqrt_ii2: AngularInertia<SimdFloat> =
            AngularInertia::from(array![|ii| rbs2[ii].world_inv_inertia_sqrt; SIMD_WIDTH]);

        let local_n1 = Vector::from(
            array![|ii| if flipped[ii] { manifolds[ii].local_n2 } else { manifolds[ii].local_n1 }; SIMD_WIDTH],
        );
        let local_n2 = Vector::from(
            array![|ii| if flipped[ii] { manifolds[ii].local_n1 } else { manifolds[ii].local_n2 }; SIMD_WIDTH],
        );

        let radius1 = SimdFloat::from(array![|ii| manifolds[ii].kinematics.radius1; SIMD_WIDTH]);
        let radius2 = SimdFloat::from(array![|ii| manifolds[ii].kinematics.radius2; SIMD_WIDTH]);

        let position1 = Isometry::from(array![|ii| rbs1[ii].predicted_position; SIMD_WIDTH]);

        let rb2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let radius = radius1 + radius2 /*- SimdFloat::splat(params.allowed_linear_error)*/;

        let n1 = position1 * local_n1;

        for l in (0..manifolds[0].num_active_contacts()).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = array![|ii| &manifolds[ii].active_contacts()[l..]; SIMD_WIDTH];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WPositionGroundConstraint {
                rb2,
                p1: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_p2: [Point::origin(); MAX_MANIFOLD_POINTS],
                n1,
                radius,
                im2,
                ii2: sqrt_ii2.squared(),
                erp: SimdFloat::splat(params.erp),
                max_linear_correction: SimdFloat::splat(params.max_linear_correction),
                num_contacts: num_points as u8,
            };

            for i in 0..num_points {
                let local_p1 = Point::from(
                    array![|ii| if flipped[ii] { manifold_points[ii][i].local_p2 } else { manifold_points[ii][i].local_p1 }; SIMD_WIDTH],
                );
                let local_p2 = Point::from(
                    array![|ii| if flipped[ii] { manifold_points[ii][i].local_p1 } else { manifold_points[ii][i].local_p2 }; SIMD_WIDTH],
                );

                constraint.p1[i] = position1 * local_p1 - n1 * radius1;
                constraint.local_p2[i] = local_p2 - local_n2 * radius2;
            }

            if push {
                if manifolds[0].kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints
                        .push(AnyPositionConstraint::GroupedPointPointGround(constraint));
                } else {
                    out_constraints
                        .push(AnyPositionConstraint::GroupedPlanePointGround(constraint));
                }
            } else {
                if manifolds[0].kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints[manifolds[0].constraint_index + l / MAX_MANIFOLD_POINTS] =
                        AnyPositionConstraint::GroupedPointPointGround(constraint);
                } else {
                    out_constraints[manifolds[0].constraint_index + l / MAX_MANIFOLD_POINTS] =
                        AnyPositionConstraint::GroupedPlanePointGround(constraint);
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
        let mut pos2 = Isometry::from(array![|ii| positions[self.rb2[ii]]; SIMD_WIDTH]);
        let allowed_err = SimdFloat::splat(params.allowed_linear_error);
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let p1 = self.p1[k];
            let p2 = pos2 * self.local_p2[k];

            let dpos = p2 - p1;
            let sqdist = dpos.norm_squared();

            if sqdist.simd_lt(target_dist * target_dist).any() {
                let dist = sqdist.simd_sqrt();
                let n = dpos / dist;
                let err = ((dist - target_dist) * self.erp)
                    .simd_clamp(-self.max_linear_correction, SimdFloat::zero());
                let dp2 = p2.coords - pos2.translation.vector;
                let gcross2 = -dp2.gcross(n);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r = self.im2 + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                pos2.translation = Translation::from(n * (-impulse * self.im2)) * pos2.translation;
                pos2.rotation = Rotation::new(ii_gcross2 * impulse) * pos2.rotation;
            }
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
        let mut pos2 = Isometry::from(array![|ii| positions[self.rb2[ii]]; SIMD_WIDTH]);
        let allowed_err = SimdFloat::splat(params.allowed_linear_error);
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let n1 = self.n1;
            let p1 = self.p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;
            let dist = dpos.dot(&n1);

            // NOTE: this condition does not seem to be useful perfomancewise?
            if dist.simd_lt(target_dist).any() {
                let err = ((dist - target_dist) * self.erp)
                    .simd_clamp(-self.max_linear_correction, SimdFloat::zero());
                let dp2 = p2.coords - pos2.translation.vector;

                let gcross2 = -dp2.gcross(n1);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r = self.im2 + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                pos2.translation = Translation::from(n1 * (-impulse * self.im2)) * pos2.translation;
                pos2.rotation = Rotation::new(ii_gcross2 * impulse) * pos2.rotation;
            }
        }

        for ii in 0..SIMD_WIDTH {
            positions[self.rb2[ii]] = pos2.extract(ii);
        }
    }
}
