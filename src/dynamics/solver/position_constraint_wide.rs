use super::AnyPositionConstraint;
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::ContactManifold;
use crate::math::{
    AngularInertia, Isometry, Point, Real, Rotation, SimdReal, Translation, Vector,
    MAX_MANIFOLD_POINTS, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WDot};

use num::Zero;
use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

pub(crate) struct WPositionConstraint {
    pub rb1: [usize; SIMD_WIDTH],
    pub rb2: [usize; SIMD_WIDTH],
    // NOTE: the points are relative to the center of masses.
    pub local_p1: [Point<SimdReal>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<SimdReal>; MAX_MANIFOLD_POINTS],
    pub dists: [SimdReal; MAX_MANIFOLD_POINTS],
    pub local_n1: Vector<SimdReal>,
    pub im1: SimdReal,
    pub im2: SimdReal,
    pub ii1: AngularInertia<SimdReal>,
    pub ii2: AngularInertia<SimdReal>,
    pub erp: SimdReal,
    pub max_linear_correction: SimdReal,
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
        let rbs1 = array![|ii| bodies.get(manifolds[ii].data.body_pair.body1).unwrap(); SIMD_WIDTH];
        let rbs2 = array![|ii| bodies.get(manifolds[ii].data.body_pair.body2).unwrap(); SIMD_WIDTH];

        let im1 = SimdReal::from(array![|ii| rbs1[ii].effective_inv_mass; SIMD_WIDTH]);
        let sqrt_ii1: AngularInertia<SimdReal> = AngularInertia::from(
            array![|ii| rbs1[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );
        let im2 = SimdReal::from(array![|ii| rbs2[ii].effective_inv_mass; SIMD_WIDTH]);
        let sqrt_ii2: AngularInertia<SimdReal> = AngularInertia::from(
            array![|ii| rbs2[ii].effective_world_inv_inertia_sqrt; SIMD_WIDTH],
        );

        let pos1 = Isometry::from(array![|ii| rbs1[ii].position; SIMD_WIDTH]);
        let pos2 = Isometry::from(array![|ii| rbs2[ii].position; SIMD_WIDTH]);

        let local_n1 = pos1.inverse_transform_vector(&Vector::from(
            array![|ii| manifolds[ii].data.normal; SIMD_WIDTH],
        ));

        let rb1 = array![|ii| rbs1[ii].active_set_offset; SIMD_WIDTH];
        let rb2 = array![|ii| rbs2[ii].active_set_offset; SIMD_WIDTH];

        let num_active_contacts = manifolds[0].data.num_active_contacts();

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = array![|ii| &manifolds[ii].data.solver_contacts[l..]; SIMD_WIDTH];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WPositionConstraint {
                rb1,
                rb2,
                local_p1: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_p2: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_n1,
                dists: [SimdReal::zero(); MAX_MANIFOLD_POINTS],
                im1,
                im2,
                ii1: sqrt_ii1.squared(),
                ii2: sqrt_ii2.squared(),
                erp: SimdReal::splat(params.positionErp),
                max_linear_correction: SimdReal::splat(params.max_linear_correction),
                num_contacts: num_points as u8,
            };

            for i in 0..num_points {
                let point = Point::from(array![|ii| manifold_points[ii][i].point; SIMD_WIDTH]);
                let dist = SimdReal::from(array![|ii| manifold_points[ii][i].dist; SIMD_WIDTH]);
                constraint.local_p1[i] = pos1.inverse_transform_point(&point);
                constraint.local_p2[i] = pos2.inverse_transform_point(&point);
                constraint.dists[i] = dist;
            }

            if push {
                out_constraints.push(AnyPositionConstraint::GroupedNonGround(constraint));
            } else {
                out_constraints[manifolds[0].data.constraint_index + l / MAX_MANIFOLD_POINTS] =
                    AnyPositionConstraint::GroupedNonGround(constraint);
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos1 = Isometry::from(array![|ii| positions[self.rb1[ii]]; SIMD_WIDTH]);
        let mut pos2 = Isometry::from(array![|ii| positions[self.rb2[ii]]; SIMD_WIDTH]);
        let allowed_err = SimdReal::splat(params.allowed_linear_error);

        for k in 0..self.num_contacts as usize {
            let target_dist = -self.dists[k] - allowed_err;
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
                    .simd_clamp(-self.max_linear_correction, SimdReal::zero());
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
