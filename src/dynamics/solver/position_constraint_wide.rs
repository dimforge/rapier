use super::AnyPositionConstraint;
use crate::dynamics::{IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition};
use crate::geometry::ContactManifold;
use crate::math::{
    AngularInertia, Isometry, Point, Real, Rotation, SimdReal, Translation, Vector,
    MAX_MANIFOLD_POINTS, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross, WDot};

use crate::data::ComponentSet;
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
    pub fn generate<Bodies>(
        params: &IntegrationParameters,
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &Bodies,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let handles1 = gather![|ii| manifolds[ii].data.rigid_body1.unwrap()];
        let handles2 = gather![|ii| manifolds[ii].data.rigid_body2.unwrap()];

        let poss1: [&RigidBodyPosition; SIMD_WIDTH] = gather![|ii| bodies.index(handles1[ii].0)];
        let poss2: [&RigidBodyPosition; SIMD_WIDTH] = gather![|ii| bodies.index(handles2[ii].0)];
        let ids1: [&RigidBodyIds; SIMD_WIDTH] = gather![|ii| bodies.index(handles1[ii].0)];
        let ids2: [&RigidBodyIds; SIMD_WIDTH] = gather![|ii| bodies.index(handles2[ii].0)];
        let mprops1: [&RigidBodyMassProps; SIMD_WIDTH] = gather![|ii| bodies.index(handles1[ii].0)];
        let mprops2: [&RigidBodyMassProps; SIMD_WIDTH] = gather![|ii| bodies.index(handles2[ii].0)];

        let im1 = SimdReal::from(gather![|ii| mprops1[ii].effective_inv_mass]);
        let sqrt_ii1: AngularInertia<SimdReal> =
            AngularInertia::from(gather![|ii| mprops1[ii].effective_world_inv_inertia_sqrt]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let sqrt_ii2: AngularInertia<SimdReal> =
            AngularInertia::from(gather![|ii| mprops2[ii].effective_world_inv_inertia_sqrt]);

        let pos1 = Isometry::from(gather![|ii| poss1[ii].position]);
        let pos2 = Isometry::from(gather![|ii| poss2[ii].position]);

        let local_n1 =
            pos1.inverse_transform_vector(&Vector::from(gather![|ii| manifolds[ii].data.normal]));

        let rb1 = gather![|ii| ids1[ii].active_set_offset];
        let rb2 = gather![|ii| ids2[ii].active_set_offset];

        let num_active_contacts = manifolds[0].data.num_active_contacts();

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = gather![|ii| &manifolds[ii].data.solver_contacts[l..]];
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
                erp: SimdReal::splat(params.erp),
                max_linear_correction: SimdReal::splat(params.max_linear_correction),
                num_contacts: num_points as u8,
            };

            for i in 0..num_points {
                let point = Point::from(gather![|ii| manifold_points[ii][i].point]);
                let dist = SimdReal::from(gather![|ii| manifold_points[ii][i].dist]);
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
        let mut pos1 = Isometry::from(gather![|ii| positions[self.rb1[ii]]]);
        let mut pos2 = Isometry::from(gather![|ii| positions[self.rb2[ii]]]);
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
