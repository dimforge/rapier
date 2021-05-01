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

pub(crate) struct WPositionGroundConstraint {
    pub rb2: [usize; SIMD_WIDTH],
    // NOTE: the points are relative to the center of masses.
    pub p1: [Point<SimdReal>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<SimdReal>; MAX_MANIFOLD_POINTS],
    pub dists: [SimdReal; MAX_MANIFOLD_POINTS],
    pub n1: Vector<SimdReal>,
    pub im2: SimdReal,
    pub ii2: AngularInertia<SimdReal>,
    pub erp: SimdReal,
    pub max_linear_correction: SimdReal,
    pub num_contacts: u8,
}

impl WPositionGroundConstraint {
    pub fn generate<Bodies>(
        params: &IntegrationParameters,
        manifolds: [&ContactManifold; SIMD_WIDTH],
        bodies: &Bodies,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) where
        Bodies: ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>,
    {
        let mut handles1 = gather![|ii| manifolds[ii].data.rigid_body1];
        let mut handles2 = gather![|ii| manifolds[ii].data.rigid_body2];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if manifolds[ii].data.relative_dominance < 0 {
                flipped[ii] = true;
                std::mem::swap(&mut handles1[ii], &mut handles2[ii]);
            }
        }

        let poss2: [&RigidBodyPosition; SIMD_WIDTH] =
            gather![|ii| bodies.index(handles2[ii].unwrap().0)];
        let ids2: [&RigidBodyIds; SIMD_WIDTH] = gather![|ii| bodies.index(handles2[ii].unwrap().0)];
        let mprops2: [&RigidBodyMassProps; SIMD_WIDTH] =
            gather![|ii| bodies.index(handles2[ii].unwrap().0)];

        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let sqrt_ii2: AngularInertia<SimdReal> =
            AngularInertia::from(gather![|ii| mprops2[ii].effective_world_inv_inertia_sqrt]);

        let n1 = Vector::from(gather![|ii| if flipped[ii] {
            -manifolds[ii].data.normal
        } else {
            manifolds[ii].data.normal
        }]);

        let pos2 = Isometry::from(gather![|ii| poss2[ii].position]);
        let rb2 = gather![|ii| ids2[ii].active_set_offset];

        let num_active_contacts = manifolds[0].data.num_active_contacts();

        for l in (0..num_active_contacts).step_by(MAX_MANIFOLD_POINTS) {
            let manifold_points = gather![|ii| &manifolds[ii].data.solver_contacts[l..]];
            let num_points = manifold_points[0].len().min(MAX_MANIFOLD_POINTS);

            let mut constraint = WPositionGroundConstraint {
                rb2,
                p1: [Point::origin(); MAX_MANIFOLD_POINTS],
                local_p2: [Point::origin(); MAX_MANIFOLD_POINTS],
                n1,
                dists: [SimdReal::zero(); MAX_MANIFOLD_POINTS],
                im2,
                ii2: sqrt_ii2.squared(),
                erp: SimdReal::splat(params.erp),
                max_linear_correction: SimdReal::splat(params.max_linear_correction),
                num_contacts: num_points as u8,
            };

            for i in 0..num_points {
                let point = Point::from(gather![|ii| manifold_points[ii][i].point]);
                let dist = SimdReal::from(gather![|ii| manifold_points[ii][i].dist]);
                constraint.p1[i] = point;
                constraint.local_p2[i] = pos2.inverse_transform_point(&point);
                constraint.dists[i] = dist;
            }

            if push {
                out_constraints.push(AnyPositionConstraint::GroupedGround(constraint));
            } else {
                out_constraints[manifolds[0].data.constraint_index + l / MAX_MANIFOLD_POINTS] =
                    AnyPositionConstraint::GroupedGround(constraint);
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos2 = Isometry::from(gather![|ii| positions[self.rb2[ii]]]);
        let allowed_err = SimdReal::splat(params.allowed_linear_error);

        for k in 0..self.num_contacts as usize {
            let target_dist = -self.dists[k] - allowed_err;
            let n1 = self.n1;
            let p1 = self.p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;
            let dist = dpos.dot(&n1);

            // NOTE: this condition does not seem to be useful perfomancewise?
            if dist.simd_lt(target_dist).any() {
                let err = ((dist - target_dist) * self.erp)
                    .simd_clamp(-self.max_linear_correction, SimdReal::zero());
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
