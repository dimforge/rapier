use crate::data::ComponentSet;
use crate::dynamics::solver::PositionGroundConstraint;
#[cfg(feature = "simd-is-enabled")]
use crate::dynamics::solver::{WPositionConstraint, WPositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition};
use crate::geometry::ContactManifold;
use crate::math::{
    AngularInertia, Isometry, Point, Real, Rotation, Translation, Vector, MAX_MANIFOLD_POINTS,
};
use crate::utils::{WAngularInertia, WCross, WDot};

pub(crate) enum AnyPositionConstraint {
    #[cfg(feature = "simd-is-enabled")]
    GroupedGround(WPositionGroundConstraint),
    NonGroupedGround(PositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    GroupedNonGround(WPositionConstraint),
    NonGroupedNonGround(PositionConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyPositionConstraint {
    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        match self {
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedGround(c) => c.solve(params, positions),
            AnyPositionConstraint::NonGroupedGround(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyPositionConstraint::GroupedNonGround(c) => c.solve(params, positions),
            AnyPositionConstraint::NonGroupedNonGround(c) => c.solve(params, positions),
            AnyPositionConstraint::Empty => unreachable!(),
        }
    }
}

pub(crate) struct PositionConstraint {
    pub rb1: usize,
    pub rb2: usize,
    // NOTE: the points are relative to the center of masses.
    pub local_p1: [Point<Real>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<Real>; MAX_MANIFOLD_POINTS],
    pub dists: [Real; MAX_MANIFOLD_POINTS],
    pub local_n1: Vector<Real>,
    pub num_contacts: u8,
    pub im1: Real,
    pub im2: Real,
    pub ii1: AngularInertia<Real>,
    pub ii2: AngularInertia<Real>,
    pub erp: Real,
    pub max_linear_correction: Real,
}

impl PositionConstraint {
    pub fn generate<Bodies>(
        params: &IntegrationParameters,
        manifold: &ContactManifold,
        bodies: &Bodies,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let handle1 = manifold.data.rigid_body1.unwrap();
        let handle2 = manifold.data.rigid_body2.unwrap();

        let ids1: &RigidBodyIds = bodies.index(handle1.0);
        let ids2: &RigidBodyIds = bodies.index(handle2.0);
        let poss1: &RigidBodyPosition = bodies.index(handle1.0);
        let poss2: &RigidBodyPosition = bodies.index(handle2.0);
        let mprops1: &RigidBodyMassProps = bodies.index(handle1.0);
        let mprops2: &RigidBodyMassProps = bodies.index(handle2.0);

        for (l, manifold_points) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let mut local_p1 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut local_p2 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut dists = [0.0; MAX_MANIFOLD_POINTS];

            for l in 0..manifold_points.len() {
                local_p1[l] = poss1
                    .position
                    .inverse_transform_point(&manifold_points[l].point);
                local_p2[l] = poss2
                    .position
                    .inverse_transform_point(&manifold_points[l].point);
                dists[l] = manifold_points[l].dist;
            }

            let constraint = PositionConstraint {
                rb1: ids1.active_set_offset,
                rb2: ids2.active_set_offset,
                local_p1,
                local_p2,
                local_n1: poss1
                    .position
                    .inverse_transform_vector(&manifold.data.normal),
                dists,
                im1: mprops1.effective_inv_mass,
                im2: mprops2.effective_inv_mass,
                ii1: mprops1.effective_world_inv_inertia_sqrt.squared(),
                ii2: mprops2.effective_world_inv_inertia_sqrt.squared(),
                num_contacts: manifold_points.len() as u8,
                erp: params.erp,
                max_linear_correction: params.max_linear_correction,
            };

            if push {
                out_constraints.push(AnyPositionConstraint::NonGroupedNonGround(constraint));
            } else {
                out_constraints[manifold.data.constraint_index + l] =
                    AnyPositionConstraint::NonGroupedNonGround(constraint);
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos1 = positions[self.rb1];
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;

        for k in 0..self.num_contacts as usize {
            let target_dist = -self.dists[k] - allowed_err;
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
