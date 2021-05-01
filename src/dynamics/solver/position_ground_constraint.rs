use super::AnyPositionConstraint;
use crate::data::{BundleSet, ComponentSet};
use crate::dynamics::{IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition};
use crate::geometry::ContactManifold;
use crate::math::{
    AngularInertia, Isometry, Point, Real, Rotation, Translation, Vector, MAX_MANIFOLD_POINTS,
};
use crate::utils::{WAngularInertia, WCross, WDot};

pub(crate) struct PositionGroundConstraint {
    pub rb2: usize,
    // NOTE: the points are relative to the center of masses.
    pub p1: [Point<Real>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<Real>; MAX_MANIFOLD_POINTS],
    pub dists: [Real; MAX_MANIFOLD_POINTS],
    pub n1: Vector<Real>,
    pub num_contacts: u8,
    pub im2: Real,
    pub ii2: AngularInertia<Real>,
    pub erp: Real,
    pub max_linear_correction: Real,
}

impl PositionGroundConstraint {
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
        let flip = manifold.data.relative_dominance < 0;

        let (handle2, n1) = if flip {
            (manifold.data.rigid_body1.unwrap(), -manifold.data.normal)
        } else {
            (manifold.data.rigid_body2.unwrap(), manifold.data.normal)
        };

        let (ids2, poss2, mprops2): (&RigidBodyIds, &RigidBodyPosition, &RigidBodyMassProps) =
            bodies.index_bundle(handle2.0);

        for (l, manifold_contacts) in manifold
            .data
            .solver_contacts
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let mut p1 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut local_p2 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut dists = [0.0; MAX_MANIFOLD_POINTS];

            for k in 0..manifold_contacts.len() {
                p1[k] = manifold_contacts[k].point;
                local_p2[k] = poss2
                    .position
                    .inverse_transform_point(&manifold_contacts[k].point);
                dists[k] = manifold_contacts[k].dist;
            }

            let constraint = PositionGroundConstraint {
                rb2: ids2.active_set_offset,
                p1,
                local_p2,
                n1,
                dists,
                im2: mprops2.effective_inv_mass,
                ii2: mprops2.effective_world_inv_inertia_sqrt.squared(),
                num_contacts: manifold_contacts.len() as u8,
                erp: params.erp,
                max_linear_correction: params.max_linear_correction,
            };

            if push {
                out_constraints.push(AnyPositionConstraint::NonGroupedGround(constraint));
            } else {
                out_constraints[manifold.data.constraint_index + l] =
                    AnyPositionConstraint::NonGroupedGround(constraint);
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;

        for k in 0..self.num_contacts as usize {
            let target_dist = -self.dists[k] - allowed_err;
            let n1 = self.n1;
            let p1 = self.p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;
            let dist = dpos.dot(&n1);

            if dist < target_dist {
                let err = ((dist - target_dist) * self.erp).max(-self.max_linear_correction);
                let dp2 = p2.coords - pos2.translation.vector;

                let gcross2 = -dp2.gcross(n1);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r = self.im2 + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                let tra2 = Translation::from(n1 * (-impulse * self.im2));
                let rot2 = Rotation::new(ii_gcross2 * impulse);
                pos2 = Isometry::from_parts(tra2 * pos2.translation, rot2 * pos2.rotation);
            }
        }

        positions[self.rb2] = pos2;
    }
}
