use super::AnyPositionConstraint;
use crate::dynamics::{IntegrationParameters, RigidBodySet};
use crate::geometry::{ContactManifold, KinematicsCategory};
use crate::math::{
    AngularInertia, Isometry, Point, Rotation, Translation, Vector, MAX_MANIFOLD_POINTS,
};
use crate::utils::{WAngularInertia, WCross, WDot};

pub(crate) struct PositionGroundConstraint {
    pub rb2: usize,
    // NOTE: the points are relative to the center of masses.
    pub p1: [Point<f32>; MAX_MANIFOLD_POINTS],
    pub local_p2: [Point<f32>; MAX_MANIFOLD_POINTS],
    pub n1: Vector<f32>,
    pub num_contacts: u8,
    pub radius: f32,
    pub im2: f32,
    pub ii2: AngularInertia<f32>,
    pub erp: f32,
    pub max_linear_correction: f32,
}

impl PositionGroundConstraint {
    pub fn generate(
        params: &IntegrationParameters,
        manifold: &ContactManifold,
        bodies: &RigidBodySet,
        out_constraints: &mut Vec<AnyPositionConstraint>,
        push: bool,
    ) {
        if manifold.is_bouncing() {
            return; // let the bounce take care of the overlap
        }

        let mut rb1 = &bodies[manifold.body_pair.body1];
        let mut rb2 = &bodies[manifold.body_pair.body2];
        let flip = !rb2.is_dynamic();

        let local_n1;
        let local_n2;
        let delta1;
        let delta2;

        if flip {
            std::mem::swap(&mut rb1, &mut rb2);
            local_n1 = manifold.local_n2;
            local_n2 = manifold.local_n1;
            delta1 = &manifold.delta2;
            delta2 = &manifold.delta1;
        } else {
            local_n1 = manifold.local_n1;
            local_n2 = manifold.local_n2;
            delta1 = &manifold.delta1;
            delta2 = &manifold.delta2;
        };

        let coll_pos1 = rb1.position * delta1;
        let shift1 = local_n1 * -manifold.kinematics.radius1;
        let shift2 = local_n2 * -manifold.kinematics.radius2;
        let n1 = coll_pos1 * local_n1;
        let radius =
            manifold.kinematics.radius1 + manifold.kinematics.radius2 /* - params.allowed_linear_error */;

        for (l, manifold_contacts) in manifold
            .active_contacts()
            .chunks(MAX_MANIFOLD_POINTS)
            .enumerate()
        {
            let mut p1 = [Point::origin(); MAX_MANIFOLD_POINTS];
            let mut local_p2 = [Point::origin(); MAX_MANIFOLD_POINTS];

            if flip {
                // Don't forget that we already swapped rb1 and rb2 above.
                // So if we flip, only manifold_contacts[k].{local_p1,local_p2} have to
                // be swapped.
                for k in 0..manifold_contacts.len() {
                    p1[k] = coll_pos1 * (manifold_contacts[k].local_p2 + shift1);
                    local_p2[k] = delta2 * (manifold_contacts[k].local_p1 + shift2);
                }
            } else {
                for k in 0..manifold_contacts.len() {
                    p1[k] = coll_pos1 * (manifold_contacts[k].local_p1 + shift1);
                    local_p2[k] = delta2 * (manifold_contacts[k].local_p2 + shift2);
                }
            }

            let constraint = PositionGroundConstraint {
                rb2: rb2.active_set_offset,
                p1,
                local_p2,
                n1,
                radius,
                im2: rb2.mass_properties.inv_mass,
                ii2: rb2.world_inv_inertia_sqrt.squared(),
                num_contacts: manifold_contacts.len() as u8,
                erp: params.erp,
                max_linear_correction: params.max_linear_correction,
            };

            if push {
                if manifold.kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints.push(AnyPositionConstraint::NongroupedPointPointGround(
                        constraint,
                    ));
                } else {
                    out_constraints.push(AnyPositionConstraint::NongroupedPlanePointGround(
                        constraint,
                    ));
                }
            } else {
                if manifold.kinematics.category == KinematicsCategory::PointPoint {
                    out_constraints[manifold.constraint_index + l] =
                        AnyPositionConstraint::NongroupedPointPointGround(constraint);
                } else {
                    out_constraints[manifold.constraint_index + l] =
                        AnyPositionConstraint::NongroupedPlanePointGround(constraint);
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
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
            let p1 = self.p1[k];
            let p2 = pos2 * self.local_p2[k];
            let dpos = p2 - p1;

            let sqdist = dpos.norm_squared();

            // NOTE: only works for the point-point case.
            if sqdist < target_dist * target_dist {
                let dist = sqdist.sqrt();
                let n = dpos / dist;
                let err = ((dist - target_dist) * self.erp).max(-self.max_linear_correction);
                let dp2 = p2.coords - pos2.translation.vector;

                let gcross2 = -dp2.gcross(n);
                let ii_gcross2 = self.ii2.transform_vector(gcross2);

                // Compute impulse.
                let inv_r = self.im2 + gcross2.gdot(ii_gcross2);
                let impulse = err / inv_r;

                // Apply impulse.
                let tra2 = Translation::from(n * (-impulse * self.im2));
                let rot2 = Rotation::new(ii_gcross2 * impulse);
                pos2 = Isometry::from_parts(tra2 * pos2.translation, rot2 * pos2.rotation);
            }
        }

        positions[self.rb2] = pos2;
    }

    pub fn solve_plane_point(
        &self,
        params: &IntegrationParameters,
        positions: &mut [Isometry<f32>],
    ) {
        // FIXME: can we avoid most of the multiplications by pos1/pos2?
        // Compute jacobians.
        let mut pos2 = positions[self.rb2];
        let allowed_err = params.allowed_linear_error;
        let target_dist = self.radius - allowed_err;

        for k in 0..self.num_contacts as usize {
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
