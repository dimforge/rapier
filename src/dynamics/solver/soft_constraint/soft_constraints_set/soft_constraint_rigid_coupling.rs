//! Coupling with the rigid bodies: cluster proxy assembly with its gather/scatter stages, and the particle attachment constraints.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::{SolverBodies, SolverVel};
use crate::math::{DIM, Matrix, Real, Vector};
use crate::utils::CrossProduct;

use super::super::soft_element_constraint::MAX_CONSTRAINT_PARTICLES;
use super::*;

impl SoftConstraintsSet {
    /// Builds the particle attachment rows of the awake soft bodies (group-major, so it runs
    /// after [`Self::assemble`] laid the groups out): the particle's slot against the rigid
    /// body's (a fixed or non-simulated body anchors the particle to a world point).
    /// `group_dt(group)` is the group's substep length (the rows' softness).
    pub fn assemble_attachments(
        &mut self,
        island_id: usize,
        bodies: &crate::dynamics::RigidBodySet,
        group_dt: impl Fn(usize) -> Real,
    ) {
        self.attachments.clear();
        for gi in 0..self.groups.len() {
            let start = self.attachments.len();
            let dt = group_dt(gi);
            let softness = crate::dynamics::SpringCoefficients::<Real>::joint_defaults();
            let (erp_inv_dt, cfm_coeff) = (softness.erp_inv_dt(dt), softness.cfm_coeff(dt));
            for ai in self.groups[gi].awake.clone() {
                let awake = &self.awake[ai];
                if awake.frozen {
                    continue;
                }
                // SAFETY: read-only access during assembly.
                let sb = unsafe { &*awake.ptr };
                let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
                for (k, attachment) in sb.attachments.iter().enumerate() {
                    let Some(&particle) = slots.get(attachment.particle as usize) else {
                        continue;
                    };
                    let im_particle = sb.particles[attachment.particle as usize].inv_mass;
                    let Some(rb) = bodies.get(attachment.body) else {
                        continue;
                    };
                    let com_pose = rb
                        .pos
                        .position
                        .prepend_translation(rb.mprops.local_mprops.local_com);
                    let anchor0 = rb.pos.position * attachment.local_anchor;
                    let simulated = rb.ids.active_island_id == island_id as u32
                        && !rb.is_sleeping()
                        && rb.is_enabled()
                        && rb.is_dynamic_or_kinematic();
                    let (body, body_local_point) = if simulated {
                        (
                            rb.ids.active_set_id,
                            com_pose.inverse_transform_point(anchor0),
                        )
                    } else {
                        (u32::MAX, anchor0)
                    };
                    self.attachments
                        .push(super::soft_attachment::SoftAttachmentConstraint {
                            soft_body: ai as u32,
                            attachment: k as u32,
                            particle,
                            im_particle,
                            body_local_point,
                            anchor0,
                            erp_inv_dt,
                            cfm_coeff,
                            body_im: Vector::ZERO,
                            body_ii: Default::default(),
                            arm: Vector::ZERO,
                            lhs: Matrix::ZERO,
                            inv_lhs: Matrix::ZERO,
                            rhs: Vector::ZERO,
                            impulse: attachment.impulse,
                        });
                }
            }
            self.groups[gi].attachments = start..self.attachments.len();
        }
    }
}

#[allow(dead_code)]
const _: () = assert!(DIM + 1 == MAX_CONSTRAINT_PARTICLES);

/// Barycentric weights of `p` on the surface element (segment in 2D, triangle in 3D),
/// clamped to the element.
pub(crate) fn barycentric_weights(x: &[Vector], p: Vector) -> [Real; DIM] {
    #[cfg(feature = "dim2")]
    {
        let e = x[1] - x[0];
        let l2 = e.length_squared();
        let t = if l2 > 0.0 {
            ((p - x[0]).dot(e) / l2).clamp(0.0, 1.0)
        } else {
            0.5
        };
        [1.0 - t, t]
    }
    #[cfg(feature = "dim3")]
    {
        // Least squares on the triangle plane, then clamped into the triangle.
        let e1 = x[1] - x[0];
        let e2 = x[2] - x[0];
        let d = p - x[0];
        let a11 = e1.dot(e1);
        let a12 = e1.dot(e2);
        let a22 = e2.dot(e2);
        let b1 = d.dot(e1);
        let b2 = d.dot(e2);
        let det = a11 * a22 - a12 * a12;
        let (mut u, mut v) = if det.abs() > 1.0e-12 {
            ((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det)
        } else {
            (1.0 / 3.0, 1.0 / 3.0)
        };
        u = u.max(0.0);
        v = v.max(0.0);
        let sum = u + v;
        if sum > 1.0 {
            u /= sum;
            v /= sum;
        }
        [1.0 - u - v, u, v]
    }
}
