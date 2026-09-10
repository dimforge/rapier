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
    /// Collects the step's active clusters, group-major (after [`Self::assemble`] laid the groups
    /// out): those whose proxy has an enabled joint (`jointed_proxies`) or a pending external
    /// impulse or force. The gather stage feeds their proxy slot; the scatter stage writes back.
    pub fn assemble_clusters(
        &mut self,
        bodies: &crate::dynamics::RigidBodySet,
        colliders: &crate::geometry::ColliderSet,
        solver_bodies: &SolverBodies,
        jointed_proxies: &parry::utils::hashmap::HashMap<u32, ()>,
    ) {
        self.clusters.clear();
        for gi in 0..self.groups.len() {
            let start = self.clusters.len();
            for ai in self.groups[gi].awake.clone() {
                let awake = &self.awake[ai];
                // SAFETY: read-only access during assembly.
                let sb = unsafe { &*awake.ptr };
                let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
                for (ci, cluster) in sb.clusters.iter().enumerate() {
                    if !cluster.is_live() {
                        continue;
                    }
                    let Some(rb) = bodies.get(cluster.proxy) else {
                        continue;
                    };
                    let slot = rb.ids.active_set_id;
                    if slot == u32::MAX {
                        continue;
                    }
                    let external = rb.vels.linvel != cluster.last_gather.0
                        || rb.vels.angvel != cluster.last_gather.1
                        || rb.forces.user_force != Vector::ZERO
                        || rb.forces.user_torque != crate::math::AngVector::default();
                    // A proxy with a collider of its own (not the soft-managed surface or
                    // balls) takes rigid contacts on its slot: those impulses must be scattered.
                    let has_user_collider = rb.colliders().iter().any(|co| {
                        colliders
                            .get(*co)
                            .is_some_and(|co| !co.is_deformable_collider())
                    });
                    if !jointed_proxies.contains_key(&slot) && !external && !has_user_collider {
                        continue;
                    }
                    let inv_inertia = rb.mprops.effective_world_inv_inertia;
                    let (com, _, _) = sb.gather_cluster_velocity(
                        ci,
                        &inv_inertia,
                        |v| solver_bodies.get_pose(slots[v as usize]).translation,
                        |v| solver_bodies.get_vel(slots[v as usize]).linear,
                    );
                    // The proxy slot starts at `rb.vels`, the sync-time gather: using that same
                    // gather as the reference scatters the external impulses applied since first.
                    let mut ref_vel = SolverVel::zero();
                    ref_vel.linear = cluster.last_gather.0;
                    ref_vel.angular = cluster.last_gather.1;
                    self.clusters.push(SoftClusterRecord {
                        awake: ai as u32,
                        cluster: ci as u32,
                        slot,
                        com,
                        ref_vel,
                    });
                }
            }
            self.groups[gi].clusters = start..self.clusters.len();
        }
    }

    /// The gather stage of one pass (worker 0, serial): re-derives each active cluster's rigid
    /// velocity and centroid from its particles' solver state and sets the proxy slot to
    /// `fresh + (slot - ref_vel)`, so the pending external and joint impulses stay.
    pub fn gather_clusters(&mut self, group: usize, bodies: &mut SolverBodies) {
        let range = self.groups[group].clusters.clone();
        for record in &mut self.clusters[range] {
            let awake = &self.awake[record.awake as usize];
            // SAFETY: worker-0 exclusive stage.
            let sb = unsafe { &*awake.ptr };
            let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
            let inv_inertia = bodies.get_pose(record.slot).ii;
            let (com, linvel, angvel) = sb.gather_cluster_velocity(
                record.cluster as usize,
                &inv_inertia,
                |v| bodies.get_pose(slots[v as usize]).translation,
                |v| bodies.get_vel(slots[v as usize]).linear,
            );
            record.com = com;
            let slot = record.slot as usize;
            let vels = &mut bodies.vels[slot];
            vels.linear += linvel - record.ref_vel.linear;
            vels.angular += angvel - record.ref_vel.angular;
            record.ref_vel.linear = linvel;
            record.ref_vel.angular = angvel;
            // The frame origin follows the particles exactly (the integrated proxy pose is only
            // an estimate of the same motion).
            bodies.poses[slot].translation = com;
        }
    }

    /// The scatter stage of one pass (worker 0, serial): distributes each active cluster's
    /// proxy-slot velocity change since the gather onto its free particles as the rigid field
    /// `Δv + Δω × r`; `Σ mᵢΔvᵢ` is the impulse the joints applied to the slot.
    pub fn scatter_clusters(&mut self, group: usize, bodies: &mut SolverBodies) {
        let range = self.groups[group].clusters.clone();
        for record in &mut self.clusters[range] {
            let slot_vels = bodies.get_vel(record.slot);
            let dlin = slot_vels.linear - record.ref_vel.linear;
            let dang = slot_vels.angular - record.ref_vel.angular;
            if dlin == Vector::ZERO && dang == crate::math::AngVector::default() {
                continue;
            }
            let awake = &self.awake[record.awake as usize];
            // SAFETY: worker-0 exclusive stage; overlapping clusters are fine, the stage is
            // serial.
            let sb = unsafe { &*awake.ptr };
            let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
            for &v in &sb.clusters[record.cluster as usize].particles {
                let p = &sb.particles[v as usize];
                if p.inv_mass > 0.0 {
                    let slot = slots[v as usize] as usize;
                    if slot < bodies.len() {
                        let r = bodies.poses[slot].translation - record.com;
                        bodies.vels[slot].linear += dlin + dang.gcross(r);
                    }
                }
            }
            record.ref_vel.linear = slot_vels.linear;
            record.ref_vel.angular = slot_vels.angular;
        }
    }

    /// Whether the given group has active clusters (the gather/scatter stages exist).
    #[inline]
    pub fn group_has_clusters(&self, group: usize) -> bool {
        !self.groups[group].clusters.is_empty()
    }

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
                            fem: None,
                            body,
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
    // A segment element (2D, and a wire in 3D): the unused weights stay at zero.
    #[cfg(feature = "dim3")]
    if x.len() < DIM {
        let mut weights = [0.0; DIM];
        if x.len() == 2 {
            let e = x[1] - x[0];
            let l2 = e.length_squared();
            let t = if l2 > 0.0 {
                ((p - x[0]).dot(e) / l2).clamp(0.0, 1.0)
            } else {
                0.5
            };
            weights[0] = 1.0 - t;
            weights[1] = t;
        } else if x.len() == 1 {
            weights[0] = 1.0;
        }
        return weights;
    }
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
