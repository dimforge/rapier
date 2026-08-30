//! Per-step and per-pass preparation: solver bodies from the particles, deformation damping, shape-matching goals and volume-piece gradients.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::{SolverBodies, SolverPose, SolverVel};
use crate::dynamics::SoftBody;
use crate::math::{AngVector, Matrix, Real, Vector};
use crate::utils::{AngularInertiaOps, CrossProduct};

use super::*;

impl SoftConstraintsSet {
    /// Per-pass prepare of one awake soft body: the substep's first pass (`update`) applies the
    /// deformation damping and refits what only moves with the poses; every pass refits the
    /// shape-matching goal velocities. Writes only this body's entries and particle velocities.
    pub fn prepare_awake_body(&mut self, ai: usize, bodies: &mut SolverBodies, update: bool) {
        let Self {
            awake,
            slots,
            shape_constraints,
            volume_constraints,
            volume_grads,
        } = self;
        let awake = &mut awake[ai];
        // SAFETY: read-only access to the soft body's particles and surface.
        let sb = unsafe { &*awake.ptr };
        let slots = &slots[awake.slot_start..awake.slot_start + awake.num_particles];
        if update && awake.damping_factor > 0.0 {
            damp_deformation(sb, slots, bodies, awake.damping_factor);
        }
        let bodies = &*bodies;
        let pos = |i: usize| -> Vector {
            let s = slots[i];
            if s == u32::MAX {
                sb.particles[i].position
            } else {
                bodies.get_pose(s).translation
            }
        };

        // Per-cluster shape matching: each shape-matched cluster gets its own best-fit frame
        // and pulls only its particles toward it.
        if !awake.awake_clusters.is_empty() {
            let vel = |i: usize| -> Vector {
                let s = slots[i];
                if s == u32::MAX {
                    sb.particles[i].velocity
                } else {
                    bodies.get_vel(s).linear
                }
            };
            let constraints = &mut shape_constraints[awake.shape_constraints.clone()];
            for k in 0..awake.awake_clusters.len() {
                let awake_cluster = &mut awake.awake_clusters[k];
                let cluster = &sb.clusters[awake_cluster.cluster as usize];

                if let Some(target_pose) = cluster.shape_matching_target {
                    // The shape-matching follows a kinematic path (step-constant: set once).
                    if !update {
                        continue;
                    }
                    // The mid-pose converges measurably faster toward the target; the target
                    // or the previous pose alone over-shoots or undershoots.
                    let mid_pose = cluster
                        .prev_shape_matching_target
                        .map(|prev| prev.lerp(&target_pose, 0.5))
                        .unwrap_or(target_pose);
                    awake_cluster.rotation = target_pose.rotation;

                    for constraint in constraints.iter_mut() {
                        if constraint.cluster != awake_cluster.cluster {
                            continue;
                        }
                        let particle = constraint.particle as usize;
                        constraint.goal = mid_pose * sb.particles[particle].rest_position;
                        constraint.goal_vel = awake_cluster.target_linvel
                            + awake_cluster
                                .target_angvel
                                .gcross(constraint.goal - mid_pose.translation);
                    }
                    continue;
                }

                if update {
                    // The fit of the poses: rotation and centroids, then the inertia of the
                    // free particles about their own center of mass (the velocity fit below
                    // reads it every pass).
                    let mut mass = 0.0;
                    let mut rest_com = Vector::ZERO;
                    let mut com = Vector::ZERO;
                    for &v in cluster.particles() {
                        let p = &sb.particles[v as usize];
                        mass += p.mass;
                        rest_com += p.rest_position * p.mass;
                        com += pos(v as usize) * p.mass;
                    }
                    if mass > 0.0 {
                        rest_com /= mass;
                        com /= mass;
                    }
                    let mut a = Matrix::ZERO;
                    for &v in cluster.particles() {
                        let p = &sb.particles[v as usize];
                        a += outer((pos(v as usize) - com) * p.mass, p.rest_position - rest_com);
                    }
                    let rot = crate::dynamics::soft_body::soft_body_shape_matching::extract_rotation(
                        a,
                        awake_cluster.rotation,
                    );
                    let mut dyn_mass = 0.0;
                    let mut dyn_com = Vector::ZERO;
                    for &v in cluster.particles() {
                        let p = &sb.particles[v as usize];
                        if p.inv_mass > 0.0 {
                            dyn_mass += p.mass;
                            dyn_com += pos(v as usize) * p.mass;
                        }
                    }
                    dyn_com *= crate::utils::inv(dyn_mass);
                    #[cfg(feature = "dim2")]
                    let mut inertia: crate::math::AngularInertia = 0.0;
                    #[cfg(feature = "dim3")]
                    let mut inertia = parry::utils::SdpMatrix3::zero();
                    for &v in cluster.particles() {
                        let p = &sb.particles[v as usize];
                        if p.inv_mass > 0.0 {
                            let r = pos(v as usize) - dyn_com;
                            #[cfg(feature = "dim2")]
                            {
                                inertia += p.mass * r.length_squared();
                            }
                            #[cfg(feature = "dim3")]
                            {
                                let d = p.mass * r.length_squared();
                                inertia = parry::utils::SdpMatrix3 {
                                    m11: inertia.m11 + d - p.mass * r.x * r.x,
                                    m12: inertia.m12 - p.mass * r.x * r.y,
                                    m13: inertia.m13 - p.mass * r.x * r.z,
                                    m22: inertia.m22 + d - p.mass * r.y * r.y,
                                    m23: inertia.m23 - p.mass * r.y * r.z,
                                    m33: inertia.m33 + d - p.mass * r.z * r.z,
                                };
                            }
                        }
                    }
                    let floor = crate::dynamics::soft_body::inertia_noise_floor(dyn_mass, dyn_com);
                    awake_cluster.rotation = rot;
                    awake_cluster.com = com;
                    awake_cluster.rest_com = rest_com;
                    awake_cluster.dyn_com = dyn_com;
                    awake_cluster.inv_inertia =
                        crate::dynamics::soft_body::pseudo_inverse_inertia(inertia, floor);
                    for constraint in constraints.iter_mut() {
                        if constraint.cluster != awake_cluster.cluster {
                            continue;
                        }
                        let particle = constraint.particle as usize;
                        constraint.goal =
                            com + rot * (sb.particles[particle].rest_position - rest_com);
                    }
                }

                // Rigid-fit velocity of the cluster (free particles): the constraints damp relative
                // to it, not to the world. The angular momentum is taken about the free particles'
                // center of mass, so the linear part drops out of it.
                let dyn_com = awake_cluster.dyn_com;
                let mut dyn_mass = 0.0;
                let mut vcom = Vector::ZERO;
                let mut angmom = crate::math::AngVector::default();
                for &v in cluster.particles() {
                    let p = &sb.particles[v as usize];
                    if p.inv_mass > 0.0 {
                        let velocity = vel(v as usize);
                        dyn_mass += p.mass;
                        vcom += velocity * p.mass;
                        angmom += (pos(v as usize) - dyn_com).gcross(velocity * p.mass);
                    }
                }
                vcom *= crate::utils::inv(dyn_mass);
                let omega = awake_cluster.inv_inertia.transform_vector(angmom);
                for constraint in constraints.iter_mut() {
                    if constraint.cluster != awake_cluster.cluster {
                        continue;
                    }
                    let particle = constraint.particle as usize;
                    constraint.goal_vel = vcom + omega.gcross(pos(particle) - dyn_com);
                }
            }
        }

        if !update {
            return;
        }
        if let Some(vi) = awake.volume_constraint {
            let vc = &mut volume_constraints[vi];
            let grads = &mut volume_grads[vc.grads.clone()];
            grads.fill(Vector::ZERO);
            let position = |i: u32| pos(i as usize);
            let volume = SoftBody::boundary_volume(&sb.boundary, position);
            SoftBody::boundary_volume_gradients(&sb.boundary, position, grads);
            let mut w = 0.0;
            for (i, p) in sb.particles.iter().enumerate() {
                    w += p.inv_mass * grads[i].length_squared();
            }
            vc.cfm_gain = w * vc.cfm_coeff;
            vc.inv_lhs = crate::utils::inv(w + vc.cfm_gain);
            // The position error, biased and capped at solve time. A piece turned inside out is
            // accepted as mirrored: the target follows the current orientation instead of
            // collapsing the body through zero volume to restore the rest winding.
            let target = if piece.inverted {
                -vc.target
            } else {
                vc.target
            };
            vc.rhs = volume - target;
        }
    }

    /// Fills the solver bodies of the awake soft bodies' particles: a pinned one is kinematic
    /// (zero inverse mass, velocity to its target), a free one gets scaled gravity and user force.
    /// `group_dt(group)` is the group's substep length, `step_dt` the whole step's.
    pub fn init_solver_bodies(
        &self,
        bodies: &mut SolverBodies,
        increments: &mut [SolverVel<Real>],
        gravity: Vector,
        step_dt: Real,
        group_dt: impl Fn(usize) -> Real,
    ) {
        let inv_step_dt = crate::utils::inv(step_dt);
        for awake in &self.awake {
            // SAFETY: read-only access to the soft body's particles.
            let sb = unsafe { &*awake.ptr };
            let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
            let dt = group_dt(awake.group as usize);
            let settings = &sb.particle_settings;
            for (p, &slot) in sb.particles.iter().zip(slots) {
                let slot = slot as usize;
                let (vel, incr) = if p.inv_mass == 0.0 {
                    let vel = match p.next_position {
                        Some(target) => (target - p.position) * inv_step_dt,
                        None => p.velocity,
                    };
                    (vel, Vector::ZERO)
                } else {
                    (
                        p.velocity,
                        (gravity * settings.gravity_scale + p.force * p.inv_mass) * dt,
                    )
                };
                let pose = &mut bodies.poses[slot];
                *pose = SolverPose::default();
                pose.translation = p.position;
                pose.im = Vector::splat(p.inv_mass);
                let v = &mut bodies.vels[slot];
                *v = SolverVel::zero();
                v.linear = vel;
                bodies.flags[slot] = 0;
                let inc = &mut increments[slot];
                *inc = SolverVel::zero();
                inc.linear = incr;
            }
        }
    }
}

/// Pulls the particles' velocities toward the body's best-fit rigid motion, removing the fraction
/// `factor` of the non-rigid part (see `SoftBodyMaterial::deformation_damping`). Pinned particles
/// weigh far more than free ones in the fit, so a clamped body's whole swing is damped.
fn damp_deformation(sb: &SoftBody, slots: &[u32], bodies: &mut SolverBodies, factor: Real) {
    let (com, vcom, omega) = rigid_fit_velocity(sb, slots, bodies);
    for (i, p) in sb.particles.iter().enumerate() {
        let s = slots[i];
        if s == u32::MAX || p.inv_mass == 0.0 || (s as usize) >= bodies.len() {
            continue;
        }
        let r = bodies.get_pose(s).translation - com;
        let v_rigid = vcom + omega.gcross(r);
        let v = &mut bodies.vels[s as usize].linear;
        *v += (v_rigid - *v) * factor;
    }
}

/// The best-fit rigid motion of a soft body's particles: mass-weighted center of mass, its
/// velocity and the angular velocity matching the angular momentum about it (pinned particles
/// weigh `1e4x` the total mass, so a pinned body's rigid motion is the pins').
fn rigid_fit_velocity(
    sb: &SoftBody,
    slots: &[u32],
    bodies: &SolverBodies,
) -> (Vector, Vector, AngVector) {
    let total_mass: Real = sb.particles.iter().map(|p| p.mass).sum();
    let pin_weight = total_mass * 1.0e4;
    let state = |i: usize| -> (Vector, Vector, Real) {
        let s = slots[i];
        let p = &sb.particles[i];
        let weight = if p.inv_mass == 0.0 {
            pin_weight
        } else {
            p.mass
        };
        if s == u32::MAX {
            (p.position, Vector::ZERO, weight)
        } else {
            (
                bodies.get_pose(s).translation,
                bodies.get_vel(s).linear,
                weight,
            )
        }
    };
    let mut mass = 0.0;
    let mut com = Vector::ZERO;
    let mut vcom = Vector::ZERO;
    for i in 0..sb.particles.len() {
        let (x, v, w) = state(i);
        mass += w;
        com += x * w;
        vcom += v * w;
    }
    if mass <= 0.0 {
        return (Vector::ZERO, Vector::ZERO, AngVector::default());
    }
    com /= mass;
    vcom /= mass;
    // Angular momentum and inertia about the center of mass.
    #[cfg(feature = "dim2")]
    let omega = {
        let mut l = 0.0;
        let mut inertia = 0.0;
        for i in 0..sb.particles.len() {
            let (x, v, w) = state(i);
            let r = x - com;
            l += w * r.gcross(v - vcom);
            inertia += w * r.length_squared();
        }
        if inertia > 0.0 { l / inertia } else { 0.0 }
    };
    #[cfg(feature = "dim3")]
    let omega = {
        let mut l = Vector::ZERO;
        let mut inertia = Matrix::ZERO;
        for i in 0..sb.particles.len() {
            let (x, v, w) = state(i);
            let r = x - com;
            l += r.cross(v - vcom) * w;
            inertia += (Matrix::IDENTITY * r.length_squared() - outer(r, r)) * w;
        }
        // Regularized: particles on a line (a rope) leave the rotation about that line
        // undetermined, and its momentum is zero anyway.
        let trace = inertia.x_axis.x + inertia.y_axis.y + inertia.z_axis.z;
        if trace > 0.0 {
            (inertia + Matrix::IDENTITY * (trace * 1.0e-6)).inverse() * l
        } else {
            Vector::ZERO
        }
    };
    (com, vcom, omega)
}

