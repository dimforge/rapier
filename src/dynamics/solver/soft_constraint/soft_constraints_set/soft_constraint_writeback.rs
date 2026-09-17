//! Writeback of the solved state to the soft bodies: element impulses and plastic flow, particle poses and velocities, shape-matching, volume and attachment state.

use core::ops::Range;
use core::sync::atomic::Ordering;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{Real, Vector};

use super::super::soft_element_constraint::SoftScalarConstraintWriteback;
use super::*;

impl SoftConstraintsSet {
    /// Writes the accumulated impulses (and shape-matching rotations) back to the soft bodies,
    /// applies the elastic cells' plastic flow (`dt`: the step length) and updates the elements'
    /// smoothed tear load. SAFETY: `range` constraints write disjoint elements, one worker each.
    pub unsafe fn writeback_constraints(&self, range: Range<usize>, dt: Real) {
        // Virtual index space: the scalar constraints first, then the elastic-cell block constraints.
        let num_scalar_constraints = self.scalar_constraints.len();
        for idx in range {
            if idx < num_scalar_constraints {
                let constraint = &self.scalar_constraints[idx];
                let awake = &self.awake[constraint.soft_body as usize];
                let sb = unsafe { &mut *awake.ptr };
                let material = sb.material;
                let impulse = crate::utils::canonicalize_zero(constraint.impulse);
                match constraint.writeback {
                    SoftScalarConstraintWriteback::Edge => {
                        let resistance = if material.tears() {
                            let e = &sb.edges[constraint.element as usize];
                            sb.effective_tear_resistance(e.tear_resistance, &e.vertices)
                        } else {
                            1.0
                        };
                        let edge = &mut sb.edges[constraint.element as usize];
                        edge.impulse = impulse;
                        // Length as seen at the last substep's update.
                        let len = (constraint.pos[1] - constraint.pos[0]).length();
                        if edge.plastic_flow(len, &material, dt) {
                            awake.plastic_flow.store(true, Ordering::Relaxed);
                        }
                        if material.tears() {
                            // The force is the step's peak substep impulse over the substep length
                            // (a spike must not hide behind the last substep); the stretch is
                            // measured against the initial rest length, so plastic flow counts.
                            let peak = constraint.peak_impulse.max(constraint.impulse);
                            let force = peak * crate::utils::inv(awake.substep_dt);
                            let load =
                                material.edge_tear_load(len, edge.initial_rest_length(), force)
                                    * crate::utils::inv(resistance);
                            edge.stress = material.smooth_stress(edge.stress, load, dt);
                            if edge.stress > 1.0 {
                                edge.torn = true;
                                awake.torn.store(true, Ordering::Relaxed);
                            }
                        }
                    }
                    #[cfg(feature = "dim3")]
                    SoftScalarConstraintWriteback::Dihedral => {
                        sb.dihedrals[constraint.element as usize].impulse = impulse
                    }
                    SoftScalarConstraintWriteback::CellVolume => {
                        sb.cells[constraint.element as usize].impulses[0] = impulse
                    }
                }
            } else {
                let constraint = &self.elastic_constraints[idx - num_scalar_constraints];
                let awake = &self.awake[constraint.soft_body as usize];
                let sb = unsafe { &mut *awake.ptr };
                let material = sb.material;
                let resistance = if material.tear_strain.is_some() {
                    let c = &sb.cells[constraint.element as usize];
                    sb.effective_tear_resistance(c.tear_resistance, &c.vertices)
                } else {
                    1.0
                };
                let cell = &mut sb.cells[constraint.element as usize];
                let impulses =
                    (constraint.strain_impulse.iter()).chain(core::iter::once(&constraint.vol_impulse));
                for (dst, src) in cell.impulses.iter_mut().zip(impulses) {
                    *dst = crate::utils::canonicalize_zero(*src);
                }
                cell.rotation = constraint.rotation;
                if material.plastic_yield > 0.0
                    && material.plastic_creep > 0.0
                    && plastic_flow(cell, &constraint.strain, &material, dt)
                {
                    awake.plastic_flow.store(true, Ordering::Relaxed);
                }
                if material.tear_strain.is_some() {
                    let load = material.cell_tear_load(tensile) * crate::utils::inv(resistance);
                    cell.stress = material.smooth_stress(cell.stress, load, dt);
                    if cell.stress > 1.0 {
                        cell.torn = true;
                        awake.torn.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
    }

    /// Writes the solved poses and velocities of awake body `ai`'s particles back to the soft
    /// body (linear damping applied, a pinned particle snapped to its target). SAFETY: writes only
    /// that body's particles; the caller guarantees each awake body is written back by one worker.
    pub unsafe fn writeback_particles(&self, ai: usize, bodies: &SolverBodies, step_dt: Real) {
        let awake = &self.awake[ai];
        let sb = unsafe { &mut *awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let damping = crate::utils::inv(1.0 + step_dt * sb.particle_settings.linear_damping);
        for (p, &slot) in sb.particles.iter_mut().zip(slots) {
            let vel = bodies.get_vel(slot).linear;
            if p.inv_mass == 0.0 {
                // A target-driven pin lands exactly on its target (and stays in that mode: with
                // the target reached, the next step's velocity is zero unless a new one is set).
                p.velocity = vel;
                p.position = match p.next_position {
                    Some(target) => target,
                    None => bodies.get_pose(slot).translation,
                };
            } else {
                p.velocity = vel * damping;
                p.position = bodies.get_pose(slot).translation;
            }
        }
    }

    /// Writes back the shape-matching, volume-piece and attachment state (serial).
    pub fn writeback_bodies(&self) {
        for constraint in &self.attachments {
            // SAFETY: serial stage, exclusive access.
            let ptr = self.awake[constraint.soft_body as usize].ptr;
            let sb = unsafe { &mut *ptr };
            if let Some(a) = sb.attachments.get_mut(constraint.attachment as usize) {
                a.impulse = constraint.impulse;
            }
        }
        for awake in &self.awake {
            // SAFETY: serial stage, exclusive access.
            let sb = unsafe { &mut *awake.ptr };
            if !awake.awake_clusters.is_empty() {
                for awake_cluster in &awake.awake_clusters {
                    let cluster = &mut sb.clusters[awake_cluster.cluster as usize];
                    cluster.rotation = awake_cluster.rotation;
                    cluster
                        .shape_impulses
                        .resize(cluster.particles.len(), Vector::ZERO);
                }
                for constraint in &self.shape_constraints[awake.shape_constraints.clone()] {
                    let cluster = &mut sb.clusters[constraint.cluster as usize];
                    if let Ok(k) = cluster.particles.binary_search(&constraint.particle) {
                        cluster.shape_impulses[k] = constraint.impulse;
                    }
                }
            }
            for vc in &self.volume_constraints[awake.volume_constraints.clone()] {
                sb.volume_pieces[vc.piece as usize].impulse =
                    crate::utils::canonicalize_zero(vc.impulse);
            }
            sb.plastic_flowing = awake.plastic_flow.load(Ordering::Relaxed);
            sb.tearing_pending |= awake.torn.load(Ordering::Relaxed);
            sb.contact_approach_speeds =
                [awake.contact_approach_speed, sb.contact_approach_speeds[0]];
            // Summed by the contact writeback that follows the solve.
            sb.contact_load = 0.0;
        }
    }
}

pub(super) use super::super::soft_element_constraint::outer_product as outer;
