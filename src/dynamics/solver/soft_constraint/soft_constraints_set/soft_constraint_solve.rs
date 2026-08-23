//! The solve passes: volume-piece and intersection-volume constraints, the colored element constraints and their re-sweep, shape-matching and attachment constraints.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{Real, Vector};
use crate::utils::{AngularInertiaOps, ComponentMul, DotProduct};

use super::*;

impl SoftConstraintsSet {
    /// Solves (or warm-starts) one volume-piece constraint on the given solver bodies.
    pub fn solve_volume_constraint(
        &mut self,
        vi: usize,
        bodies: &mut SolverBodies,
        warmstart: Option<Real>,
    ) {
        let vc = &mut self.volume_constraints[vi];
        let awake = &self.awake[vc.soft_body as usize];
        // SAFETY: read-only access to the soft body's particles.
        let sb = unsafe { &*awake.ptr };
        let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
        let grads = &self.volume_grads[vc.grads.clone()];
        // The bias is capped so that it pushes no particle faster than the max corrective
        // velocity: a crushed closed surface has a large volume error but near-cancelling
        // gradients, and the uncapped correction would fling its particles.
        let mut g_max: Real = 0.0;
        let max_bias = if g_max * vc.inv_lhs > 0.0 {
            vc.max_bias_velocity / (g_max * vc.inv_lhs)
        } else {
            Real::MAX
        };
        let rhs_bias = (vc.rhs * vc.erp_inv_dt).clamp(-max_bias, max_bias);

        if let Some(coeff) = warmstart {
            vc.impulse *= coeff;
            apply(bodies, vc.impulse);
        }

        let mut dc = 0.0;
        for i in 0..sb.particles.len() {
            let s = slots[i];
            if s != u32::MAX {
                dc += grads[i].gdot(bodies.get_vel(s).linear);
            }
        }
        let rhs = dc + rhs_bias;
        let total = vc.impulse + vc.inv_lhs * (rhs - vc.cfm_gain * vc.impulse);
        let delta = total - vc.impulse;
        vc.impulse = total;
        apply(bodies, delta);
    }

    /// Total number of element constraints (scalar + elastic-cell blocks), the writeback stage's domain.
    #[inline]
    pub fn num_element_constraints(&self) -> usize {
        self.scalar_constraints.len() + self.elastic_constraints.len()
    }

    /// Updates, optionally warm-starts, and solves the `idx`-th constraint of `range` (virtual
    /// index over the color's scalar then block constraints); an update also records whether it
    /// is strained past `min_strain` (see [`Self::resweep_strained_constraints`]).
    #[inline]
    pub fn solve_color_constraint(
        &mut self,
        range: &SoftColorRange,
        idx: usize,
        bodies: &mut SolverBodies,
        update: bool,
        warmstart: Option<Real>,
        min_strain: Real,
    ) {
        if idx < range.scalar_constraints.len() {
            let i = range.scalar_constraints.start + idx;
            let constraint = &mut self.scalar_constraints[i];
            if update {
                constraint.update(bodies);
                self.strained[i] = constraint.is_strained(min_strain);
            }
            if let Some(coeff) = warmstart {
                constraint.impulse *= coeff;
                constraint.warmstart(bodies);
            }
            constraint.solve(bodies);
        } else {
            let i = range.elastic_constraints.start + idx - range.scalar_constraints.len();
            let constraint = &mut self.elastic_constraints[i];
            if update {
                constraint.update(bodies);
                self.strained[self.scalar_constraints.len() + i] =
                    constraint.is_strained(min_strain);
            }
            if let Some(coeff) = warmstart {
                constraint.strain_impulse *= coeff;
                constraint.vol_impulse *= coeff;
                constraint.warmstart(bodies);
            }
            constraint.solve(bodies);
        }
    }

    /// Re-solves, serially, every constraint of the group flagged strained at its last update
    /// (distance error or largest cell strain past the re-sweep threshold): the re-sweep after
    /// the contacts touches only the few constraints being torn.
    pub fn resweep_strained_constraints(&mut self, group: usize, bodies: &mut SolverBodies) {
        let colors = self.groups[group].colors.clone();
        let serial = self.groups[group].serial.clone();
        let num_scalar = self.scalar_constraints.len();
        for ci in colors.chain(core::iter::once(usize::MAX)) {
            let (constraints, elastic_constraints) = if ci == usize::MAX {
                (serial.scalar_constraints.clone(), serial.elastic_constraints.clone())
            } else {
                let r = &self.color_ranges[ci];
                (r.scalar_constraints.clone(), r.elastic_constraints.clone())
            };
            for i in constraints {
                if self.strained[i] {
                    self.scalar_constraints[i].solve(bodies);
                }
            }
            for i in elastic_constraints {
                if self.strained[num_scalar + i] {
                    self.elastic_constraints[i].solve(bodies);
                }
            }
        }
    }

    /// Solves the attachment constraints of `group` (serial): updated from the current poses when
    /// `update`, warm-started with `Some(coefficient)`, without bias in the relax pass.
    pub fn solve_attachments(
        &mut self,
        group: usize,
        bodies: &mut SolverBodies,
        wo_bias: bool,
        update: bool,
        warmstart: Option<Real>,
    ) {
        for constraint in &mut self.attachments[self.groups[group].attachments.clone()] {
            if update {
                constraint.update(bodies, wo_bias);
            }
            if let Some(coeff) = warmstart {
            }
        }
    }
}
