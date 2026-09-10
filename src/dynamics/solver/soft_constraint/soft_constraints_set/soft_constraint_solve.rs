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
        let fem = vc.fem.map(|side| {
            let n = sb.particles.len();
            (
                side,
                &self.fem_responses[side.start as usize..side.start as usize + n],
                slots.first().copied().unwrap_or(u32::MAX) as usize,
            )
        });

        // The bias is capped so that it pushes no particle faster than the max corrective
        // velocity: a crushed closed surface has a large volume error but near-cancelling
        // gradients, and the uncapped correction would fling its particles.
        let mut g_max: Real = 0.0;
        match &fem {
            Some((side, _, _)) => g_max = side.u_max,
            None => {
                for (i, p) in sb.particles.iter().enumerate() {
                        g_max = g_max.max(grads[i].length() * p.inv_mass);
                }
            }
        }
        let max_bias = if g_max * vc.inv_lhs > 0.0 {
            vc.max_bias_velocity / (g_max * vc.inv_lhs)
        } else {
            Real::MAX
        };
        let rhs_bias = (vc.rhs * vc.erp_inv_dt).clamp(-max_bias, max_bias);

        let apply = |bodies: &mut SolverBodies, impulse: Real| match &fem {
            Some((_, u, first)) => {
                for (i, ui) in u.iter().enumerate() {
                    bodies.vels[first + i].linear -= *ui * impulse;
                }
            }
            None => {
                for (i, p) in sb.particles.iter().enumerate() {
                        bodies.vels[s as usize].linear -= grads[i] * (p.inv_mass * impulse);
                }
            }
        };

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

    /// Applies an overlap constraint's warm impulses (see `SoftOverlapConstraint::warm`) at the
    /// start of every substep, before the elastic constraints: the per-particle impulses of the
    /// last step on the first substep, the constraint's own (fitted, then solved) on later ones.
    pub fn warmstart_overlap_constraint(&mut self, oi: usize, bodies: &mut SolverBodies) {
        let constraint = &mut self.overlap_constraints[oi];
        if constraint.warm.is_none() {
            return;
        }
        let num_bodies = bodies.len();
        let live = |slot: u32| (slot as usize) < num_bodies;
        let grads = &self.overlap_grads[constraint.grads.clone()];
        let fem_sides = &self.overlap_fem_sides
            [constraint.fem_sides.start as usize..constraint.fem_sides.end as usize];
        let pool = &self.fem_responses;
        if constraint.warm_pending {
            constraint.warm_pending = false;
            let warm_impulses = &self.overlap_warm_impulses[constraint.grads.clone()];
            for (&(slot, im, _, _), &p) in grads.iter().zip(warm_impulses) {
                if live(slot) {
                    bodies.vels[slot as usize].linear += p * im;
                }
            }
            for side in fem_sides {
                if side.warm != u32::MAX {
                    let n = side.num_particles as usize;
                    let u = &pool[side.warm as usize..side.warm as usize + n];
                    let first = side.first_slot as usize;
                    for (i, ui) in u.iter().enumerate() {
                        bodies.vels[first + i].linear += *ui;
                    }
                }
            }
            if let Some((slot, ..)) = constraint.rigid.filter(|r| live(r.0)) {
                let pose = bodies.get_pose(slot);
                let (lin, ang) = constraint.warm_rigid;
                let v = &mut bodies.vels[slot as usize];
                v.linear += pose.im.component_mul(&lin);
                v.angular += pose.ii.transform_vector(ang);
            }
            return;
        }
        let constraint = &self.overlap_constraints[oi];
        for &(slot, im, g, _) in grads {
            if live(slot) {
                bodies.vels[slot as usize].linear -= g * (im * constraint.impulse);
            }
        }
        for side in fem_sides {
            let n = side.num_particles as usize;
            let u = &pool[side.start as usize..side.start as usize + n];
            let first = side.first_slot as usize;
            for (i, ui) in u.iter().enumerate() {
                bodies.vels[first + i].linear -= *ui * constraint.impulse;
            }
        }
        if let Some((slot, g_lin, g_ang)) = constraint.rigid.filter(|r| live(r.0)) {
            let pose = bodies.get_pose(slot);
            let ii_g = pose.ii.transform_vector(g_ang);
            let v = &mut bodies.vels[slot as usize];
            v.linear -= pose.im.component_mul(&g_lin) * constraint.impulse;
            v.angular -= ii_g * constraint.impulse;
        }
    }

    pub fn solve_overlap_constraint(&mut self, oi: usize, bodies: &mut SolverBodies) {
        let constraint = &mut self.overlap_constraints[oi];
        let num_bodies = bodies.len();
        let live = |slot: u32| (slot as usize) < num_bodies;
        let grads: &[(u32, Real, Vector, Vector)] = &self.overlap_grads[constraint.grads.clone()];
        let fem_sides = &self.overlap_fem_sides
            [constraint.fem_sides.start as usize..constraint.fem_sides.end as usize];
        let pool = &self.fem_responses;
        let mut w = 0.0;
        let mut g_max: Real = 0.0;
        for &(slot, im, g, _) in grads {
            if live(slot) {
                w += im * g.length_squared();
                g_max = g_max.max(g.length() * im);
            }
        }
        for side in fem_sides {
            w += side.gain;
            g_max = g_max.max(side.u_max);
        }
        // The rigid side's effective mass (its mass properties are step-constant, read from
        // the solver body).
        let rigid = constraint.rigid.filter(|r| live(r.0)).map(|(slot, g_lin, g_ang)| {
            let pose = bodies.get_pose(slot);
            let ii_g = pose.ii.transform_vector(g_ang);
            (slot, g_lin, g_ang, pose.im, ii_g)
        });
        if let Some((_, g_lin, g_ang, im, ii_g)) = rigid {
            w += g_lin.gdot(im.component_mul(&g_lin)) + ii_g.gdot(g_ang);
            g_max = g_max.max(im.component_mul(&g_lin).length());
        }
        if w <= 0.0 {
            return;
        }
        // A hard constraint's volume, updated from the substep's poses (see `hard`).
        if constraint.hard {
            let mut rhs = constraint.rhs0;
            for &(slot, _, g, x0) in grads {
                if live(slot) {
                    rhs += g.gdot(bodies.get_pose(slot).translation - x0);
                }
            }
            if let Some((slot, g_lin, g_ang, _, _)) = rigid {
                let pose = bodies.get_pose(slot);
                let (com0, rot0) = constraint.rigid_pose0;
                let drot = pose.rotation * rot0.inverse();
                #[cfg(feature = "dim2")]
                let dtheta = drot.angle();
                #[cfg(feature = "dim3")]
                let dtheta = drot.to_scaled_axis();
                rhs += g_lin.gdot(pose.translation - com0) + g_ang.gdot(dtheta);
            }
            constraint.rhs = rhs;
        }
        let cfm_gain = w * constraint.cfm_coeff;
        let inv_lhs = crate::utils::inv(w + cfm_gain);
        // Capped like the volume-piece constraints: no particle faster than the pace.
        let max_bias = if g_max * inv_lhs > 0.0 {
            constraint.max_bias_velocity / (g_max * inv_lhs)
        } else {
            Real::MAX
        };
        let rhs_bias = if constraint.rhs < 0.0 {
            constraint.rhs * constraint.speculative_inv_dt
        } else {
            (constraint.rhs * constraint.erp_inv_dt).clamp(0.0, max_bias)
        };
        let mut dc = 0.0;
        for &(slot, _, g, _) in grads {
            if live(slot) {
                dc += g.gdot(bodies.get_vel(slot).linear);
            }
        }
        if let Some((slot, g_lin, g_ang, _, _)) = rigid {
            let v = bodies.get_vel(slot);
            dc += g_lin.gdot(v.linear) + g_ang.gdot(v.angular);
        }
        let mut total =
            (constraint.impulse + inv_lhs * (dc + rhs_bias - cfm_gain * constraint.impulse)).max(0.0);
        // A soft overlap constraint's accumulated impulse is bounded by the pace (released elastic
        // energy is metered, not shot into the lighter side); a hard constraint keeps its velocity
        // part, and one applying the correction itself (`rhs > 0`) bounds the bias instead.
        if !constraint.hard && constraint.rhs <= 0.0 && g_max > 0.0 {
            total = total.min(constraint.max_bias_velocity / g_max);
        }
        let delta = total - constraint.impulse;
        constraint.impulse = total;
        for &(slot, im, g, _) in grads {
            if live(slot) {
                bodies.vels[slot as usize].linear -= g * (im * delta);
            }
        }
        for side in fem_sides {
            let n = side.num_particles as usize;
            let u = &pool[side.start as usize..side.start as usize + n];
            let first = side.first_slot as usize;
            for (i, ui) in u.iter().enumerate() {
                bodies.vels[first + i].linear -= *ui * delta;
            }
        }
        if let Some((slot, g_lin, _, im, ii_g)) = rigid {
            let v = &mut bodies.vels[slot as usize];
            v.linear -= im.component_mul(&g_lin) * delta;
            v.angular -= ii_g * delta;
        }
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
                // A new substep begins: the impulse is the previous one's total.
                constraint.peak_impulse = constraint.peak_impulse.max(constraint.impulse);
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

    /// Solves one shape-matching constraint: updated from the current poses when `update`,
    /// warm-started with `Some(coefficient)`.
    #[inline]
    pub fn solve_shape_constraint(
        &mut self,
        constraint_id: usize,
        bodies: &mut SolverBodies,
        update: bool,
        warmstart: Option<Real>,
    ) {
        let constraint = &mut self.shape_constraints[constraint_id];
        let pool = &self.fem_responses;
        if update {
            constraint.update(bodies);
        }
        if let Some(coeff) = warmstart {
            constraint.impulse *= coeff;
            constraint.warmstart(bodies, pool);
        }
        constraint.solve(bodies, pool);
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
        let pool: &[Vector] = &self.fem_responses;
        for constraint in &mut self.attachments[self.groups[group].attachments.clone()] {
            if update {
                constraint.update(bodies, wo_bias);
            }
            if let Some(coeff) = warmstart {
                constraint.warmstart(bodies, pool, coeff);
            }
            constraint.solve(bodies, pool);
        }
    }
}
