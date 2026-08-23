//! The scalar N-particle constraint: gradients of each element kind, its update, warm start and solve.

use crate::dynamics::SoftBody;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Number of particles a soft constraint can touch.
pub(crate) const MAX_CONSTRAINT_PARTICLES: usize = DIM + 1;

/// What a soft constraint constrains.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum SoftScalarConstraintKind {
    /// `|x1 - x0| - rest`, two particles.
    Distance,
    /// The angle between two triangles sharing an edge, four particles (3D only).
    #[cfg(feature = "dim3")]
    Dihedral,
    /// The signed area/volume of a simplex cell minus its rest value.
    CellVolume,
}

/// Where a soft constraint writes its accumulated impulse back (warm-start state on the soft body).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum SoftScalarConstraintWriteback {
    Edge,
    #[cfg(feature = "dim3")]
    Dihedral,
    /// `impulses[0]` of a cell (the volume model).
    CellVolume,
}

/// One scalar constraint over up to `MAX_CONSTRAINT_PARTICLES` particles.
///
/// Solved like a joint constraint: `Δλ = inv_lhs · (Ċ + rhs - cfm_gain·λ)`, clamped to the impulse
/// bounds, then `v_i -= im_i ∘ ∇_i C · Δλ`; gradients follow the positions before every pass.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftScalarConstraint {
    pub kind: SoftScalarConstraintKind,
    pub num_particles: u8,
    pub writeback: SoftScalarConstraintWriteback,
    /// Index of the owning soft body in the solver's awake list.
    pub soft_body: u32,
    /// Index of the element in the soft body's element array.
    pub element: u32,
    /// Solver-body slots of the particles (`u32::MAX`: no slot, position frozen).
    pub solver_ids: [u32; MAX_CONSTRAINT_PARTICLES],
    /// Current particle positions (updated by `update`).
    pub pos: [Vector; MAX_CONSTRAINT_PARTICLES],
    /// Particle inverse masses (step-constant).
    pub im: [Real; MAX_CONSTRAINT_PARTICLES],
    /// Current constraint gradients.
    pub grad: [Vector; MAX_CONSTRAINT_PARTICLES],
    /// Rest length / rest volume / rest angle, depending on the kind.
    pub rest: Real,
    pub erp_inv_dt: Real,
    pub cfm_coeff: Real,
    pub inv_lhs: Real,
    pub cfm_gain: Real,
    pub rhs: Real,
    pub impulse: Real,
    pub impulse_bounds: [Real; 2],
}

impl SoftScalarConstraint {
    /// Whether this constraint is strained beyond `min_strain` (relative error of a distance constraint; the
    /// other kinds never are): the criterion of the re-sweep after the contacts.
    #[inline]
    pub fn is_strained(&self, min_strain: Real) -> bool {
        self.kind == SoftScalarConstraintKind::Distance
            && self.erp_inv_dt > 0.0
            && self.rest > 0.0
            && (self.rhs / self.erp_inv_dt).abs() > min_strain * self.rest
    }

    /// Updates the positions, gradients, rhs and effective mass from the current solver-body
    /// poses. Soft constraints are springs whose bias is the spring force, not an error-correction
    /// velocity, so they are solved once per substep (biased pass only) with the bias always on.
    #[inline]
    pub fn update(&mut self, bodies: &SolverBodies) {
        let n = self.num_particles as usize;
        for k in 0..n {
            let id = self.solver_ids[k];
            if id != u32::MAX {
                self.pos[k] = bodies.get_pose(id).translation;
            }
        }

        let c = self.compute_gradients();

        // Effective mass and softness.
        let mut w = 0.0;
        for k in 0..n {
            w += self.grad[k].gdot(self.grad[k] * self.im[k]);
        }
        let cfm_gain = w * self.cfm_coeff;
        self.cfm_gain = cfm_gain;
        self.inv_lhs = crate::utils::inv(w + cfm_gain);

        // No corrective-velocity cap: the bias is the spring force itself, and capping it would
        // make the material's response depend on the substep count.
        self.rhs = c * self.erp_inv_dt;
    }

    /// Recomputes the gradients from `self.pos`; returns the constraint value.
    #[inline]
    pub(super) fn compute_gradients(&mut self) -> Real {
        match self.kind {
            SoftScalarConstraintKind::Distance => {
                let d = self.pos[1] - self.pos[0];
                let len = d.length();
                let n = if len > 1.0e-9 { d / len } else { Vector::X };
                self.grad[0] = -n;
                self.grad[1] = n;
                len - self.rest
            }
            #[cfg(feature = "dim3")]
            SoftScalarConstraintKind::Dihedral => self.dihedral_gradients(),
            SoftScalarConstraintKind::CellVolume => {
                let x: [Vector; DIM + 1] = core::array::from_fn(|k| self.pos[k]);
                let vol = SoftBody::cell_volume(x);
                vol - self.rest
            }
        }
    }

    #[cfg(feature = "dim3")]
    fn dihedral_gradients(&mut self) -> Real {
    /// Applies the accumulated impulse to the particle velocities.
    #[inline]
    pub fn warmstart(&self, bodies: &mut SolverBodies) {
        self.apply_impulse(bodies, self.impulse);
    }

    /// One Gauss-Seidel iteration on this constraint.
    #[inline]
    pub fn solve(&mut self, bodies: &mut SolverBodies) {
        let n = self.num_particles as usize;
        let mut dc = 0.0;
        for k in 0..n {
            let id = self.solver_ids[k];
            if id != u32::MAX {
                dc += self.grad[k].gdot(bodies.get_vel(id).linear);
            }
        }
        let rhs = dc + self.rhs;
        let total = (self.impulse + self.inv_lhs * (rhs - self.cfm_gain * self.impulse))
            .clamp(self.impulse_bounds[0], self.impulse_bounds[1]);
        let delta = total - self.impulse;
        self.impulse = total;
        self.apply_impulse(bodies, delta);
    }

    #[inline]
    fn apply_impulse(&self, bodies: &mut SolverBodies, impulse: Real) {
        let n = self.num_particles as usize;
        for k in 0..n {
            let id = self.solver_ids[k];
            if id != u32::MAX && (id as usize) < bodies.len() {
                let dv = self.grad[k] * self.im[k] * impulse;
                bodies.vels[id as usize].linear -= dv;
            }
        }
    }
}
