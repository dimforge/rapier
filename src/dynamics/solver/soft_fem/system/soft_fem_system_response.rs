//! The step matrix: its factorization at the step start, and the response and column solves the constraints go through.

use super::super::soft_fem_skyline::SkylineCholesky;
use super::SoftFemSystem;
use crate::dynamics::SoftFemParameters;
use crate::math::{DIM, Matrix, Real, Vector};

impl SoftFemSystem {
    /// Assembles `A` at the step's start and factorizes it: the operator the constraints
    /// answer through for the whole step (see [`Self::response`]). A direct skyline Cholesky
    /// up to `max_dense_dofs` degrees of freedom, the block-Jacobi PCG on the kept matrix above.
    pub fn factorize_step_matrix(&mut self, dt: Real, params: &SoftFemParameters) {
        // The same operator as the first substep's predict (same positions): kept for it.
        self.assemble_operator(dt);
        self.operator_fresh = true;
        self.step_matrix.clone_from(&self.matrix);
        let dofs = self.matrix.num_rows() * DIM;
        self.direct_valid = false;
        if dofs <= params.max_dense_dofs {
            let direct = self
                .direct
                .get_or_insert_with(|| SkylineCholesky::new(&self.step_matrix));
            self.direct_valid = direct.factorize(&self.step_matrix);
        }
        if !self.direct_valid {
            self.step_cg.update_preconditioner(&self.step_matrix);
        }
    }

    /// Solves `A_step x = b`.
    fn solve_step(&mut self, b: &[Vector], x: &mut [Vector], params: &SoftFemParameters) {
        if let (true, Some(direct)) = (self.direct_valid, self.direct.as_mut()) {
            direct.solve(b, x);
        } else {
            x.fill(Vector::ZERO);
            self.step_cg.solve(
                &self.step_matrix,
                b,
                x,
                params.linear_tolerance,
                params.max_linear_iterations,
            );
        }
    }

    /// The body's answer to a constraint: for its jacobian `entries` `(particle, gradient)` on
    /// this body, solves `A_step u = Jᵀ` into `out` (one vector per particle, zero when pinned)
    /// and returns the gain `J · u` (augmented inverse mass), like `Multibody::fill_jacobians`.
    pub fn response_into(
        &mut self,
        entries: impl Iterator<Item = (u32, Vector)> + Clone,
        out: &mut [Vector],
        params: &SoftFemParameters,
    ) -> Real {
        let n = self.num_particles();
        debug_assert_eq!(out.len(), n);
        let mut rhs = core::mem::take(&mut self.response_rhs);
        rhs.clear();
        rhs.resize(n, Vector::ZERO);
        let mut any = false;
        for (p, g) in entries.clone() {
            if !self.pinned[p as usize] {
                rhs[p as usize] += g;
                any = true;
            }
        }
        if !any {
            out.fill(Vector::ZERO);
            self.response_rhs = rhs;
            return 0.0;
        }
        self.solve_step(&rhs, out, params);
        let mut gain = 0.0;
        for (p, g) in entries {
            if !self.pinned[p as usize] {
                gain += g.dot(out[p as usize]);
            }
        }
        self.response_rhs = rhs;
        gain.max(0.0)
    }

    pub fn num_particles(&self) -> usize {
        self.mass.len()
    }

    /// Forgets the step's column responses.
    pub fn clear_columns(&mut self) {
        let n = self.num_particles();
        self.column_of.clear();
        self.column_of.resize(n, u32::MAX);
        self.columns.clear();
        self.num_columns = 0;
    }

    /// Marks `particle` as constrained this step: [`Self::compute_columns`] then solves its `DIM`
    /// axis responses once, and its constraints combine them ([`Self::response_from_columns`])
    /// instead of solving on their own. A pinned particle answers nothing and is not loaded.
    pub fn load_particle(&mut self, particle: u32) {
        let p = particle as usize;
        if !self.pinned[p] && self.column_of[p] == u32::MAX {
            self.column_of[p] = self.num_columns as u32;
            self.num_columns += 1;
        }
    }

    /// Solves the axis responses of every loaded particle.
    pub fn compute_columns(&mut self, params: &SoftFemParameters) {
        let n = self.num_particles();
        let mut columns = core::mem::take(&mut self.columns);
        columns.clear();
        columns.resize(self.num_columns * DIM * n, Vector::ZERO);
        for p in 0..n {
            let column = self.column_of[p];
            if column == u32::MAX {
                continue;
            }
            for k in 0..DIM {
                let mut axis = Vector::ZERO;
                axis[k] = 1.0;
                let start = (column as usize * DIM + k) * n;
                self.response_into(
                    core::iter::once((p as u32, axis)),
                    &mut columns[start..start + n],
                    params,
                );
            }
        }
        self.columns = columns;
    }

    /// The response of loaded `particle` to a unit impulse along axis `k`.
    #[inline]
    fn column(&self, particle: u32, k: usize) -> Option<&[Vector]> {
        let column = self.column_of[particle as usize];
        if column == u32::MAX {
            return None;
        }
        let n = self.num_particles();
        let start = (column as usize * DIM + k) * n;
        Some(&self.columns[start..start + n])
    }

    /// [`Self::response_into`] from the column responses, `u = Σ C_p g_p` over the entries, with
    /// no solve; `None` when an entry's particle was not loaded (the caller solves).
    pub fn response_from_columns(
        &self,
        entries: impl Iterator<Item = (u32, Vector)> + Clone,
        out: &mut [Vector],
    ) -> Option<Real> {
        let n = self.num_particles();
        debug_assert_eq!(out.len(), n);
        out.fill(Vector::ZERO);
        let mut any = false;
        for (p, g) in entries.clone() {
            if self.pinned[p as usize] {
                continue;
            }
            any = true;
            for k in 0..DIM {
                let weight = g[k];
                if weight == 0.0 {
                    continue;
                }
                let column = self.column(p, k)?;
                for (o, c) in out.iter_mut().zip(column) {
                    *o += *c * weight;
                }
            }
        }
        if !any {
            return Some(0.0);
        }
        let mut gain = 0.0;
        for (p, g) in entries {
            if !self.pinned[p as usize] {
                gain += g.dot(out[p as usize]);
            }
        }
        Some(gain.max(0.0))
    }

    /// The `DIM` axis responses of loaded `particle`, copied into `out` (axis-major), and their
    /// values at the particle: the block `(A⁻¹)_pp`, the augmented inverse mass of a constraint
    /// acting on the particle alone (an attachment). `None` when the particle was not loaded.
    pub fn particle_block_from_columns(&self, particle: u32, out: &mut [Vector]) -> Option<Matrix> {
        let n = self.num_particles();
        debug_assert_eq!(out.len(), DIM * n);
        let mut inv_mass = Matrix::ZERO;
        if self.pinned[particle as usize] {
            out.fill(Vector::ZERO);
            return Some(inv_mass);
        }
        for k in 0..DIM {
            let column = self.column(particle, k)?;
            out[k * n..(k + 1) * n].copy_from_slice(column);
            *inv_mass.col_mut(k) = column[particle as usize];
        }
        // The block is symmetric up to the solve's accuracy: symmetrize it so the constraint's
        // inverse stays consistent with the responses.
        Some((inv_mass + inv_mass.transpose()) * 0.5)
    }
}
