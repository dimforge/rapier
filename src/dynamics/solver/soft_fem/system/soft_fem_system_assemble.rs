//! The per-substep work: reading the particle state, assembling `A` and the elastic force, the predictor solve and the writeback to the soft body.

use super::SoftFemSystem;
use crate::alloc_prelude::*;
use crate::dynamics::SoftBody;
use crate::dynamics::soft_body::soft_body_shape_matching::extract_rotation;
use crate::dynamics::solver::soft_constraint::soft_constraints_set::plastic_flow;
#[cfg(feature = "dim3")]
use crate::dynamics::solver::soft_constraint::soft_element_constraint::dihedral_gradients;
use crate::dynamics::solver::soft_constraint::soft_element_constraint::{
    MAX_CONSTRAINT_PARTICLES, NeoHookeanConstraint, STIFFNESS_UPDATE_STRAIN, SoftElasticConstraint,
    StrainJacobian, outer_product, strain_rows_of,
};
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{Matrix, Real, Vector};
use crate::utils::RotationOps;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

impl SoftFemSystem {
    /// Reads the particles' state from their solver bodies.
    fn read_state(&mut self, bodies: &SolverBodies) {
        for (k, &slot) in self.slots.iter().enumerate() {
            if slot != u32::MAX {
                self.position[k] = bodies.get_pose(slot).translation;
                self.velocity[k] = bodies.get_vel(slot).linear;
            }
        }
    }

    /// Assembles `A = M + (h² + h β) K` and the elastic force at the current positions; pinned
    /// degrees of freedom become Dirichlet rows, with their coupling to the free rows
    /// (`Σ_{q pinned} A_pq v_q`) kept for the right-hand side; a step's first substep reuses it.
    pub(super) fn assemble_operator(&mut self, dt: Real) {
        self.matrix.clear_values();
        for (k, m) in self.mass.iter().enumerate() {
            let idx = self.matrix.diagonal_index(k as u32) as usize;
            self.matrix.blocks[idx] = Matrix::IDENTITY * *m;
        }
        self.force.fill(Vector::ZERO);

        for (ci, cell) in self.cells.iter_mut().enumerate() {
            if cell.mu <= 0.0 {
                continue;
            }
            let x: [Vector; MAX_CONSTRAINT_PARTICLES] =
                core::array::from_fn(|k| self.position[cell.vertices[k] as usize]);
            let f = SoftBody::cell_edge_matrix(x) * cell.inv_rest_matrix;
            cell.rotation = extract_rotation(f, cell.rotation);
            let s = cell.rotation.inverse().to_mat() * f;
            cell.strain = strain_rows_of(&(s - Matrix::IDENTITY));

            let gradient = if cell.neo_hookean {
                // Stable Neo-Hookean (Smith et al. 2018): the energy is defined and bounded
                // below for inverted cells, and its tangent is floored mode-wise at the rest
                // curvature so one implicit step per substep needs no line search.
                let lambda = cell.lambda + cell.mu;
                let (s, kappa, gradient) = NeoHookeanConstraint::gradient(cell.mu, lambda, &cell.strain);
                let stale = (cell.strain - cell.tangent_strain)
                    .iter()
                    .any(|d| d.abs() > STIFFNESS_UPDATE_STRAIN);
                if stale {
                    cell.tangent = NeoHookeanConstraint::hessian(cell.mu, lambda, &s, kappa);
                    cell.tangent_strain = cell.strain;
                }
                gradient
            } else {
                // Linear elasticity in the cell's rotation-free frame: a constant tangent.
                cell.tangent * cell.strain
            };

            // World-frame strain gradients: column `r` of `jacobians[p]` is `R gᵣ[p]`.
            let rotation = cell.rotation.to_mat();
            let jacobians: [StrainJacobian; MAX_CONSTRAINT_PARTICLES] = core::array::from_fn(|p| {
                SoftElasticConstraint::strain_jacobian(&cell.coeffs, p, &rotation)
            });

            // Elastic force `f_p = −Σᵣ g̃ᵣ[p] gradᵣ`.
            for p in 0..MAX_CONSTRAINT_PARTICLES {
                let fp: Vector = (jacobians[p] * gradient).into();
                self.force[cell.vertices[p] as usize] -= fp;
            }

            // Tangent `K_pq = Σᵣₛ Hᵣₛ g̃ᵣ[p] g̃ₛ[q]ᵀ = J_p H J_qᵀ`, scaled by `h² + h β` (the
            // stiffness and its Rayleigh damping share the same matrix); `K_qp = K_pqᵀ`.
            let scale = dt * dt + dt * cell.beta;
            let blocks = &self.cell_blocks[ci];
            for q in 0..MAX_CONSTRAINT_PARTICLES {
                let hq = cell.tangent * jacobians[q].transpose();
                for p in 0..=q {
                    let slot = blocks[p * MAX_CONSTRAINT_PARTICLES + q];
                    if slot == u32::MAX {
                        continue;
                    }
                    let block: Matrix = (jacobians[p] * hq).into();
                    let block = block * scale;
                    self.matrix.blocks[slot as usize] += block;
                    if p != q {
                        let mirror = blocks[q * MAX_CONSTRAINT_PARTICLES + p];
                        self.matrix.blocks[mirror as usize] += block.transpose();
                    }
                }
            }
        }

        self.assemble_springs(dt);
        #[cfg(feature = "dim3")]
        self.assemble_dihedrals(dt);
        self.assemble_volume_cells(dt);

        // The pinned particles' coupling to the free rows (`A_pq = A_qpᵀ`), then their Dirichlet
        // rows.
        self.pinned_coupling.clear();
        self.pinned_coupling.resize(self.mass.len(), Vector::ZERO);
        for (k, &pinned) in self.pinned.iter().enumerate() {
            if pinned {
                let v = self.velocity[k];
                let coupling = &mut self.pinned_coupling;
                self.matrix.for_each_block_of_row(k as u32, |col, block| {
                    if col as usize != k {
                        coupling[col as usize] += block.transpose() * v;
                    }
                });
                self.matrix.make_dirichlet(k as u32);
            }
        }
        self.cg.update_preconditioner(&self.matrix);
    }

    /// The predictor's right-hand side `b = h f − (A − M) v` from the assembled operator and
    /// the current velocities.
    fn assemble_rhs(&mut self, dt: Real) {
        self.matrix.mul(&self.velocity, &mut self.rhs);
        for k in 0..self.rhs.len() {
            self.rhs[k] = self.force[k] * dt - (self.rhs[k] + self.pinned_coupling[k])
                + self.velocity[k] * self.mass[k];
        }
        for (k, &pinned) in self.pinned.iter().enumerate() {
            if pinned {
                self.rhs[k] = Vector::ZERO;
            }
        }
    }

    /// Accumulates the volume elements' force and Gauss-Newton tangent.
    fn assemble_volume_cells(&mut self, dt: Real) {
        for element in &self.volume_cells {
            if element.stiffness <= 0.0 {
                continue;
            }
            let cell = &self.cells[element.cell as usize];
            let x: [Vector; MAX_CONSTRAINT_PARTICLES] =
                core::array::from_fn(|k| self.position[cell.vertices[k] as usize]);
            let c = SoftBody::cell_volume(x) - element.rest_volume;
            let grad = SoftBody::cell_volume_gradients(x);
            for k in 0..MAX_CONSTRAINT_PARTICLES {
                self.force[cell.vertices[k] as usize] -= grad[k] * (element.stiffness * c);
            }
            let scale = element.stiffness * (dt * dt + dt * element.beta);
            let blocks = &self.cell_blocks[element.cell as usize];
            for p in 0..MAX_CONSTRAINT_PARTICLES {
                for q in 0..MAX_CONSTRAINT_PARTICLES {
                    let slot = blocks[p * MAX_CONSTRAINT_PARTICLES + q];
                    if slot != u32::MAX {
                        self.matrix.blocks[slot as usize] += outer_product(grad[p], grad[q]) * scale;
                    }
                }
            }
        }
    }

    /// Accumulates the distance elements' force and Gauss-Newton tangent.
    fn assemble_springs(&mut self, dt: Real) {
        for spring in &self.springs {
            if spring.stiffness <= 0.0 {
                continue;
            }
            let a = spring.vertices[0] as usize;
            let b = spring.vertices[1] as usize;
            let d = self.position[b] - self.position[a];
            let length = d.length();
            if length < 1.0e-9 {
                continue;
            }
            let c = length - spring.rest_length;
            if spring.tension_only && c <= 0.0 {
                continue;
            }
            let n = d / length;
            let force = n * (spring.stiffness * c);
            self.force[a] += force;
            self.force[b] -= force;

            let block = outer_product(n, n) * (spring.stiffness * (dt * dt + dt * spring.beta));
            for (k, &slot) in spring.blocks.iter().enumerate() {
                if slot != u32::MAX {
                    // Diagonal blocks (k = 0, 3) get `+k nnᵀ`, off-diagonal ones `−k nnᵀ`.
                    let sign = if k == 0 || k == 3 { 1.0 } else { -1.0 };
                    self.matrix.blocks[slot as usize] += block * sign;
                }
            }
        }
    }

    /// Accumulates the dihedral elements' force and Gauss-Newton tangent.
    #[cfg(feature = "dim3")]
    fn assemble_dihedrals(&mut self, dt: Real) {
        for dihedral in &self.dihedrals {
            if dihedral.stiffness <= 0.0 {
                continue;
            }
            let pos: [Vector; 4] =
                core::array::from_fn(|k| self.position[dihedral.vertices[k] as usize]);
            let mut grad = [Vector::ZERO; 4];
            let c = dihedral_gradients(&pos, dihedral.rest_angle, &mut grad);
            for k in 0..4 {
                self.force[dihedral.vertices[k] as usize] -= grad[k] * (dihedral.stiffness * c);
            }
            let scale = dihedral.stiffness * (dt * dt + dt * dihedral.beta);
            for p in 0..4 {
                for q in 0..4 {
                    let slot = dihedral.blocks[p * 4 + q];
                    if slot != u32::MAX {
                        self.matrix.blocks[slot as usize] +=
                            outer_product(grad[p], grad[q]) * scale;
                    }
                }
            }
        }
    }

    /// The implicit elastic step of one substep: assemble, solve `A Δv = b`, and add `Δv` to the
    /// particles' solver velocities. Returns the conjugate-gradient iteration count.
    pub fn predict(
        &mut self,
        bodies: &mut SolverBodies,
        dt: Real,
        tolerance: Real,
        max_iterations: usize,
    ) -> usize {
        self.read_state(bodies);
        // The step-start factorization assembled this operator at these same positions (the
        // substep only changed the velocities): reused by the first substep.
        if !core::mem::take(&mut self.operator_fresh) {
            self.assemble_operator(dt);
        }
        self.assemble_rhs(dt);
        // Started from zero, never warm-started: a truncated CG converges the smooth modes first,
        // so a partial solve is an under-relaxed step, while a stale start leaves high-frequency
        // content the truncation never removes and pumps energy into a stiff body.
        self.delta.fill(Vector::ZERO);
        let iterations = self.cg.solve(
            &self.matrix,
            &self.rhs,
            &mut self.delta,
            tolerance,
            max_iterations,
        );
        for (k, &slot) in self.slots.iter().enumerate() {
            if slot != u32::MAX && !self.pinned[k] {
                bodies.vels[slot as usize].linear += self.delta[k];
            }
        }
        iterations
    }

    /// Writes back what the soft body keeps across steps: each cell's warm-started polar rotation,
    /// the step's plastic flow, and the tearing marks of elements strained past `tear_strain`
    /// (the FEM counterpart of `SoftConstraintsSet::writeback_constraints`).
    pub fn writeback(&self, sb: &mut SoftBody, step_dt: Real) {
        let material = sb.material;
        let plastic = material.plastic_yield > 0.0 && material.plastic_creep > 0.0;
        let mut flowing = false;
        let mut torn = false;
        // The interior strength reads the particles' flags: taken before the cells are borrowed.
            out.rotation = cell.rotation;
            if cell.mu <= 0.0 {
                continue;
            }
            if plastic && plastic_flow(out, &cell.strain, &material, step_dt) {
                flowing = true;
            }
            }
        }
        sb.plastic_flowing |= flowing;
        sb.tearing_pending |= torn;
    }
}
