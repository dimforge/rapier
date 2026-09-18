//! The block-sparse matrix and the preconditioned conjugate gradient behind the FEM soft-body
//! solver. A soft body's system matrix `A = M + h D + h² K` is symmetric positive definite with a
//! `DIM × DIM` block per particle pair sharing an element, stored as block-CSR (both triangles).

use crate::alloc_prelude::*;
use crate::math::{Matrix, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// A block-sparse symmetric matrix with `DIM × DIM` blocks, stored in block-CSR with both
/// triangles; the sparsity pattern is built once from the element graph and reused across steps,
/// only the block values being refilled (`block_index` resolves a `(row, col)` pair to its slot).
#[derive(Clone, Debug, Default)]
pub(crate) struct BlockMatrix {
    /// Start of each row's blocks in `cols`/`blocks` (length `num_rows + 1`).
    row_offsets: Vec<u32>,
    /// Column of every stored block, ascending within a row.
    cols: Vec<u32>,
    /// Slot of every row's diagonal block.
    diagonal: Vec<u32>,
    /// The block values, parallel to `cols`.
    pub blocks: Vec<Matrix>,
}

impl BlockMatrix {
    /// Builds the pattern of a `num_rows × num_rows` block matrix: the diagonal blocks plus the
    /// off-diagonal `pairs` (either order, symmetrized). The values are left zeroed.
    pub fn from_pairs(num_rows: usize, pairs: impl Iterator<Item = (u32, u32)> + Clone) -> Self {
        let mut counts = vec![1u32; num_rows + 1];
        for (a, b) in pairs.clone() {
            if a != b {
                counts[a as usize] += 1;
                counts[b as usize] += 1;
            }
        }
        // Prefix sum into the row offsets (with room for the duplicates, pruned below).
        let mut row_offsets = Vec::with_capacity(num_rows + 1);
        let mut total = 0u32;
        for c in &counts[..num_rows] {
            row_offsets.push(total);
            total += *c;
        }
        row_offsets.push(total);

        let mut cols = vec![u32::MAX; total as usize];
        let mut cursors: Vec<u32> = row_offsets[..num_rows].to_vec();
        for r in 0..num_rows {
            cols[cursors[r] as usize] = r as u32;
            cursors[r] += 1;
        }
        for (a, b) in pairs {
            if a != b {
                cols[cursors[a as usize] as usize] = b;
                cursors[a as usize] += 1;
                cols[cursors[b as usize] as usize] = a;
                cursors[b as usize] += 1;
            }
        }

        // Sort and deduplicate each row, compacting in place.
        let mut out_cols = Vec::with_capacity(total as usize);
        let mut out_offsets = Vec::with_capacity(num_rows + 1);
        let mut diagonal = Vec::with_capacity(num_rows);
        for r in 0..num_rows {
            out_offsets.push(out_cols.len() as u32);
            let row = &mut cols[row_offsets[r] as usize..row_offsets[r + 1] as usize];
            row.sort_unstable();
            let mut previous = u32::MAX;
            for &c in row.iter() {
                if c != previous {
                    if c == r as u32 {
                        diagonal.push(out_cols.len() as u32);
                    }
                    out_cols.push(c);
                    previous = c;
                }
            }
        }
        out_offsets.push(out_cols.len() as u32);

        let blocks = vec![Matrix::ZERO; out_cols.len()];
        Self {
            row_offsets: out_offsets,
            cols: out_cols,
            diagonal,
            blocks,
        }
    }

    pub fn num_rows(&self) -> usize {
        self.row_offsets.len().saturating_sub(1)
    }

    /// The slot of the `(row, col)` block, or `u32::MAX` if the pattern has none.
    pub fn block_index(&self, row: u32, col: u32) -> u32 {
        let start = self.row_offsets[row as usize] as usize;
        let end = self.row_offsets[row as usize + 1] as usize;
        match self.cols[start..end].binary_search(&col) {
            Ok(k) => (start + k) as u32,
            Err(_) => u32::MAX,
        }
    }

    /// The slot of the `row`-th diagonal block.
    #[inline]
    pub fn diagonal_index(&self, row: u32) -> u32 {
        self.diagonal[row as usize]
    }

    /// Calls `f(col, block)` for every stored block of row `row`.
    #[inline]
    pub fn for_each_block_of_row(&self, row: u32, mut f: impl FnMut(u32, &Matrix)) {
        let start = self.row_offsets[row as usize] as usize;
        let end = self.row_offsets[row as usize + 1] as usize;
        for k in start..end {
            f(self.cols[k], &self.blocks[k]);
        }
    }

    pub fn clear_values(&mut self) {
        self.blocks.fill(Matrix::ZERO);
    }

    /// `out = A · x`.
    pub fn mul(&self, x: &[Vector], out: &mut [Vector]) {
        debug_assert_eq!(x.len(), self.num_rows());
        for (r, o) in out.iter_mut().enumerate() {
            let start = self.row_offsets[r] as usize;
            let end = self.row_offsets[r + 1] as usize;
            let mut acc = Vector::ZERO;
            for k in start..end {
                acc += self.blocks[k] * x[self.cols[k] as usize];
            }
            *o = acc;
        }
    }

    /// Turns row `row` into the identity row of a Dirichlet (pinned) degree of freedom: its
    /// diagonal block becomes the identity and every other block of the row and of the matching
    /// column is zeroed, which keeps the matrix symmetric.
    pub fn make_dirichlet(&mut self, row: u32) {
        let start = self.row_offsets[row as usize] as usize;
        let end = self.row_offsets[row as usize + 1] as usize;
        for k in start..end {
            let col = self.cols[k];
            self.blocks[k] = if col == row {
                Matrix::IDENTITY
            } else {
                Matrix::ZERO
            };
            if col != row {
                let mirror = self.block_index(col, row);
                if mirror != u32::MAX {
                    self.blocks[mirror as usize] = Matrix::ZERO;
                }
            }
        }
    }
}

/// Workspace and stopping rules of the block-Jacobi preconditioned conjugate gradient.
#[derive(Clone, Debug, Default)]
pub(crate) struct ConjugateGradient {
    /// Inverse of every diagonal block (the preconditioner).
    preconditioner: Vec<Matrix>,
    residual: Vec<Vector>,
    direction: Vec<Vector>,
    /// `A · direction`.
    a_direction: Vec<Vector>,
    /// The preconditioned residual.
    z: Vec<Vector>,
}

impl ConjugateGradient {
    /// Updates the block-Jacobi preconditioner from the matrix's diagonal blocks. Must be
    /// called after every change of the matrix values, before [`Self::solve`].
    pub fn update_preconditioner(&mut self, matrix: &BlockMatrix) {
        let n = matrix.num_rows();
        self.preconditioner.resize(n, Matrix::IDENTITY);
        for (r, p) in self.preconditioner.iter_mut().enumerate() {
            let block = matrix.blocks[matrix.diagonal_index(r as u32) as usize];
            // The diagonal blocks are SPD (every free particle has mass), but a degenerate
            // element can still make one singular: fall back to the identity there rather than
            // poisoning the whole solve with a non-finite preconditioner.
            let inv = block.inverse();
            *p = if inv.is_finite() {
                inv
            } else {
                Matrix::IDENTITY
            };
        }
    }

    /// Solves `A x = b` from the guess in `x` with fixed-order reductions (worker-independent
    /// result), returning the iteration count; stops on `‖r‖ ≤ tolerance · ‖b‖` or after
    /// `max_iterations`, then scales the solution by its energy minimizer, clamped to `[0, 1]`.
    pub fn solve(
        &mut self,
        matrix: &BlockMatrix,
        b: &[Vector],
        x: &mut [Vector],
        tolerance: Real,
        max_iterations: usize,
    ) -> usize {
        let n = matrix.num_rows();
        debug_assert_eq!(b.len(), n);
        debug_assert_eq!(x.len(), n);
        self.residual.resize(n, Vector::ZERO);
        self.direction.resize(n, Vector::ZERO);
        self.a_direction.resize(n, Vector::ZERO);
        self.z.resize(n, Vector::ZERO);

        let b_norm2: Real = b.iter().map(|v| v.length_squared()).sum();
        if b_norm2 == 0.0 {
            x.fill(Vector::ZERO);
            return 0;
        }
        let threshold = tolerance * tolerance * b_norm2;

        matrix.mul(x, &mut self.residual);
        for (r, bi) in self.residual.iter_mut().zip(b) {
            *r = *bi - *r;
        }
        let mut residual_norm2: Real = self.residual.iter().map(|v| v.length_squared()).sum();
        if residual_norm2 <= threshold {
            return 0;
        }
        for (k, z) in self.z.iter_mut().enumerate() {
            *z = self.preconditioner[k] * self.residual[k];
        }
        self.direction.copy_from_slice(&self.z);
        let mut rz: Real = self
            .residual
            .iter()
            .zip(&self.z)
            .map(|(r, z)| r.dot(*z))
            .sum();

        for iteration in 0..max_iterations {
            matrix.mul(&self.direction, &mut self.a_direction);
            let denominator: Real = self
                .direction
                .iter()
                .zip(&self.a_direction)
                .map(|(p, ap)| p.dot(*ap))
                .sum();
            if denominator.partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
                // A non-positive curvature means the matrix is not SPD (a degenerate element, or
                // a stiffness that slipped past the flooring): stop with what we have.
                self.rescale(matrix, b, x);
                return iteration;
            }
            let alpha = rz / denominator;
            for k in 0..n {
                x[k] += self.direction[k] * alpha;
                self.residual[k] -= self.a_direction[k] * alpha;
            }
            residual_norm2 = self.residual.iter().map(|v| v.length_squared()).sum();
            if residual_norm2 <= threshold {
                return iteration + 1;
            }
            for (k, z) in self.z.iter_mut().enumerate() {
                *z = self.preconditioner[k] * self.residual[k];
            }
            let rz_next: Real = self
                .residual
                .iter()
                .zip(&self.z)
                .map(|(r, z)| r.dot(*z))
                .sum();
            let beta = rz_next / rz;
            rz = rz_next;
            for k in 0..n {
                self.direction[k] = self.z[k] + self.direction[k] * beta;
            }
        }
        self.rescale(matrix, b, x);
        max_iterations
    }

    /// Scales `x` by the minimizer of `½ xᵀAx − bᵀx` along it, clamped to `[0, 1]`.
    fn rescale(&mut self, matrix: &BlockMatrix, b: &[Vector], x: &mut [Vector]) {
        matrix.mul(x, &mut self.a_direction);
        let numerator: Real = b.iter().zip(x.iter()).map(|(bi, xi)| bi.dot(*xi)).sum();
        let denominator: Real = x
            .iter()
            .zip(&self.a_direction)
            .map(|(xi, ax)| xi.dot(*ax))
            .sum();
        if denominator <= 0.0 {
            x.fill(Vector::ZERO);
            return;
        }
        let alpha = (numerator / denominator).clamp(0.0, 1.0);
        if alpha != 1.0 {
            for xi in x.iter_mut() {
                *xi *= alpha;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::solver::soft_constraint::soft_element_constraint::outer_product;

    /// A small SPD block system solved by the conjugate gradient leaves a negligible residual.
    #[test]
    fn conjugate_gradient_solves_the_system() {
        let n = 12;
        // A chain: every particle couples with its two neighbours.
        let pairs: Vec<(u32, u32)> = (0..n as u32 - 1).map(|i| (i, i + 1)).collect();
        let mut matrix = BlockMatrix::from_pairs(n, pairs.iter().copied());

        // Diagonally dominant SPD blocks: an isotropic mass plus a spring per neighbour.
        let mut seed = 12345u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 33) as Real) / (u32::MAX as Real / 2.0) - 1.0
        };
        for r in 0..n {
            let idx = matrix.diagonal_index(r as u32) as usize;
            matrix.blocks[idx] = Matrix::IDENTITY * (2.0 + random().abs());
        }
        for &(a, b) in &pairs {
            let dir = Vector::splat(1.0).normalize();
            let k = 1.0 + random().abs();
            let block = outer_product(dir, dir) * k;
            for (r, c) in [(a, b), (b, a)] {
                let idx = matrix.block_index(r, c) as usize;
                matrix.blocks[idx] += -block;
            }
            for r in [a, b] {
                let idx = matrix.diagonal_index(r) as usize;
                matrix.blocks[idx] += block;
            }
        }

        let rhs: Vec<Vector> = (0..n)
            .map(|_| Vector::splat(0.0) + Vector::ONE * random())
            .collect();
        let mut x = vec![Vector::ZERO; n];
        let mut cg = ConjugateGradient::default();
        cg.update_preconditioner(&matrix);
        cg.solve(&matrix, &rhs, &mut x, 1.0e-10, 1000);

        let mut ax = vec![Vector::ZERO; n];
        matrix.mul(&x, &mut ax);
        for k in 0..n {
            let err = (ax[k] - rhs[k]).length();
            assert!(err < 1.0e-4, "residual {err} at row {k}");
        }
    }

    /// A Dirichlet row must keep its right-hand side and decouple from its neighbours.
    #[test]
    fn dirichlet_rows_are_decoupled() {
        let pairs = [(0u32, 1u32), (1, 2)];
        let mut matrix = BlockMatrix::from_pairs(3, pairs.iter().copied());
        for r in 0..3 {
            let idx = matrix.diagonal_index(r) as usize;
            matrix.blocks[idx] = Matrix::IDENTITY * 4.0;
        }
        for &(a, b) in &pairs {
            for (r, c) in [(a, b), (b, a)] {
                let idx = matrix.block_index(r, c) as usize;
                matrix.blocks[idx] = Matrix::IDENTITY * -1.0;
            }
        }
        matrix.make_dirichlet(1);
        let x = [Vector::ONE, Vector::ONE * 2.0, Vector::ONE * 3.0];
        let mut out = [Vector::ZERO; 3];
        matrix.mul(&x, &mut out);
        assert_eq!(out[1], Vector::ONE * 2.0);
        assert_eq!(out[0], Vector::ONE * 4.0);
        assert_eq!(out[2], Vector::ONE * 12.0);
    }
}
