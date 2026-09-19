//! The direct factorization of a FEM step matrix: a skyline (envelope) Cholesky under a reverse
//! Cuthill-McKee ordering of the particles. Cholesky fill-in stays inside the envelope of the
//! permuted matrix: factorization and solves cost its size, a few times a mesh matrix's non-zeros.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_fem_sparse::BlockMatrix;

/// The skyline Cholesky factorization of a symmetric positive-definite block matrix, in the
/// ordering computed from its pattern (see the module docs). The ordering and the envelope are
/// built once per pattern; [`Self::factorize`] refills and refactorizes for the current values.
#[derive(Clone, Debug)]
pub(crate) struct SkylineCholesky {
    /// Permuted block index to particle.
    perm: Vec<u32>,
    /// Particle to permuted block index.
    inv_perm: Vec<u32>,
    /// Per permuted degree of freedom, the first column of its envelope row.
    first: Vec<u32>,
    /// Start of every row's envelope in `values` (row `i` holds the columns `first[i]..=i`).
    row_start: Vec<usize>,
    /// The factor `L` (lower triangle, row by row over the envelope).
    values: Vec<Real>,
    /// The permuted right-hand side, then solution, of a solve.
    scratch: Vec<Real>,
}

impl SkylineCholesky {
    /// The ordering and envelope of `pattern` (its values are ignored).
    pub fn new(pattern: &BlockMatrix) -> Self {
        let n = pattern.num_rows();
        // The particle adjacency, without the diagonal.
        let mut offsets = Vec::with_capacity(n + 1);
        let mut adjacency: Vec<u32> = Vec::new();
        offsets.push(0u32);
        for r in 0..n {
            pattern.for_each_block_of_row(r as u32, |c, _| {
                if c as usize != r {
                    adjacency.push(c);
                }
            });
            offsets.push(adjacency.len() as u32);
        }
        let neighbors = |v: usize| &adjacency[offsets[v] as usize..offsets[v + 1] as usize];
        let perm = reverse_cuthill_mckee(n, neighbors);
        let mut inv_perm = vec![0u32; n];
        for (i, &p) in perm.iter().enumerate() {
            inv_perm[p as usize] = i as u32;
        }
        // The envelope: a block row starts at its lowest permuted neighbor.
        let mut first = Vec::with_capacity(n * DIM);
        let mut row_start = Vec::with_capacity(n * DIM + 1);
        row_start.push(0usize);
        for (i, &p) in perm.iter().enumerate() {
            let mut first_block = i;
            for &c in neighbors(p as usize) {
                first_block = first_block.min(inv_perm[c as usize] as usize);
            }
            for d in 0..DIM {
                let row = i * DIM + d;
                let col = first_block * DIM;
                first.push(col as u32);
                row_start.push(row_start[row] + (row - col + 1));
            }
        }
        let envelope = row_start[n * DIM];
        Self {
            perm,
            inv_perm,
            first,
            row_start,
            values: vec![0.0; envelope],
            scratch: vec![0.0; n * DIM],
        }
    }

    /// The number of stored factor entries.
    #[cfg(test)]
    pub fn envelope_len(&self) -> usize {
        self.values.len()
    }

    /// Factorizes `matrix` (same pattern as the one this was built from). Returns `false` when
    /// the matrix is not positive definite (the factor is then unusable).
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // A NaN pivot must fail the factorization.
    pub fn factorize(&mut self, matrix: &BlockMatrix) -> bool {
        let Self {
            inv_perm,
            first,
            row_start,
            values,
            ..
        } = self;
        let n_dofs = first.len();
        values.fill(0.0);
        // The lower triangle of the permuted matrix into the envelope.
        for r in 0..matrix.num_rows() {
            let i = inv_perm[r] as usize;
            matrix.for_each_block_of_row(r as u32, |c, block| {
                let j = inv_perm[c as usize] as usize;
                if j > i {
                    return;
                }
                for d in 0..DIM {
                    let row = i * DIM + d;
                    let base = row_start[row] - first[row] as usize;
                    for e in 0..DIM {
                        let col = j * DIM + e;
                        if col <= row {
                            values[base + col] = block.col(e)[d];
                        }
                    }
                }
            });
        }
        // Row-oriented Cholesky: row `i` of `L` from the rows above it, all inside the envelope.
        for i in 0..n_dofs {
            let fi = first[i] as usize;
            let ri = row_start[i];
            for j in fi..i {
                let fj = first[j] as usize;
                let rj = row_start[j];
                let k0 = fi.max(fj);
                let mut sum = values[ri + j - fi];
                if k0 < j {
                    let a = &values[ri + k0 - fi..ri + j - fi];
                    let b = &values[rj + k0 - fj..rj + j - fj];
                    sum -= a.iter().zip(b).map(|(x, y)| x * y).sum::<Real>();
                }
                values[ri + j - fi] = sum / values[rj + j - fj];
            }
            let mut diag = values[ri + i - fi];
            for k in fi..i {
                let l = values[ri + k - fi];
                diag -= l * l;
            }
            if !(diag > 0.0) {
                return false;
            }
            values[ri + i - fi] = diag.sqrt();
        }
        true
    }

    /// Solves `A x = b` with the last factorization (`b` and `x` per particle).
    pub fn solve(&mut self, b: &[Vector], x: &mut [Vector]) {
        let Self {
            perm,
            first,
            row_start,
            values,
            scratch,
            ..
        } = self;
        let n_dofs = first.len();
        for (i, &p) in perm.iter().enumerate() {
            for d in 0..DIM {
                scratch[i * DIM + d] = b[p as usize][d];
            }
        }
        // Forward: L y = b.
        for i in 0..n_dofs {
            let fi = first[i] as usize;
            let ri = row_start[i];
            let mut sum = scratch[i];
            if fi < i {
                let row = &values[ri..ri + i - fi];
                sum -= row
                    .iter()
                    .zip(&scratch[fi..i])
                    .map(|(l, y)| l * y)
                    .sum::<Real>();
            }
            scratch[i] = sum / values[ri + i - fi];
        }
        // Backward: Lᵀ x = y.
        for i in (0..n_dofs).rev() {
            let fi = first[i] as usize;
            let ri = row_start[i];
            let xi = scratch[i] / values[ri + i - fi];
            scratch[i] = xi;
            let row = &values[ri..ri + i - fi];
            for (y, l) in scratch[fi..i].iter_mut().zip(row) {
                *y -= l * xi;
            }
        }
        for (i, &p) in perm.iter().enumerate() {
            for d in 0..DIM {
                x[p as usize][d] = scratch[i * DIM + d];
            }
        }
    }
}

/// The reverse Cuthill-McKee ordering of a graph (new index to node): a breadth-first order
/// from a pseudo-peripheral node, neighbors by ascending degree, reversed; one component
/// after another. Deterministic (ties broken by node index).
fn reverse_cuthill_mckee<'a>(n: usize, neighbors: impl Fn(usize) -> &'a [u32]) -> Vec<u32> {
    let degree = |v: usize| neighbors(v).len();
    let mut order: Vec<u32> = Vec::with_capacity(n);
    let mut visited = vec![false; n];
    let mut levels = vec![u32::MAX; n];
    let mut queue: Vec<u32> = Vec::new();
    let mut sorted: Vec<u32> = Vec::new();
    // Breadth-first levels from `start` (over the unvisited nodes): the last level's node of
    // least degree, and the eccentricity.
    let bfs = |start: usize, levels: &mut Vec<u32>, queue: &mut Vec<u32>, visited: &[bool]| {
        levels.fill(u32::MAX);
        queue.clear();
        queue.push(start as u32);
        levels[start] = 0;
        let mut head = 0;
        while head < queue.len() {
            let v = queue[head] as usize;
            head += 1;
            for &w in neighbors(v) {
                let w = w as usize;
                if !visited[w] && levels[w] == u32::MAX {
                    levels[w] = levels[v] + 1;
                    queue.push(w as u32);
                }
            }
        }
        let last = levels[queue[queue.len() - 1] as usize];
        let far = queue
            .iter()
            .copied()
            .filter(|&v| levels[v as usize] == last)
            .min_by_key(|&v| (degree(v as usize), v))
            .unwrap_or(start as u32);
        (far as usize, last)
    };
    let mut done = 0;
    while done < n {
        // The component's start: a pseudo-peripheral node found from its least-degree node.
        let mut start = (0..n)
            .filter(|&v| !visited[v])
            .min_by_key(|&v| (degree(v), v))
            .unwrap();
        let (mut far, mut ecc) = bfs(start, &mut levels, &mut queue, &visited);
        for _ in 0..4 {
            let (next, next_ecc) = bfs(far, &mut levels, &mut queue, &visited);
            if next_ecc <= ecc {
                break;
            }
            start = far;
            far = next;
            ecc = next_ecc;
        }
        let _ = (far, ecc);
        // Cuthill-McKee over the component.
        let component_start = order.len();
        order.push(start as u32);
        visited[start] = true;
        let mut head = component_start;
        while head < order.len() {
            let v = order[head] as usize;
            head += 1;
            sorted.clear();
            sorted.extend(
                neighbors(v)
                    .iter()
                    .copied()
                    .filter(|&w| !visited[w as usize]),
            );
            sorted.sort_unstable_by_key(|&w| (degree(w as usize), w));
            for &w in &sorted {
                if !visited[w as usize] {
                    visited[w as usize] = true;
                    order.push(w);
                }
            }
        }
        done = order.len();
    }
    order.reverse();
    order
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::solver::soft_constraint::soft_element_constraint::outer_product;
    use crate::math::Matrix;

    /// A grid-shaped SPD block system (with a pinned row) solved through the skyline
    /// factorization must satisfy the system to solver accuracy, whatever the ordering did.
    #[test]
    fn skyline_solve_matches_the_system() {
        let (w, h) = (7usize, 5usize);
        let n = w * h;
        let id = |x: usize, y: usize| (y * w + x) as u32;
        let mut pairs: Vec<(u32, u32)> = Vec::new();
        for y in 0..h {
            for x in 0..w {
                if x + 1 < w {
                    pairs.push((id(x, y), id(x + 1, y)));
                }
                if y + 1 < h {
                    pairs.push((id(x, y), id(x, y + 1)));
                }
                if x + 1 < w && y + 1 < h {
                    pairs.push((id(x, y), id(x + 1, y + 1)));
                }
            }
        }
        let mut matrix = BlockMatrix::from_pairs(n, pairs.iter().copied());
        let mut seed = 987654321u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 33) as Real) / (u32::MAX as Real / 2.0) - 1.0
        };
        for r in 0..n {
            let idx = matrix.diagonal_index(r as u32) as usize;
            matrix.blocks[idx] = Matrix::IDENTITY * (1.0 + random().abs());
        }
        for &(a, b) in &pairs {
            let mut dir = Vector::ZERO;
            for d in 0..DIM {
                dir[d] = random();
            }
            let dir = dir.normalize();
            let block = outer_product(dir, dir) * (1.0 + random().abs());
            for (r, c) in [(a, b), (b, a)] {
                let idx = matrix.block_index(r, c) as usize;
                matrix.blocks[idx] += -block;
            }
            for r in [a, b] {
                let idx = matrix.diagonal_index(r) as usize;
                matrix.blocks[idx] += block;
            }
        }
        matrix.make_dirichlet(id(3, 2));

        let mut skyline = SkylineCholesky::new(&matrix);
        let mut seen = vec![false; n];
        for &p in &skyline.perm {
            assert!(!core::mem::replace(&mut seen[p as usize], true));
        }
        assert!(skyline.envelope_len() < n * DIM * (n * DIM + 1) / 2);
        assert!(skyline.factorize(&matrix));

        let rhs: Vec<Vector> = (0..n).map(|_| Vector::ONE * random()).collect();
        let mut x = vec![Vector::ZERO; n];
        skyline.solve(&rhs, &mut x);
        let mut ax = vec![Vector::ZERO; n];
        matrix.mul(&x, &mut ax);
        for k in 0..n {
            let err = (ax[k] - rhs[k]).length();
            assert!(err < 1.0e-4, "residual {err} at row {k}");
        }
        assert_eq!(x[id(3, 2) as usize], rhs[id(3, 2) as usize]);
    }
}
