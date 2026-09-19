//! The stable Neo-Hookean state of an elastic cell: stiffness update, energy gradient and Hessian.

use super::*;
use crate::math::{DIM, Matrix, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Stable Neo-Hookean state of a [`SoftElasticConstraint`]: the energy of Smith et al. 2018 in
/// Kim & Eberle 2020's form, bounded below when inverted. `update` refreshes the gradient every
/// substep and the Hessian after a [`STIFFNESS_UPDATE_STRAIN`] move, modes floored at rest.
#[derive(Copy, Clone, Debug)]
pub(crate) struct NeoHookeanConstraint {
    /// Substep length.
    pub dt: Real,
    /// `μ V₀`.
    pub mu: Real,
    /// `λ' V₀ = (λ + μ) V₀`.
    pub lambda: Real,
    /// Damping coefficient of every strain row (cell frame).
    pub damping: StrainVector,
    // Per-pass state.
    /// Strain at which `m` and the block inverse were last computed (`Real::MAX`: never).
    pub stiffness_strain: StrainVector,
    /// `M = dt (dt K + D)`.
    pub m: StrainMatrix,
    /// `dt g`, the gradient of the energy scaled by the substep length.
    pub dt_grad: StrainVector,
}

/// Strain change (any component) past which a Neo-Hookean constraint recomputes its tangent stiffness.
pub(crate) const STIFFNESS_UPDATE_STRAIN: Real = 0.02;

impl NeoHookeanConstraint {
    /// Rest stiffness of strain row `r` per unit rest volume: `2μ + λ` on the diagonal rows, `4μ`
    /// on the shear rows (the linear-elastic block, reproduced by the Neo-Hookean Hessian at rest).
    #[inline]
    pub fn rest_stiffness(mu: Real, lambda: Real, r: usize) -> Real {
        if r < DIM { 2.0 * mu + lambda } else { 4.0 * mu }
    }

    /// Updates `M = dt (dt K + D)`, the gradient `dt g` at `strain`, and the block inverse
    /// `(M A + I)⁻¹` into `inv_a`.
    pub(super) fn update(
        &mut self,
        strain: &StrainVector,
        coeffs: &[Vector; MAX_CONSTRAINT_PARTICLES],
        im: &[Real; MAX_CONSTRAINT_PARTICLES],
        inv_a: &mut StrainMatrix,
    ) {
        let (s, kappa, grad) = Self::gradient(self.mu, self.lambda, strain);
        let dt = self.dt;
        self.dt_grad = grad * dt;
        // The stiffness only linearizes: it is kept while the strain stays within
        // `STIFFNESS_UPDATE_STRAIN` of the one it was computed at (a resting body then pays
        // one update per step, at its first substep).
        let fresh = (strain - self.stiffness_strain)
            .iter()
            .all(|d| d.abs() <= STIFFNESS_UPDATE_STRAIN);
        if fresh {
            return;
        }
        self.stiffness_strain = *strain;
        let hess = Self::hessian(self.mu, self.lambda, &s, kappa);
        self.m = (hess * dt + StrainMatrix::from_diagonal(&self.damping)) * dt;
        // `(M A + I)⁻¹`, with the effective-mass block `A_rs = Σₚ wₚ gᵣ[p]·gₛ[p]` built from
        // `G = Σₚ wₚ cₚ cₚᵀ` (the gradients are axis-aligned).
        let weighted: [Vector; MAX_CONSTRAINT_PARTICLES] =
            core::array::from_fn(|p| coeffs[p] * im[p]);
        let a = Self::strain_block_from_g(&outer_sum(&weighted, coeffs));
        *inv_a = (self.m * a + StrainMatrix::identity())
            .try_inverse()
            .unwrap_or_else(StrainMatrix::zeros);
    }

    /// The strain effective-mass block from `G = Σₚ wₚ cₚ cₚᵀ`: `A_rs = ¼ (δ_ik G_jl + δ_il G_jk +
    /// δ_jk G_il + δ_jl G_ik)`, with `gᵣ[p] = ½ (c_pj e_i + c_pi e_j)` for the row `r = (i, j)`.
    pub(super) fn strain_block_from_g(g: &Matrix) -> StrainMatrix {
        let pairs = strain_pairs();
        let delta = |i: usize, j: usize| if i == j { 1.0 } else { 0.0 };
        let at = |i: usize, j: usize| g.col(j)[i];
        StrainMatrix::from_fn(|r, s| {
            let ((i, j), (k, l)) = (pairs[r], pairs[s]);
            0.25 * (delta(i, k) * at(j, l)
                + delta(i, l) * at(j, k)
                + delta(j, k) * at(i, l)
                + delta(j, l) * at(i, k))
        })
    }

    /// The gradient `V₀ ∂Ψ/∂ε` and tangent stiffness (Hessian floored mode-wise at the rest
    /// curvature, see the type docs) at the strain `ε` (cell frame), `mu`/`lambda` scaled by `V₀`.
    #[cfg(test)]
    pub fn gradient_and_hessian(
        mu: Real,
        lambda: Real,
        strain: &StrainVector,
    ) -> (StrainVector, StrainMatrix) {
        let (s, kappa, grad) = Self::gradient(mu, lambda, strain);
        (grad, Self::hessian(mu, lambda, &s, kappa))
    }

    /// `S = I + ε`, `κ = λ'(J - 1) - μ` and the gradient `V₀ ∂Ψ/∂ε` at the strain `ε`.
    pub(crate) fn gradient(
        mu: Real,
        lambda: Real,
        strain: &StrainVector,
    ) -> (Matrix, Real, StrainVector) {
        let s = Matrix::IDENTITY + strain_matrix(strain);
        let kappa = lambda * (s.determinant() - 1.0) - mu;
        // ∂Ψ/∂S = μ S + κ cof(S).
        let grad = strain_rows_dual(&(s * mu + cofactor(&s) * kappa));
        (s, kappa, grad)
    }

    /// The tangent stiffness at `S` (see [`Self::gradient_and_hessian`]).
    pub(crate) fn hessian(mu: Real, lambda: Real, s: &Matrix, kappa: Real) -> StrainMatrix {
        // Hessian of Ψ in the principal frame of `S = Q diag(σ) Qᵀ`: μ I from the ‖F‖² term, the
        // ∂²J/∂σ² blocks from J, and λ' g_s g_sᵀ (g_s = ∂J/∂σ) from the volume term.
        let (sigma, q) = symmetric_eigen(s);
        // The product of the principal stretches other than `a` and `b`.
        let others = |a: usize, b: usize| {
            (0..DIM)
                .filter(|c| *c != a && *c != b)
                .fold(1.0, |prod, c| prod * sigma[c])
        };
        let mut g_s = Vector::ZERO;
        let mut h_s = Matrix::IDENTITY * mu;
        for a in 0..DIM {
            g_s[a] = others(a, a);
            for b in a + 1..DIM {
                h_s.col_mut(b)[a] += kappa * others(a, b);
                h_s.col_mut(a)[b] += kappa * others(a, b);
            }
        }
        h_s += outer_product(g_s, g_s) * lambda;
        // Floor every mode at its rest curvature (`2μ I + λ 11ᵀ` on the scaling block, `2μ` on the
        // flips); see the type docs.
        let (mut nu, v) = symmetric_eigen(&h_s);
        for k in 0..DIM {
            let sum = v.col(k).element_sum();
            nu[k] = nu[k].max(2.0 * mu + (lambda - mu) * sum * sum);
        }

        // Back to strain coordinates: `d_a = (q_a q_aᵀ) : ε`, `u_ab = √2 (q_a q_bᵀ) : ε`, so
        // `K = Σ_k ν_k w_k w_kᵀ + Σ_f h_f t_f t_fᵀ` with `w_k = Σ_a v_ak t_a`.
        let scaling: [StrainVector; DIM] =
            core::array::from_fn(|a| strain_rows_dual(&outer_product(q.col(a), q.col(a))));
        let mut k = StrainMatrix::zeros();
        for m in 0..DIM {
            let mut w = StrainVector::zeros();
            for a in 0..DIM {
                w += scaling[a] * v.col(m)[a];
            }
            k.ger(nu[m], &w, &w, 1.0);
        }
        // Flip modes: pairs (a, b), scaled by the remaining axis c in 3D (no such axis in 2D).
        let sqrt2 = Real::sqrt(2.0);
        for (a, b) in &strain_pairs()[DIM..] {
            let h_f = (mu - kappa * others(*a, *b)).max(2.0 * mu);
            let t = strain_rows_dual(&outer_product(q.col(*a), q.col(*b))) * sqrt2;
            k.ger(h_f, &t, &t, 1.0);
        }
        k
    }
}
