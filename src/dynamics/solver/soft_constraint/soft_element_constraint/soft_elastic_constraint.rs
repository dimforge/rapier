//! The elastic-cell block constraint: strain blocks, their inverse, the per-substep update, warm start and the corotational and Neo-Hookean solves.

use crate::dynamics::SoftBody;
use crate::dynamics::soft_body::soft_body_shape_matching::extract_rotation;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::math::{DIM, Matrix, Real, Rotation, Vector};
use crate::utils::{DotProduct, RotationOps};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};
use super::*;

/// Number of strain rows of an elastic cell: the independent components of the symmetric
/// strain tensor.
pub(crate) const STRAIN_ROWS: usize = DIM * (DIM + 1) / 2;

/// One value per strain row (a strain, its rate, or the strain impulses).
pub(crate) type StrainVector = na::SVector<Real, STRAIN_ROWS>;
/// A `STRAIN_ROWS × STRAIN_ROWS` matrix over the strain rows.
pub(crate) type StrainMatrix = na::SMatrix<Real, STRAIN_ROWS, STRAIN_ROWS>;
/// The strain-row gradients of one particle, as the columns of a `DIM × STRAIN_ROWS` matrix.
#[cfg(feature = "fem")]
pub(crate) type StrainJacobian = na::SMatrix<Real, DIM, STRAIN_ROWS>;

/// The `(a, b)` tensor indices of the strain rows: the diagonal first, then `a < b`.
#[inline]
pub(crate) const fn strain_pairs() -> [(usize, usize); STRAIN_ROWS] {
    #[cfg(feature = "dim2")]
    {
        [(0, 0), (1, 1), (0, 1)]
    }
    #[cfg(feature = "dim3")]
    {
        [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)]
    }
}

/// The strain-constraint coordinates of the symmetric part of `m`: `½ (m_ab + m_ba)` for the constraint
/// `(a, b)` (in [`strain_pairs`] order).
#[inline]
pub(crate) fn strain_rows_of(m: &Matrix) -> StrainVector {
    let (x, y) = (m.x_axis, m.y_axis);
    #[cfg(feature = "dim2")]
    {
        StrainVector::new(x.x, y.y, 0.5 * (y.x + x.y))
    }
    #[cfg(feature = "dim3")]
    {
        let z = m.z_axis;
        StrainVector::new(
            x.x,
            y.y,
            z.z,
            0.5 * (y.x + x.y),
            0.5 * (z.x + x.z),
            0.5 * (z.y + y.z),
        )
    }
}

/// The symmetric tensor `T` with `T coeffs[p] = Σᵣ gᵣ[p] λᵣ`: `λᵣ` on the diagonal rows, `½ λᵣ`
/// on both entries of the off-diagonal ones.
#[inline]
fn strain_tensor(lambda: &StrainVector) -> Matrix {
    #[cfg(feature = "dim2")]
    {
        let h = 0.5 * lambda.z;
        Matrix::from_cols(Vector::new(lambda.x, h), Vector::new(h, lambda.y))
    }
    #[cfg(feature = "dim3")]
    {
        let (h01, h02, h12) = (0.5 * lambda.w, 0.5 * lambda.a, 0.5 * lambda.b);
        Matrix::from_cols(
            Vector::new(lambda.x, h01, h02),
            Vector::new(h01, lambda.y, h12),
            Vector::new(h02, h12, lambda.z),
        )
    }
}

/// The symmetric matrix of a strain in strain-constraint coordinates: `ε_ab = ε_ba = εᵣ` for
/// the constraint `(a, b)`.
#[inline]
pub(crate) fn strain_matrix(strain: &StrainVector) -> Matrix {
    let e = strain;
    #[cfg(feature = "dim2")]
    {
        Matrix::from_cols(Vector::new(e.x, e.z), Vector::new(e.z, e.y))
    }
    #[cfg(feature = "dim3")]
    {
        Matrix::from_cols(
            Vector::new(e.x, e.w, e.a),
            Vector::new(e.w, e.y, e.b),
            Vector::new(e.a, e.b, e.z),
        )
    }
}

/// The strain-row coordinates of `ε ↦ m : ε` for a symmetric `m`: `m_aa` on the diagonal rows,
/// `m_ab + m_ba` on the off-diagonal ones (a shear constraint is two entries of `ε`).
#[inline]
pub(crate) fn strain_rows_dual(m: &Matrix) -> StrainVector {
    let (x, y) = (m.x_axis, m.y_axis);
    #[cfg(feature = "dim2")]
    {
        StrainVector::new(x.x, y.y, y.x + x.y)
    }
    #[cfg(feature = "dim3")]
    {
        let z = m.z_axis;
        StrainVector::new(x.x, y.y, z.z, y.x + x.y, z.x + x.z, z.y + y.z)
    }
}

/// `Σₚ aₚ bₚᵀ`: with `a` the particle velocities and `b` the cell coefficients, the velocity
/// gradient of the cell, whose strain-constraint coordinates are the strain rates.
#[inline]
pub(crate) fn outer_sum(
    a: &[Vector; MAX_CONSTRAINT_PARTICLES],
    b: &[Vector; MAX_CONSTRAINT_PARTICLES],
) -> Matrix {
    let mut m = Matrix::ZERO;
    for j in 0..DIM {
        let mut col = Vector::ZERO;
        for p in 0..MAX_CONSTRAINT_PARTICLES {
            col += a[p] * b[p][j];
        }
        *m.col_mut(j) = col;
    }
    m
}

/// The elastic constraints of a cell, solved together as one block: the independent components of
/// the corotated strain `ε = sym(RᵀF) - I` (`R` from the polar decomposition of `F`, extracted
/// before every solve), with the corotational or Neo-Hookean model of [`SoftElasticModel`].
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftElasticConstraint {
    pub soft_body: u32,
    pub element: u32,
    pub solver_ids: [u32; MAX_CONSTRAINT_PARTICLES],
    pub pos: [Vector; MAX_CONSTRAINT_PARTICLES],
    pub im: [Real; MAX_CONSTRAINT_PARTICLES],
    /// `∂F_cj/∂(x_a)_c = coeffs[a][j]` (constraints of `Dm⁻¹`, particle 0 minus their sum).
    pub coeffs: [Vector; MAX_CONSTRAINT_PARTICLES],
    pub inv_rest_matrix: Matrix,
    /// Rotation of the polar decomposition of `F` (warm-started from the previous pass).
    pub rotation: Rotation,
    /// `rotation` as a matrix, refreshed with it at every update: the solves read it.
    pub rot_mat: Matrix,
    /// Corotational: inverse of the softened strain effective-mass block `(A + CFM)⁻¹`.
    /// Neo-Hookean: `(M A + I)⁻¹` with `M = dt (dt K + D)` (see [`NeoHookeanConstraint`]).
    pub inv_a: StrainMatrix,
    /// Per-constraint cap on the strain error fed to the bias while the cell is inverted
    /// (`det F < 0`); non-inverted cells are clamped at 100% strain or this cap when it is larger.
    pub strain_cap: StrainVector,
    /// Strain of the last update (cell frame, clamped).
    pub strain: StrainVector,
    /// Strain impulses (cell frame).
    pub strain_impulse: StrainVector,
    /// Volumetric impulse (corotational only).
    pub vol_impulse: Real,
    pub model: SoftElasticModel,
}

/// The constitutive model of a [`SoftElasticRow`] and its model-specific state.
#[derive(Copy, Clone, Debug)]
pub(crate) enum SoftElasticModel {
    Corotational(CorotationalConstraint),
    NeoHookean(NeoHookeanConstraint),
}

/// Corotational state of a [`SoftElasticRow`]: constant per-row spring coefficients and the
/// volumetric row solved through a Schur complement of the strain block.
#[derive(Copy, Clone, Debug)]
pub(crate) struct CorotationalConstraint {
    pub erp_strain: StrainVector,
    pub cfm_strain: StrainVector,
    pub erp_vol: Real,
    pub cfm_coeff_vol: Real,
    /// Whether the volumetric row is active (a zero Poisson ratio disables it).
    pub volumetric: bool,
    /// Snap-back cap of the volumetric row (see `SoftElasticConstraint::strain_cap`).
    pub vol_cap: Real,
    // Per-pass state.
    pub grad_vol: [Vector; MAX_CONSTRAINT_PARTICLES],
    pub cfm_gain_vol: Real,
    pub rhs_vol: Real,
    /// Coupling of the volumetric row with the strain rows, and its image through `inv_a`.
    pub b: StrainVector,
    pub z: StrainVector,
    pub inv_s: Real,
}

impl SoftElasticConstraint {
    /// Whether a strain component (as last updated) exceeds `min_strain`: the criterion of the
    /// re-sweep after the contacts.
    #[inline]
    pub fn is_strained(&self, min_strain: Real) -> bool {
        self.strain.iter().any(|s| s.abs() > min_strain)
    }

    pub fn max_tensile_strain(&self) -> Real {
        max_tensile_strain(&self.strain)
    }

    /// Deviatoric coefficients of a cell from its inverse rest matrix.
    pub fn coefficients(inv_rest_matrix: &Matrix) -> [Vector; MAX_CONSTRAINT_PARTICLES] {
        let constraints = inv_rest_matrix.transpose();
        let mut coeffs = [Vector::ZERO; MAX_CONSTRAINT_PARTICLES];
        for a in 0..DIM {
            coeffs[a + 1] = constraints.col(a);
            coeffs[0] -= coeffs[a + 1];
        }
        coeffs
    }

    /// Gradient of strain row `r` w.r.t. particle `p`'s velocity, in the cell frame:
    /// `½ (coeffs[p][b] e_a + coeffs[p][a] e_b)`.
    #[inline]
    pub(crate) fn strain_gradient(
        coeffs: &[Vector; MAX_CONSTRAINT_PARTICLES],
        r: usize,
        p: usize,
    ) -> Vector {
        let (a, b) = strain_pairs()[r];
        let mut g = Vector::ZERO;
        if a == b {
            g[a] = coeffs[p][a];
        } else {
            g[a] = 0.5 * coeffs[p][b];
            g[b] = 0.5 * coeffs[p][a];
        }
        g
    }

    /// The strain-constraint gradients of particle `p` rotated by `rotation`: the columns `R gᵣ[p]`.
    #[cfg(feature = "fem")]
    #[inline]
    pub(crate) fn strain_jacobian(
        coeffs: &[Vector; MAX_CONSTRAINT_PARTICLES],
        p: usize,
        rotation: &Matrix,
    ) -> StrainJacobian {
        let mut jacobian = StrainJacobian::zeros();
        for r in 0..STRAIN_ROWS {
            let column = *rotation * Self::strain_gradient(coeffs, r, p);
            jacobian.set_column(r, &na::SVector::<Real, DIM>::from(column));
        }
        jacobian
    }

    /// The strain effective-mass block `A_rs = Σₚ wₚ gᵣ[p]·gₛ[p]`, in closed form from
    /// `G = Σₚ wₚ cₚ cₚᵀ` (see [`NeoHookeanConstraint::strain_block_from_g`]).
    pub fn strain_block(coeffs: &[Vector; MAX_CONSTRAINT_PARTICLES], im: &[Real]) -> StrainMatrix {
        let weighted: [Vector; MAX_CONSTRAINT_PARTICLES] =
            core::array::from_fn(|p| coeffs[p] * im[p]);
        NeoHookeanConstraint::strain_block_from_g(&outer_sum(&weighted, coeffs))
    }

    /// Inverse of a small symmetric positive-definite matrix (Cholesky); constraints flagged inert are
    /// zeroed in the result, and a block that is not positive definite gives the zero matrix.
    pub fn invert_block(mut a: StrainMatrix, inert: [bool; STRAIN_ROWS]) -> StrainMatrix {
        for (r, inert) in inert.iter().enumerate() {
            if *inert {
                a.row_mut(r).fill(0.0);
                a.column_mut(r).fill(0.0);
                a[(r, r)] = 1.0;
            }
        }
        let mut inv = a.cholesky().map_or_else(StrainMatrix::zeros, |chol| chol.inverse());
        for (r, inert) in inert.iter().enumerate() {
            if *inert {
                inv.row_mut(r).fill(0.0);
                inv.column_mut(r).fill(0.0);
            }
        }
        inv
    }

    /// Updates positions, the rotation, the strain, and the model state: the volumetric
    /// gradient, rhs and Schur complement (corotational), or the tangent stiffness and gradient
    /// (Neo-Hookean).
    #[inline]
    pub fn update(&mut self, bodies: &SolverBodies) {
        for k in 0..MAX_CONSTRAINT_PARTICLES {
            let id = self.solver_ids[k];
            if id != u32::MAX {
                self.pos[k] = bodies.get_pose(id).translation;
            }
        }
        let f = self.deformation_gradient();
        self.rotation = extract_rotation(f, self.rotation);
        self.rot_mat = self.rotation.to_mat();
        let inv_rot = self.rot_mat.transpose();
        let s = inv_rot * f;
        // Snap-back caps: an inverted cell recovers under the max corrective velocity over a few
        // substeps; a cell torn far past rest has its strain error clamped at 100% (or the same cap
        // when larger) to keep the bias continuous on un-inversion; ordinary strains are unclamped.
        let inverted = f.determinant() < 0.0;
        let caps = if inverted {
            self.strain_cap
        } else {
            self.strain_cap.map(|cap| cap.max(1.0))
        };
        self.strain =
            strain_rows_of(&(s - Matrix::IDENTITY)).zip_map(&caps, |e, cap| e.clamp(-cap, cap));

        match &mut self.model {
            SoftElasticModel::Corotational(c) => {
                let mut c_vol =
                    Self::volumetric_gradient(&f, &self.inv_rest_matrix, &mut c.grad_vol);
                let cap = if inverted {
                    c.vol_cap
                } else {
                    c.vol_cap.max(1.0)
                };
                c_vol = c_vol.clamp(-cap, cap);

                // Volumetric effective mass and its coupling `bᵣ = Σₚ wₚ gᵣ[p]·(Rᵀ∇C_V)ₚ` with
                // the strain rows (cell frame).
                let mut a_vv = 0.0;
                let mut weighted = [Vector::ZERO; MAX_CONSTRAINT_PARTICLES];
                for p in 0..MAX_CONSTRAINT_PARTICLES {
                    let g = c.grad_vol[p];
                    a_vv += self.im[p] * g.length_squared();
                    weighted[p] = (inv_rot * g) * self.im[p];
                }
                let b = strain_rows_of(&outer_sum(&weighted, &self.coeffs));
                c.cfm_gain_vol = a_vv * c.cfm_coeff_vol;
                c.z = self.inv_a * b;
                let schur = a_vv + c.cfm_gain_vol - b.dot(&c.z);
                c.b = b;
                c.inv_s = if c.volumetric {
                    crate::utils::inv(schur)
                } else {
                    0.0
                };
                c.rhs_vol = c_vol * c.erp_vol;
            }
            SoftElasticModel::NeoHookean(nh) => {
                nh.update(&self.strain, &self.coeffs, &self.im, &mut self.inv_a);
            }
        }
    }

    /// The deformation gradient `F = Ds · Dm⁻¹` from `self.pos`.
    #[inline]
    fn deformation_gradient(&self) -> Matrix {
        let x: [Vector; DIM + 1] = core::array::from_fn(|k| self.pos[k]);
        let ds = SoftBody::cell_edge_matrix(x);
        ds * self.inv_rest_matrix
    }

    /// The volumetric gradient of `C_V = det(F) - 1` per particle; returns `C_V`.
    #[inline]
    fn volumetric_gradient(
        f: &Matrix,
        inv_rest_matrix: &Matrix,
        grad_vol: &mut [Vector; MAX_CONSTRAINT_PARTICLES],
    ) -> Real {
        // ∂C_V/∂F = cofactor(F); through F = Ds · Dm⁻¹, the columns of (∂C/∂F) · Dm⁻ᵀ are the
        // gradients of particles 1..=DIM, particle 0 gets minus their sum.
        let c_vol = f.determinant() - 1.0;
        let dc_dds = cofactor(f) * inv_rest_matrix.transpose();
        let mut g0 = Vector::ZERO;
        for j in 0..DIM {
            grad_vol[j + 1] = dc_dds.col(j);
            g0 -= grad_vol[j + 1];
        }
        grad_vol[0] = g0;
        c_vol
    }

    /// Recomputes `F` and the volumetric gradient from `self.pos`; returns `(F, C_V)`
    /// (corotational constraints only).
    #[cfg(test)]
    pub(super) fn compute_gradients(&mut self) -> (Matrix, Real) {
        let f = self.deformation_gradient();
        let SoftElasticModel::Corotational(c) = &mut self.model else {
            unreachable!()
        };
        let c_vol = Self::volumetric_gradient(&f, &self.inv_rest_matrix, &mut c.grad_vol);
        (f, c_vol)
    }

    #[inline]
    pub fn warmstart(&self, bodies: &mut SolverBodies) {
        match &self.model {
            SoftElasticModel::Corotational(c) => self.apply_impulses(
                bodies,
                &self.strain_impulse,
                Some((&c.grad_vol, self.vol_impulse)),
            ),
            SoftElasticModel::NeoHookean(_) => {
                self.apply_impulses(bodies, &self.strain_impulse, None)
            }
        }
    }

    /// One block Gauss-Seidel iteration.
    #[inline]
    pub fn solve(&mut self, bodies: &mut SolverBodies) {
        match &self.model {
            SoftElasticModel::Corotational(_) => self.solve_corotational(bodies),
            SoftElasticModel::NeoHookean(_) => self.solve_neo_hookean(bodies),
        }
    }

    /// The particle velocities in the cell frame, and the corotational volumetric rate
    /// `Σₚ ∇C_V[p]·vₚ` (zero when `grad_vol` is `None`).
    #[inline]
    fn local_velocities(
        &self,
        bodies: &SolverBodies,
        grad_vol: Option<&[Vector; MAX_CONSTRAINT_PARTICLES]>,
    ) -> ([Vector; MAX_CONSTRAINT_PARTICLES], Real) {
        let inv_rot = self.rot_mat.transpose();
        let mut vel = [Vector::ZERO; MAX_CONSTRAINT_PARTICLES];
        let mut vol_rate = 0.0;
        for k in 0..MAX_CONSTRAINT_PARTICLES {
            let id = self.solver_ids[k];
            if id != u32::MAX {
                let v = bodies.get_vel(id).linear;
                if let Some(grad_vol) = grad_vol {
                    vol_rate += grad_vol[k].gdot(v);
                }
                vel[k] = inv_rot * v;
            }
        }
        (vel, vol_rate)
    }

    #[inline]
    fn solve_corotational(&mut self, bodies: &mut SolverBodies) {
        let SoftElasticModel::Corotational(c) = &self.model else {
            unreachable!()
        };
        let (vel, vol_rate) = self.local_velocities(bodies, Some(&c.grad_vol));
        let r_vol = c.rhs_vol - c.cfm_gain_vol * self.vol_impulse + vol_rate;
        // Strain residuals `ε̇ᵣ + erpᵣ εᵣ - cfmᵣ λᵣ` (cell frame).
        let rate = strain_rows_of(&outer_sum(&vel, &self.coeffs));
        let res = rate + c.erp_strain.component_mul(&self.strain)
            - c.cfm_strain.component_mul(&self.strain_impulse);
        let y = self.inv_a * res;
        // Schur complement: volumetric first, strain corrected by the coupling.
        let d_vol = c.inv_s * (r_vol - c.b.dot(&y));
        let d_strain = y - c.z * d_vol;
        self.strain_impulse += d_strain;
        self.vol_impulse += d_vol;
        self.apply_impulses(bodies, &d_strain, Some((&c.grad_vol, d_vol)));
    }

    /// `Δλ = (M A + I)⁻¹ (M ε̇ + dt g - λ)` (see [`NeoHookeanConstraint`]).
    #[inline]
    fn solve_neo_hookean(&mut self, bodies: &mut SolverBodies) {
        let SoftElasticModel::NeoHookean(nh) = &self.model else {
            unreachable!()
        };
        let (vel, _) = self.local_velocities(bodies, None);
        let rate = strain_rows_of(&outer_sum(&vel, &self.coeffs));
        let d_strain = self.inv_a * (nh.m * rate + nh.dt_grad - self.strain_impulse);
        self.strain_impulse += d_strain;
        self.apply_impulses(bodies, &d_strain, None);
    }

    /// Applies strain impulses (cell frame) and, if given, a volumetric impulse along its
    /// gradients.
    #[inline]
    fn apply_impulses(
        &self,
        bodies: &mut SolverBodies,
        strain: &StrainVector,
        vol: Option<(&[Vector; MAX_CONSTRAINT_PARTICLES], Real)>,
    ) {
        let t = strain_tensor(strain);
        for p in 0..MAX_CONSTRAINT_PARTICLES {
            let id = self.solver_ids[p];
            if id != u32::MAX && (id as usize) < bodies.len() {
                let local = t * self.coeffs[p];
                let dv = match vol {
                    Some((grad_vol, vol)) => self.rot_mat * local + grad_vol[p] * vol,
                    None => self.rot_mat * local,
                };
                bodies.vels[id as usize].linear -= dv * self.im[p];
            }
        }
    }
}
