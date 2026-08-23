//! Finite-difference and linear-algebra checks of the soft constraints.

use crate::dynamics::SoftBody;
use crate::dynamics::soft_body::soft_body_shape_matching::extract_rotation;
use crate::math::{DIM, Matrix, Real, Rotation, Vector};
use crate::utils::{DotProduct, RotationOps};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};
use super::*;

#[cfg(feature = "dim2")]
fn base_positions() -> [Vector; 3] {
    [
        Vector::new(0.1, 0.2),
        Vector::new(1.15, -0.2),
        Vector::new(0.3, 0.95),
    ]
}
#[cfg(feature = "dim3")]
fn base_positions() -> [Vector; 4] {
    [
        Vector::new(0.1, 0.2, -0.1),
        Vector::new(1.15, -0.2, 0.0),
        Vector::new(0.3, 0.95, 0.4),
        Vector::new(-0.2, 0.3, 1.2),
    ]
}

/// Finite-difference check of every scalar constraint kind's gradients.
#[test]
fn gradients_match_finite_differences() {
    let mut constraint = SoftScalarConstraint {
        kind: SoftScalarConstraintKind::Distance,
        num_particles: 2,
        writeback: SoftScalarConstraintWriteback::Edge,
        soft_body: 0,
        element: 0,
        solver_ids: [u32::MAX; MAX_CONSTRAINT_PARTICLES],
        pos: [Vector::ZERO; MAX_CONSTRAINT_PARTICLES],
        im: [1.0; MAX_CONSTRAINT_PARTICLES],
        grad: [Vector::ZERO; MAX_CONSTRAINT_PARTICLES],
        rest: 0.3,
        erp_inv_dt: 0.0,
        cfm_coeff: 0.0,
        inv_lhs: 0.0,
        cfm_gain: 0.0,
        rhs: 0.0,
        impulse: 0.0,
        impulse_bounds: [-Real::MAX, Real::MAX],
    };
    #[cfg(feature = "dim2")]
    let kinds = [(SoftScalarConstraintKind::Distance, 2), (SoftScalarConstraintKind::CellVolume, 3)];
    #[cfg(feature = "dim3")]
    let kinds = [
        (SoftScalarConstraintKind::Distance, 2),
        (SoftScalarConstraintKind::Dihedral, 4),
        (SoftScalarConstraintKind::CellVolume, 4),
    ];
    for (kind, n) in kinds {
        constraint.kind = kind;
        constraint.num_particles = n as u8;
        constraint.pos = base_positions();
        constraint.rest = 0.3;
        let c0 = constraint.compute_gradients();
        let grad = constraint.grad;
        let eps = 1.0e-3;
        for k in 0..n {
            for axis in 0..DIM {
                let mut plus = constraint;
                plus.pos[k][axis] += eps;
                let mut minus = constraint;
                minus.pos[k][axis] -= eps;
                let fd = (plus.compute_gradients() - minus.compute_gradients()) / (2.0 * eps);
                let an = grad[k][axis];
                assert!(
                    (fd - an).abs() < 2.0e-3 * (1.0 + an.abs()),
                    "{kind:?}: particle {k} axis {axis}: fd {fd} vs analytic {an} (C = {c0})"
                );
            }
        }
        let sum: Vector = grad[..n].iter().copied().sum();
        assert!(
            sum.length() < 1.0e-5,
            "{kind:?}: gradients don't sum to zero: {sum:?}"
        );
    }
}

/// Finite-difference check of the elastic-cell gradients: the volumetric one through the
/// cofactor path, the strain ones (with the rotation frozen) through the coefficients.
#[test]
fn elastic_cell_gradients_match_finite_differences() {
    let rest = base_positions();
    let inv_rest_matrix = SoftBody::cell_edge_matrix(rest).inverse();
    let mut constraint = SoftElasticConstraint {
        soft_body: 0,
        element: 0,
        solver_ids: [u32::MAX; MAX_CONSTRAINT_PARTICLES],
        pos: rest,
        im: [1.0; MAX_CONSTRAINT_PARTICLES],
        coeffs: SoftElasticConstraint::coefficients(&inv_rest_matrix),
        inv_rest_matrix,
        rotation: Rotation::IDENTITY,
        rot_mat: Matrix::IDENTITY,
        inv_a: StrainMatrix::zeros(),
        strain_cap: StrainVector::repeat(Real::MAX),
        strain: StrainVector::zeros(),
        strain_impulse: StrainVector::zeros(),
        vol_impulse: 0.0,
        model: SoftElasticModel::Corotational(CorotationalConstraint {
            erp_strain: StrainVector::zeros(),
            cfm_strain: StrainVector::zeros(),
            erp_vol: 0.0,
            cfm_coeff_vol: 0.0,
            volumetric: true,
            vol_cap: Real::MAX,
            grad_vol: [Vector::ZERO; MAX_CONSTRAINT_PARTICLES],
            cfm_gain_vol: 0.0,
            rhs_vol: 0.0,
            b: StrainVector::zeros(),
            z: StrainVector::zeros(),
            inv_s: 0.0,
        }),
    };
    let grad_vol_of = |r: &SoftElasticConstraint| match &r.model {
        SoftElasticModel::Corotational(c) => c.grad_vol,
        SoftElasticModel::NeoHookean(_) => unreachable!(),
    };
    // Deform.
    constraint.pos[1] += Vector::splat(0.07);
    constraint.pos[2] -= Vector::splat(0.11);
    let (f0, c_vol) = constraint.compute_gradients();
    let rot = extract_rotation(f0, Rotation::IDENTITY);
    let gv = grad_vol_of(&constraint);
    let strain_of = |r: &mut SoftElasticConstraint| -> [Real; STRAIN_ROWS] {
        let (f, _) = r.compute_gradients();
        let s = rot.inverse().to_mat() * f;
        let mut out = [0.0; STRAIN_ROWS];
        for (i, (a, b)) in strain_pairs().iter().enumerate() {
            let sym = 0.5 * (s.col(*b)[*a] + s.col(*a)[*b]);
            out[i] = if a == b { sym - 1.0 } else { sym };
        }
        out
    };
    let eps = 1.0e-3;
    for k in 0..MAX_CONSTRAINT_PARTICLES {
        for axis in 0..DIM {
            let mut plus = constraint;
            plus.pos[k][axis] += eps;
            let mut minus = constraint;
            minus.pos[k][axis] -= eps;
            let (_, pv) = plus.compute_gradients();
            let (_, mv) = minus.compute_gradients();
            let fd_vol = (pv - mv) / (2.0 * eps);
            assert!(
                (fd_vol - gv[k][axis]).abs() < 2.0e-3 * (1.0 + gv[k][axis].abs()),
                "volumetric: particle {k} axis {axis}: fd {fd_vol} vs {} (C = {c_vol})",
                gv[k][axis]
            );
            // Strain rows: the analytic gradient is in the cell frame, so world `axis` maps to
            // `Rᵀ e_axis`.
            let sp = strain_of(&mut plus);
            let sm = strain_of(&mut minus);
            let mut e = Vector::ZERO;
            e[axis] = 1.0;
            let local = rot.inverse() * e;
            for r in 0..STRAIN_ROWS {
                let fd = (sp[r] - sm[r]) / (2.0 * eps);
                let expected = SoftElasticConstraint::strain_gradient(&constraint.coeffs, r, k).gdot(local);
                assert!(
                    (fd - expected).abs() < 2.0e-3 * (1.0 + expected.abs()),
                    "strain row {r}: particle {k} axis {axis}: fd {fd} vs {expected}"
                );
            }
        }
    }
}

/// The strain block inverse inverts the block (and zeroes inert constraints), and the closed form
/// from `G` used by the Neo-Hookean constraints matches the direct sum.
#[test]
fn strain_block_inverse() {
    let rest = base_positions();
    let inv_rest_matrix = SoftBody::cell_edge_matrix(rest).inverse();
    let coeffs = SoftElasticConstraint::coefficients(&inv_rest_matrix);
    let im = [1.0, 0.5, 2.0, 0.25][..MAX_CONSTRAINT_PARTICLES].to_vec();
    // The block from its definition, `A_rs = Σₚ wₚ gᵣ[p]·gₛ[p]`.
    let mut a = StrainMatrix::zeros();
    for r in 0..STRAIN_ROWS {
        for s in r..STRAIN_ROWS {
            let mut sum = 0.0;
            for p in 0..MAX_CONSTRAINT_PARTICLES {
                sum += im[p]
                    * SoftElasticConstraint::strain_gradient(&coeffs, r, p)
                        .gdot(SoftElasticConstraint::strain_gradient(&coeffs, s, p));
            }
            a[(r, s)] = sum;
            a[(s, r)] = sum;
        }
    }
    let closed_form = SoftElasticConstraint::strain_block(&coeffs, &im);
    assert!((a - closed_form).abs().max() < 1.0e-5 * (1.0 + a.abs().max()));
    let mut g = Matrix::ZERO;
    for p in 0..MAX_CONSTRAINT_PARTICLES {
        for a in 0..DIM {
            for b in 0..DIM {
                g.col_mut(b)[a] += im[p] * coeffs[p][a] * coeffs[p][b];
            }
        }
    }
    let from_g = NeoHookeanConstraint::strain_block_from_g(&g);
    for r in 0..STRAIN_ROWS {
        for s in 0..STRAIN_ROWS {
            assert!(
                (a[(r, s)] - from_g[(r, s)]).abs() < 1.0e-5 * (1.0 + a[(r, s)].abs()),
                "({r},{s}): {} vs {}",
                a[(r, s)],
                from_g[(r, s)]
            );
        }
    }
    let inv = SoftElasticConstraint::invert_block(a, [false; STRAIN_ROWS]);
    let err = (a * inv - StrainMatrix::identity()).abs().max();
    assert!(err < 1.0e-4, "inverse error {err}");
}

/// The Neo-Hookean gradient matches finite differences of the energy, and the Hessian
/// (unprojected where the energy is convex) matches finite differences of the gradient.
#[test]
fn neo_hookean_gradient_and_hessian_match_finite_differences() {
    let (mu, lambda) = (1.3, 2.7);
    let energy = |strain: &StrainVector| -> Real {
        let m = Matrix::IDENTITY + strain_matrix(strain);
        let j = m.determinant();
        let norm2: Real = (0..DIM).map(|k| m.col(k).length_squared()).sum();
        0.5 * mu * (norm2 - DIM as Real) - mu * (j - 1.0) + 0.5 * lambda * (j - 1.0) * (j - 1.0)
    };
    // Moderate strains around rest (convex region) and a strongly compressed cell.
    let cases = [
        StrainVector::from_fn(|r, _| {
            0.05 * (r as Real + 1.0) * if r % 2 == 0 { 1.0 } else { -1.0 }
        }),
        StrainVector::from_fn(|r, _| if r < DIM { -0.4 } else { 0.1 }),
    ];
    let eps = 1.0e-3;
    let perturbed = |strain: &StrainVector, r: usize, delta: Real| {
        let mut out = *strain;
        out[r] += delta;
        out
    };
    let fd_hessian = |strain: &StrainVector| -> StrainMatrix {
        let mut h = StrainMatrix::zeros();
        for r in 0..STRAIN_ROWS {
            let plus = perturbed(strain, r, eps);
            let minus = perturbed(strain, r, -eps);
            let (gp, _) = NeoHookeanConstraint::gradient_and_hessian(mu, lambda, &plus);
            let (gm, _) = NeoHookeanConstraint::gradient_and_hessian(mu, lambda, &minus);
            h.row_mut(r).copy_from(&((gp - gm) / (2.0 * eps)).transpose());
        }
        h
    };
    for strain in &cases {
        let (grad, hess) = NeoHookeanConstraint::gradient_and_hessian(mu, lambda, strain);
        for r in 0..STRAIN_ROWS {
            let fd = (energy(&perturbed(strain, r, eps)) - energy(&perturbed(strain, r, -eps)))
                / (2.0 * eps);
            assert!(
                (fd - grad[r]).abs() < 2.0e-3 * (1.0 + grad[r].abs()),
                "gradient row {r}: fd {fd} vs {}",
                grad[r]
            );
        }
        // The stiffness dominates the tangent (floored modes): diagonal entries and 2x2
        // minors of the difference are non-negative.
        let diff = hess - fd_hessian(strain);
        for r in 0..STRAIN_ROWS {
            let d_rr = diff[(r, r)];
            assert!(
                d_rr >= -5.0e-3 * (1.0 + hess[(r, r)].abs()),
                "({r},{r}): {d_rr}"
            );
            for s in 0..STRAIN_ROWS {
                let minor = d_rr * diff[(s, s)] - diff[(r, s)] * diff[(r, s)];
                assert!(
                    minor >= -1.0e-2 * (1.0 + hess[(r, r)] * hess[(s, s)]).abs(),
                    "minor ({r},{s}): {minor}"
                );
            }
        }
    }
    // Under uniform stretch the volumetric mode is above its floor: the stiffness is the
    // tangent along it.
    let stretched = StrainVector::from_fn(|r, _| if r < DIM { 0.5 } else { 0.0 });
    let (_, hess) = NeoHookeanConstraint::gradient_and_hessian(mu, lambda, &stretched);
    let fd = fd_hessian(&stretched);
    let quad_h = hess.fixed_view::<DIM, DIM>(0, 0).sum();
    let quad_fd = fd.fixed_view::<DIM, DIM>(0, 0).sum();
    assert!(
        (quad_h - quad_fd).abs() < 1.0e-2 * quad_fd.abs(),
        "volumetric tangent under stretch: {quad_h} vs fd {quad_fd}"
    );
    // At rest the Hessian is the linear-elastic block: 2μ + λ on the diagonal rows, λ between
    // them, 4μ on the shears.
    let (grad, hess) = NeoHookeanConstraint::gradient_and_hessian(mu, lambda, &StrainVector::zeros());
    assert!(grad.amax() < 1.0e-6, "nonzero rest gradient {grad:?}");
    for r in 0..STRAIN_ROWS {
        for s in 0..STRAIN_ROWS {
            let expected = if r == s {
                NeoHookeanConstraint::rest_stiffness(mu, lambda - mu, r)
            } else if r < DIM && s < DIM {
                lambda - mu
            } else {
                0.0
            };
            assert!(
                (hess[(r, s)] - expected).abs() < 1.0e-4,
                "rest hessian ({r},{s}) = {} vs {expected}",
                hess[(r, s)]
            );
        }
    }
}

/// The cofactor matrix is the gradient of the determinant: `det(F) F⁻ᵀ` for an invertible `F`.
#[test]
fn cofactor_is_the_determinant_gradient() {
    let f = SoftBody::cell_edge_matrix(base_positions());
    let expected = f.inverse().transpose() * f.determinant();
    let cof = cofactor(&f);
    for k in 0..DIM {
        let err = (cof.col(k) - expected.col(k)).abs().max_element();
        assert!(err < 1.0e-5, "column {k}: {:?} vs {:?}", cof.col(k), expected.col(k));
    }
}

/// The symmetric eigen-decomposition reconstructs the degenerate matrices a resting or uniformly
/// stretched cell produces: identity, diagonal, and repeated eigenvalues with a small shear.
#[test]
fn symmetric_eigen_handles_degenerate_matrices() {
    let mut shear = Matrix::IDENTITY * 1.5;
    shear.col_mut(1)[0] = 1.0e-3;
    shear.col_mut(0)[1] = 1.0e-3;
    let diagonal = Matrix::from_diagonal(Vector::ONE + Vector::X * 0.25);
    for m in [Matrix::IDENTITY, diagonal, shear] {
        let (values, vectors) = symmetric_eigen(&m);
        let rebuilt = vectors * Matrix::from_diagonal(values) * vectors.transpose();
        let orthonormality = vectors.transpose() * vectors;
        for k in 0..DIM {
            let err = (rebuilt.col(k) - m.col(k)).abs().max_element();
            let ortho = (orthonormality.col(k) - Matrix::IDENTITY.col(k)).abs().max_element();
            assert!(err < 1.0e-4 && ortho < 1.0e-4, "{m:?}: column {k}: {err} {ortho}");
        }
        let max = max_symmetric_eigenvalue(&m);
        assert!((max - values.max_element()).abs() < 1.0e-4, "{m:?}: {max} vs {values:?}");
    }
}

