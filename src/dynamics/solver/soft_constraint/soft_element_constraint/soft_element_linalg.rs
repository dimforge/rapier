//! Small dense linear-algebra helpers of the soft rows: the cofactor matrix and the outer product (which glam and glamx lack), and the symmetric eigen-decompositions.

use crate::glamx::MatExt;
use crate::math::{DIM, Matrix, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Eigen-decomposition of a symmetric matrix by cyclic Jacobi, stopped once the off-diagonal part
/// is below 1e-4 of the diagonal: the eigenvalues, and the eigenvectors as columns.
pub(crate) fn symmetric_eigen(m: &Matrix) -> (Vector, Matrix) {
    let mut a = m.transpose().to_cols_array_2d();
    let mut v = Matrix::IDENTITY.to_cols_array_2d();
    const MAX_SWEEPS: usize = 8;
    for _ in 0..MAX_SWEEPS {
        let mut off = 0.0;
        let mut diag = 0.0;
        for p in 0..DIM {
            diag += a[p][p] * a[p][p];
            for q in p + 1..DIM {
                off += a[p][q] * a[p][q];
            }
        }
        if off <= 1.0e-8 * diag {
            break;
        }
        for p in 0..DIM {
            for q in p + 1..DIM {
                let apq = a[p][q];
                if apq == 0.0 {
                    continue;
                }
                // tan φ of the rotation zeroing a_pq (the smaller root), c = cos φ, s = sin φ.
                let d = a[q][q] - a[p][p];
                let r = (d * d + 4.0 * apq * apq).sqrt();
                let t = 2.0 * apq / (d + d.signum() * r);
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = t * c;
                // A ← Jᵀ A J on rows/columns p and q, V ← V J.
                for k in 0..DIM {
                    let akp = a[k][p];
                    let akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for k in 0..DIM {
                    let apk = a[p][k];
                    let aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for row in &mut v {
                    let vp = row[p];
                    let vq = row[q];
                    row[p] = c * vp - s * vq;
                    row[q] = s * vp + c * vq;
                }
            }
        }
    }
    let values = Vector::from_array(core::array::from_fn(|k| a[k][k]));
    // `v[i][k]` is component `i` of eigenvector `k`.
    (values, Matrix::from_cols_array_2d(&v).transpose())
}

/// The largest eigenvalue of a symmetric matrix (glamx's closed form).
#[inline]
pub(crate) fn max_symmetric_eigenvalue(m: &Matrix) -> Real {
    m.symmetric_eigenvalues().max_element()
}

/// The outer product `a bᵀ`.
#[inline]
pub(crate) fn outer_product(a: Vector, b: Vector) -> Matrix {
    #[cfg(feature = "dim2")]
    {
        Matrix::from_cols(a * b.x, a * b.y)
    }
    #[cfg(feature = "dim3")]
    {
        Matrix::from_cols(a * b.x, a * b.y, a * b.z)
    }
}

/// The cofactor matrix `∂det(F)/∂F`.
#[inline]
pub(crate) fn cofactor(f: &Matrix) -> Matrix {
    #[cfg(feature = "dim2")]
    {
        // F = [[a, b], [c, d]] (rows), det = ad - bc, ∂det/∂F = [[d, -c], [-b, a]].
        let (a, c) = (f.x_axis.x, f.x_axis.y);
        let (b, d) = (f.y_axis.x, f.y_axis.y);
        Matrix::from_cols(Vector::new(d, -b), Vector::new(-c, a))
    }
    #[cfg(feature = "dim3")]
    {
        let (f0, f1, f2) = (f.x_axis, f.y_axis, f.z_axis);
        Matrix::from_cols(f1.cross(f2), f2.cross(f0), f0.cross(f1))
    }
}
