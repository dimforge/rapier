//! Plastic flow of the elastic cells from the strain of the last solve, shared by the constraint
//! writeback and the FEM writeback, and the total (elastic plus plastic) stretch a cell tears on.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::math::{DIM, Matrix, Real};

use super::super::soft_element_constraint::{StrainVector, strain_matrix};

/// Plastic flow of an elastic cell (see `SoftBodyMaterial::plastic_yield`): past the yield the rest
/// shape flows toward the deviatoric stretch at the creep rate (`Dm' = S_p Dm`, unit determinant,
/// `Dm = P Dm₀` bounded by `plastic_max`); inverted cells do not flow. Returns whether it flowed.
pub(crate) fn plastic_flow(
    cell: &mut crate::dynamics::SoftBodyCell,
    strain: &StrainVector,
    material: &crate::dynamics::SoftBodyMaterial,
    dt: Real,
) -> bool {
    let diagonal = strain.fixed_rows::<DIM>(0);
    if diagonal.iter().any(|e| *e <= -0.9) {
    }
    let eps = strain_matrix(strain);
    let trace = diagonal.sum();
    let dev = eps - Matrix::IDENTITY * (trace / DIM as Real);
    let norm = frobenius_norm(&dev);
    if norm <= material.plastic_yield {
        return false;
    }
    let gamma = ((norm - material.plastic_yield) / norm * material.plastic_creep * dt).min(1.0);
    // The plastic stretch: a fraction of the full stretch `S = I + ε`, normalized to a unit
    // determinant so the flow keeps the rest volume.
    let s_p = Matrix::IDENTITY + eps * gamma;
    let det = s_p.determinant();
    if det <= 1.0e-6 {
        return false;
    }
    let s_p = s_p * (1.0 / det.powf(1.0 / DIM as Real));
    let total = s_p * cell.plastic_stretch;
    let deviation = total - Matrix::IDENTITY;
    let deviation_norm = frobenius_norm(&deviation);
    let total = if deviation_norm > material.plastic_max {
        Matrix::IDENTITY + deviation * (material.plastic_max / deviation_norm)
    } else {
        total
    };
    let total_det = total.determinant();
    if total_det <= 1.0e-6 {
        return false;
    }
    let total = total * (1.0 / total_det.powf(1.0 / DIM as Real));
    // The new rest edge matrix `P' Dm₀` and its signed volume (`det(Dm) / DIM!`).
    cell.inv_rest_matrix = cell.inv_rest_matrix * cell.plastic_stretch * total.inverse();
    let increment = frobenius_norm(&(total - cell.plastic_stretch));
    cell.plastic_stretch = total;
    // Rest volume from the new rest matrix (signed, `det(Dm) / DIM!`).
    let inv_det = cell.inv_rest_matrix.determinant();
    if inv_det != 0.0 {
        let factorial = if DIM == 2 { 2.0 } else { 6.0 };
        cell.rest_volume = 1.0 / (inv_det * factorial);
    }
    increment > 1.0e-4
}

/// Frobenius norm of a matrix.
#[inline]
fn frobenius_norm(m: &Matrix) -> Real {
    let mut norm_sq = 0.0;
    for j in 0..DIM {
        norm_sq += m.col(j).length_squared();
    }
    norm_sq.sqrt()
}
