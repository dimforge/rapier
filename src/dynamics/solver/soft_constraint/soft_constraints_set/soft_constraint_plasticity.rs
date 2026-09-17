//! Plastic flow of the elastic cells from the strain of the last solve, shared by the constraint
//! writeback and the FEM writeback, and the total (elastic plus plastic) stretch a cell tears on.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::dynamics::SoftBody;
use crate::math::{DIM, Matrix, Real, Vector};

use super::super::soft_element_constraint::{
    STRAIN_ROWS, StrainVector, max_symmetric_eigenvalue, strain_matrix,
};

/// Plastic flow of an elastic cell (see `SoftBodyMaterial::plastic_yield`): past the yield the rest
/// shape flows toward the deviatoric stretch at the creep rate (`Dm' = S_p Dm`, unit determinant,
/// `Dm = P Dm₀` bounded by `plastic_max`); inverted cells do not flow. Returns whether it flowed.
pub(crate) fn plastic_flow(
    cell: &mut crate::dynamics::SoftBodyCell,
    strain: &StrainVector,
    inverted: bool,
    rest0: &[Vector; DIM + 1],
    material: &crate::dynamics::SoftBodyMaterial,
    dt: Real,
) -> bool {
    if inverted {
        return false;
    }
    let eps = strain_matrix(strain);
    // The diagonal strain rows come first.
    let trace = strain.fixed_rows::<DIM>(0).sum();
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
    let dm = total * SoftBody::cell_edge_matrix(*rest0);
    let factorial = if DIM == 2 { 2.0 } else { 6.0 };
    let volume = dm.determinant() / factorial;
    if !volume.is_finite() || volume.abs() <= Real::EPSILON {
        return false;
    }
    cell.inv_rest_matrix = dm.inverse();
    cell.rest_volume = volume;
    let increment = frobenius_norm(&(total - cell.plastic_stretch));
    cell.plastic_stretch = total;
    for impulse in &mut cell.impulses[..STRAIN_ROWS] {
        *impulse *= 1.0 - gamma;
    }
    increment > 1.0e-4
}

/// The largest tensile strain of a cell, plastic flow included: the largest principal value of the
/// symmetric part of the total stretch `(I + ε) P` minus one (the tearing criterion, so a creeping
/// cell tears at the strain an elastic one would); without plastic flow, that of `ε`.
pub(crate) fn total_tensile_strain(strain: &StrainVector, plastic_stretch: &Matrix) -> Real {
    let stretch = (Matrix::IDENTITY + strain_matrix(strain)) * *plastic_stretch;
    let sym = (stretch + stretch.transpose()) * 0.5 - Matrix::IDENTITY;
    max_symmetric_eigenvalue(&sym)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{SoftBodyCell, SoftBodyMaterial};
    use crate::math::Rotation;

    /// A unit right simplex cell at rest.
    fn unit_cell() -> (SoftBodyCell, [Vector; DIM + 1]) {
        let mut rest0 = [Vector::ZERO; DIM + 1];
        for k in 0..DIM {
            let mut axis = Vector::ZERO;
            axis[k] = 1.0;
            rest0[k + 1] = axis;
        }
        let volume = SoftBody::cell_volume(rest0);
        let cell = SoftBodyCell {
            vertices: core::array::from_fn(|k| k as u32),
            rest_volume: volume,
            inv_rest_matrix: SoftBody::cell_edge_matrix(rest0).inverse(),
            plastic_stretch: Matrix::IDENTITY,
            impulses: [1.0; STRAIN_ROWS + 1],
            rotation: Rotation::IDENTITY,
            stiffness_scale: 1.0,
            tear_resistance: 1.0,
            color: 0,
            torn: false,
            stress: 0.0,
        };
        (cell, rest0)
    }

    fn clay() -> SoftBodyMaterial {
        SoftBodyMaterial {
            plastic_yield: 0.05,
            plastic_creep: Real::INFINITY,
            plastic_max: 1.0,
            ..Default::default()
        }
    }

    /// A shear past the yield flows the rest shape, keeps its volume and rebuilds the rest
    /// matrix from the initial rest shape: the elastic strain left is the yield.
    #[test]
    fn shear_flows_and_keeps_the_rest_volume() {
        let (mut cell, rest0) = unit_cell();
        let mut strain = StrainVector::zeros();
        // The first off-diagonal row: an `xy` shear of 0.2.
        strain[DIM] = 0.2;
        let flowed = plastic_flow(&mut cell, &strain, false, &rest0, &clay(), 1.0 / 60.0);
        assert!(flowed);
        assert!((cell.rest_volume - SoftBody::cell_volume(rest0)).abs() < 1.0e-5);
        assert!((cell.plastic_stretch.determinant() - 1.0).abs() < 1.0e-5);
        // `Dm = P Dm₀` exactly.
        let dm = cell.plastic_stretch * SoftBody::cell_edge_matrix(rest0);
        let err = frobenius_norm(&(cell.inv_rest_matrix * dm - Matrix::IDENTITY));
        assert!(err < 1.0e-5, "rest matrix drifted: {err}");
        // The strain rows' warm start followed the flow (full creep: nothing elastic left).
        assert!(cell.impulses[..STRAIN_ROWS].iter().all(|i| i.abs() < 1.0e-6));
        assert_eq!(cell.impulses[STRAIN_ROWS], 1.0);
    }

    /// An inverted cell never flows, whatever its strain rows look like.
    #[test]
    fn inverted_cells_do_not_flow() {
        let (mut cell, rest0) = unit_cell();
        let before = cell;
        // A reflection about an oblique plane: no diagonal strain reaches the old `-0.9` guard.
        let mut strain = StrainVector::zeros();
        for k in 0..DIM {
            strain[k] = -2.0 / DIM as Real;
        }
        for k in DIM..STRAIN_ROWS {
            strain[k] = -2.0 / DIM as Real;
        }
        // A partial creep: the full stretch is the reflection itself, which the determinant
        // guard rejects on its own; a fraction of it is a plain squash.
        let material = SoftBodyMaterial {
            plastic_creep: 1.0,
            ..clay()
        };
        assert!(!plastic_flow(&mut cell, &strain, true, &rest0, &material, 0.3));
        assert_eq!(cell.plastic_stretch, before.plastic_stretch);
        assert_eq!(cell.inv_rest_matrix, before.inv_rest_matrix);
        // The same rows on a cell that is not inverted flow, bounded by the plastic maximum.
        let mut flowed = 0;
        for _ in 0..200 {
            flowed += plastic_flow(&mut cell, &strain, false, &rest0, &material, 0.3) as usize;
        }
        assert!(flowed > 0);
        let deviation = frobenius_norm(&(cell.plastic_stretch - Matrix::IDENTITY));
        assert!(deviation <= 1.0 + 1.0e-3, "flow past the maximum: {deviation}");
    }

    /// The tear strain counts the plastic stretch: an unflowed cell reads its largest
    /// principal strain, a flowed one its total stretch.
    #[test]
    fn total_tensile_strain_counts_the_flow() {
        let mut strain = StrainVector::zeros();
        strain[0] = 0.3;
        strain[1] = -0.1;
        assert!((total_tensile_strain(&strain, &Matrix::IDENTITY) - 0.3).abs() < 1.0e-6);
        let mut p = Matrix::IDENTITY;
        p.x_axis[0] = 1.5;
        p.y_axis[1] = 1.0 / 1.5;
        assert!((total_tensile_strain(&StrainVector::zeros(), &p) - 0.5).abs() < 1.0e-6);
        assert!((total_tensile_strain(&strain, &p) - (1.3 * 1.5 - 1.0)).abs() < 1.0e-6);
    }
}
