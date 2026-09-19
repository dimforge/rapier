//! The step-constant preparation: the sparsity pattern, the per-element material and shape functions, and the particle state.

use super::super::soft_fem_sparse::BlockMatrix;
use super::SoftFemSystem;
#[cfg(feature = "dim3")]
use super::soft_fem_system_elements::FemDihedral;
use super::soft_fem_system_elements::{CELL_BLOCKS, FemCell, FemSpring, FemVolumeCell};
#[cfg(feature = "dim3")]
use crate::dynamics::solver::soft_constraint::soft_element_constraint::dihedral_gradients;
use crate::dynamics::solver::soft_constraint::soft_element_constraint::{
    MAX_CONSTRAINT_PARTICLES, STRAIN_ROWS, SoftElasticConstraint, StrainMatrix, StrainVector,
};
use crate::dynamics::{SoftBody, SoftBodyCellModel, SoftBodyEdgeKind, SpringCoefficients};
use crate::math::{DIM, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

impl SoftFemSystem {
    /// Updates everything that is constant over a step: the sparsity pattern (rebuilt when the
    /// topology changed), the per-cell material and shape functions, and the particle masses.
    pub fn prepare(&mut self, sb: &SoftBody, slots: &[u32]) {
        let n = sb.particles.len();
        let rebuild = self.matrix.num_rows() != n
            || self.cells.len() != sb.cells.len()
            || self.springs.len() != sb.edges.len()
            || self.topology_version != sb.topology_version();
        if rebuild {
            self.topology_version = sb.topology_version();
            self.rebuild_pattern(sb);
            self.direct = None;
        }

        self.mass.clear();
        self.pinned.clear();
        self.position.clear();
        self.velocity.clear();
        for p in &sb.particles {
            self.mass.push(if p.inv_mass == 0.0 { 0.0 } else { p.mass });
            self.pinned.push(p.inv_mass == 0.0);
            self.position.push(p.position);
            self.velocity.push(p.velocity);
        }
        self.slots.clear();
        self.slots.extend_from_slice(slots);
        self.force.resize(n, Vector::ZERO);
        self.rhs.resize(n, Vector::ZERO);
        self.delta.resize(n, Vector::ZERO);
        self.response_rhs.resize(n, Vector::ZERO);
        self.response.resize(n, Vector::ZERO);
        self.clear_columns();

        let (mu, lambda) = sb.material.lame_parameters();
        let zeta = sb.material.elastic_damping_ratio;
        let elastic = matches!(
            sb.cell_model,
            SoftBodyCellModel::Corotational | SoftBodyCellModel::NeoHookean
        );
        self.cells.clear();
        for c in &sb.cells {
            // A degenerate cell, one with no material, or the `Volume` cell model (a volume
            // element instead, see `prepare_volume_cells`): no elastic element.
            let volume = if elastic && mu > 0.0 {
                c.rest_volume.abs()
            } else {
                0.0
            };
            let (mu, lambda) = (mu * c.stiffness_scale, lambda * c.stiffness_scale);
            let coeffs = SoftElasticConstraint::coefficients(&c.inv_rest_matrix);
            let inv_mass: [Real; MAX_CONSTRAINT_PARTICLES] =
                core::array::from_fn(|k| sb.particles[c.vertices[k] as usize].inv_mass);
            let beta = Self::rayleigh_coefficient(&coeffs, &inv_mass, mu, lambda, volume, zeta);
            self.cells.push(FemCell {
                vertices: c.vertices,
                coeffs,
                inv_rest_matrix: c.inv_rest_matrix,
                mu: mu * volume,
                lambda: lambda * volume,
                beta,
                rotation: c.rotation,
                strain: StrainVector::zeros(),
                inverted: false,
                neo_hookean: sb.cell_model == SoftBodyCellModel::NeoHookean,
                tangent: Self::corotational_hessian(mu * volume, lambda * volume),
                tangent_strain: StrainVector::repeat(Real::MAX),
            });
        }

        self.prepare_springs(sb);
        #[cfg(feature = "dim3")]
        self.prepare_dihedrals(sb);
        self.prepare_volume_cells(sb);
    }

    /// Updates the volume elements of a `SoftBodyCellModel::Volume` body: `volume_softness`
    /// normalized by the cell-volume constraint's rest effective mass `Σ m⁻¹ |∇V|²`, matching the
    /// constraint path's softness exactly.
    fn prepare_volume_cells(&mut self, sb: &SoftBody) {
        self.volume_cells.clear();
        if sb.cell_model != SoftBodyCellModel::Volume {
            return;
        }
        let softness = sb.material.volume_softness;
        for (ci, c) in sb.cells.iter().enumerate() {
            if c.rest_volume == 0.0 {
                continue;
            }
            let rest: [Vector; MAX_CONSTRAINT_PARTICLES] =
                core::array::from_fn(|k| sb.particles[c.vertices[k] as usize].rest_position);
            let grad = SoftBody::cell_volume_gradients(rest);
            let w: Real = (0..MAX_CONSTRAINT_PARTICLES)
                .map(|k| sb.particles[c.vertices[k] as usize].inv_mass * grad[k].length_squared())
                .sum();
            let (stiffness, beta) = Self::spring_stiffness(&softness, w);
            self.volume_cells.push(FemVolumeCell {
                cell: ci as u32,
                rest_volume: c.rest_volume,
                stiffness,
                beta,
            });
        }
    }

    /// Rebuilds the sparsity pattern from the element graph and caches every element's block
    /// slots.
    fn rebuild_pattern(&mut self, sb: &SoftBody) {
        let n = sb.particles.len();
        let cell_pairs = sb.cells.iter().flat_map(|c| {
            (0..MAX_CONSTRAINT_PARTICLES).flat_map(move |a| {
                (a + 1..MAX_CONSTRAINT_PARTICLES).map(move |b| (c.vertices[a], c.vertices[b]))
            })
        });
        let edge_pairs = sb.edges.iter().map(|e| (e.vertices[0], e.vertices[1]));
        #[cfg(feature = "dim3")]
        let dihedral_pairs = sb.dihedrals.iter().flat_map(|d| {
            (0..4).flat_map(move |a| (a + 1..4).map(move |b| (d.vertices[a], d.vertices[b])))
        });
        #[cfg(feature = "dim2")]
        let dihedral_pairs = core::iter::empty();
        self.matrix =
            BlockMatrix::from_pairs(n, cell_pairs.chain(edge_pairs).chain(dihedral_pairs));

        self.cell_blocks.clear();
        for c in &sb.cells {
            let mut blocks = [u32::MAX; CELL_BLOCKS];
            for a in 0..MAX_CONSTRAINT_PARTICLES {
                for b in 0..MAX_CONSTRAINT_PARTICLES {
                    blocks[a * MAX_CONSTRAINT_PARTICLES + b] =
                        self.matrix.block_index(c.vertices[a], c.vertices[b]);
                }
            }
            self.cell_blocks.push(blocks);
        }
    }

    /// Updates the distance elements: `k = ω² / w₀` with the effective mass `w₀` taken at rest
    /// reproduces the constraint path's spring exactly.
    fn prepare_springs(&mut self, sb: &SoftBody) {
        self.springs.clear();
        for e in &sb.edges {
            let softness = e.softness.unwrap_or(match e.kind {
                SoftBodyEdgeKind::Structural => sb.material.edge_softness,
                SoftBodyEdgeKind::Bend => sb.material.bend_softness,
            });
            let w = sb.particles[e.vertices[0] as usize].inv_mass
                + sb.particles[e.vertices[1] as usize].inv_mass;
            let (stiffness, beta) = Self::spring_stiffness(&softness, w);
            let blocks = core::array::from_fn(|k| {
                self.matrix
                    .block_index(e.vertices[k / 2], e.vertices[k % 2])
            });
            self.springs.push(FemSpring {
                vertices: e.vertices,
                blocks,
                rest_length: e.rest_length,
                stiffness,
                beta,
                tension_only: e.tension_only,
            });
        }
    }

    /// Updates the dihedral bending elements.
    #[cfg(feature = "dim3")]
    fn prepare_dihedrals(&mut self, sb: &SoftBody) {
        self.dihedrals.clear();
        let softness = sb.material.bend_softness;
        for d in &sb.dihedrals {
            let pos: [Vector; 4] =
                core::array::from_fn(|k| sb.particles[d.vertices[k] as usize].position);
            let mut grad = [Vector::ZERO; 4];
            dihedral_gradients(&pos, d.rest_angle, &mut grad);
            let w: Real = (0..4)
                .map(|k| sb.particles[d.vertices[k] as usize].inv_mass * grad[k].length_squared())
                .sum();
            let (stiffness, beta) = Self::spring_stiffness(&softness, w);
            let blocks = core::array::from_fn(|k| {
                self.matrix
                    .block_index(d.vertices[k / 4], d.vertices[k % 4])
            });
            self.dihedrals.push(FemDihedral {
                vertices: d.vertices,
                blocks,
                rest_angle: d.rest_angle,
                stiffness,
                beta,
            });
        }
    }

    /// The stiffness `k = ω²/w` and Rayleigh coefficient `β = 2ζ/ω` of a spring-like element with
    /// rest effective mass `w`; a fully pinned element gets no stiffness.
    fn spring_stiffness(softness: &SpringCoefficients<Real>, w: Real) -> (Real, Real) {
        if w <= 0.0 || softness.natural_frequency <= 0.0 {
            return (0.0, 0.0);
        }
        let omega = softness.angular_frequency();
        let stiffness = omega * omega / w;
        let beta = if softness.damping_ratio > 0.0 {
            2.0 * softness.damping_ratio / omega
        } else {
            0.0
        };
        (stiffness, beta)
    }

    /// The Rayleigh coefficient `β = 2ζ / ω` of a cell, `ω` the mean natural frequency of its rest
    /// strain modes; a single coefficient per cell keeps the damping matrix `β K` positive
    /// semi-definite, as the implicit solve needs.
    fn rayleigh_coefficient(
        coeffs: &[Vector; MAX_CONSTRAINT_PARTICLES],
        inv_mass: &[Real; MAX_CONSTRAINT_PARTICLES],
        mu: Real,
        lambda: Real,
        volume: Real,
        zeta: Real,
    ) -> Real {
        if zeta <= 0.0 {
            return 0.0;
        }
        let block = SoftElasticConstraint::strain_block(coeffs, inv_mass);
        let mut sum = 0.0;
        let mut count = 0;
        for r in 0..STRAIN_ROWS {
            let stiffness = Self::rest_stiffness(mu, lambda, r) * volume;
            let omega = (stiffness * block[(r, r)]).sqrt();
            if omega > 0.0 {
                sum += omega;
                count += 1;
            }
        }
        if count == 0 {
            return 0.0;
        }
        2.0 * zeta * count as Real / sum
    }

    /// Rest stiffness of strain row `r` per unit rest volume of the linear-elastic energy
    /// `Ψ = μ‖ε‖² + λ/2 tr(ε)²`: `2μ + λ` on the diagonal rows, `4μ` on the shear rows.
    #[inline]
    fn rest_stiffness(mu: Real, lambda: Real, r: usize) -> Real {
        if r < DIM { 2.0 * mu + lambda } else { 4.0 * mu }
    }

    /// The linear-elastic Hessian `V₀ ∂²Ψ/∂ε²` in strain-constraint coordinates (constant, the
    /// corotational model's tangent).
    fn corotational_hessian(mu: Real, lambda: Real) -> StrainMatrix {
        let mut h = StrainMatrix::zeros();
        h.fixed_view_mut::<DIM, DIM>(0, 0).fill(lambda);
        for r in 0..STRAIN_ROWS {
            h[(r, r)] = Self::rest_stiffness(mu, lambda, r);
        }
        h
    }
}
