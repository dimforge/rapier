//! Soft-body plasticity: edge flow modes, the rest-length and rest-angle flow rules, the fit of
//! the rest positions to the flowed rest shapes, and the reset. The cells' flow rule runs on the
//! solver's strain rows (`soft_constraints_set::plastic_flow`).
use crate::alloc_prelude::*;
#[cfg(feature = "dim3")]
use crate::dynamics::SoftBodyDihedral;
use crate::dynamics::{SoftBody, SoftBodyCellModel, SoftBodyEdge, SoftBodyMaterial};
use crate::math::{DIM, Matrix, Real, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

/// Which strains make a soft-body edge flow plastically (see
/// [`SoftBodyMaterial::edge_plastic_yield`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum SoftEdgePlasticFlow {
    /// Both a squeeze and a stretch past the yield take a permanent set.
    #[default]
    Both,
    /// Only a squeeze sets; a stretched edge always springs back (clay dents, but does not
    /// stay stretched).
    Compression,
    /// Only a stretch sets; a squeezed edge always springs back.
    Tension,
}

/// Jacobi sweeps of the rest-position fit per step.
const REST_FIT_SWEEPS: usize = 8;
/// The fit has settled when no particle moved by more than this fraction of the rest size in a
/// sweep.
const REST_FIT_TOLERANCE: Real = 1.0e-5;

impl SoftBodyEdge {
    /// The permanent set of this edge: how far its rest length has flowed from the length it was
    /// created with, as a fraction of that length (negative: a squeeze that stayed, positive: a
    /// stretch that stayed; see [`SoftBodyMaterial::edge_plastic_yield`]).
    pub fn plastic_strain(&self) -> Real {
        self.plastic_strain
    }

    /// The rest length this edge was created with (its `rest_length` before any plastic flow).
    /// The tear strain is measured against it, so plastic flow counts toward a tear.
    pub fn initial_rest_length(&self) -> Real {
        self.rest_length / (1.0 + self.plastic_strain)
    }

    /// Flows the rest length toward `length` when the strain exceeds the material's edge yield
    /// ([`SoftBodyMaterial::edge_plastic_yield`]; `dt`: the step length). Returns whether the edge
    /// flowed significantly (such a body is kept awake).
    pub(crate) fn plastic_flow(
        &mut self,
        length: Real,
        material: &SoftBodyMaterial,
        dt: Real,
    ) -> bool {
        let yield_strain = material.edge_plastic_yield;
        if yield_strain <= 0.0 || material.edge_plastic_creep <= 0.0 || self.rest_length <= 0.0 {
            return false;
        }
        let strain = length / self.rest_length - 1.0;
        let allowed = match material.edge_plastic_flow {
            SoftEdgePlasticFlow::Both => true,
            SoftEdgePlasticFlow::Compression => strain < 0.0,
            SoftEdgePlasticFlow::Tension => strain > 0.0,
        };
        if !allowed || strain.abs() <= yield_strain {
            return false;
        }
        // The rest length that would leave exactly the yield strain: only the excess flows.
        let signed_yield = if strain > 0.0 {
            yield_strain
        } else {
            -yield_strain
        };
        let target = length / (1.0 + signed_yield);
        let blend = (material.edge_plastic_creep * dt).min(1.0);
        let initial = self.initial_rest_length();
        let max = material.edge_plastic_max.max(0.0);
        let new_rest = (self.rest_length + (target - self.rest_length) * blend).clamp(
            initial * (1.0 - max).max(Real::EPSILON),
            initial * (1.0 + max),
        );
        let increment = (new_rest - self.rest_length).abs() / initial;
        // The warm-start impulse followed the old length error; it scales with what is left of it.
        let error = length - self.rest_length;
        if error != 0.0 {
            self.impulse *= ((length - new_rest) / error).clamp(0.0, 1.0);
        }
        self.rest_length = new_rest;
        self.plastic_strain = new_rest / initial - 1.0;
        increment > 1.0e-4
    }
}

#[cfg(feature = "dim3")]
impl SoftBodyDihedral {
    /// Flows the rest angle toward the current `angle` when the fold exceeds the material's
    /// edge yield, read in radians (see [`SoftBodyMaterial::edge_plastic_yield`]; `dt`: the step
    /// length). Returns whether the dihedral flowed by a significant amount.
    pub(crate) fn plastic_flow(
        &mut self,
        angle: Real,
        material: &SoftBodyMaterial,
        dt: Real,
    ) -> bool {
        let yield_angle = material.edge_plastic_yield;
        if yield_angle <= 0.0 || material.edge_plastic_creep <= 0.0 {
            return false;
        }
        let deviation = angle - self.rest_angle;
        if deviation.abs() <= yield_angle {
            return false;
        }
        // The rest angle that would leave exactly the yield: only the excess flows.
        let target = angle - yield_angle.copysign(deviation);
        let blend = (material.edge_plastic_creep * dt).min(1.0);
        let initial = self.initial_rest_angle();
        let max = material.edge_plastic_max.max(0.0);
        let new_rest = (self.rest_angle + (target - self.rest_angle) * blend)
            .clamp(initial - max, initial + max);
        let increment = (new_rest - self.rest_angle).abs();
        // The warm-start impulse followed the old angle error; it scales with what is left of it.
        self.impulse *= ((angle - new_rest) / deviation).clamp(0.0, 1.0);
        self.rest_angle = new_rest;
        self.plastic_set = new_rest - initial;
        increment > 1.0e-4
    }
}

impl SoftBody {
    /// Moves the particles' rest positions toward the flowed rest shapes of the elements that can
    /// flow (cells with a cell yield, edges with an edge yield): Jacobi sweeps to a least-squares
    /// fit. Returns whether the fit settled; until then it continues at the next step.
    pub(crate) fn fit_rest_positions(&mut self) -> bool {
        let cells_flow = self.material.plastic_yield > 0.0
            && self.cell_model != SoftBodyCellModel::Volume
            && !self.cells.is_empty();
        let edges_flow = self.material.edge_plastic_yield > 0.0 && !self.edges.is_empty();
        if !cells_flow && !edges_flow {
            return true;
        }
        let n = self.particles.len();
        let size = self
            .particles
            .iter()
            .map(|p| p.rest_position.length())
            .fold(0.0, Real::max);
        let tolerance = size.max(Real::EPSILON) * REST_FIT_TOLERANCE;
        let mut sum = vec![Vector::ZERO; n];
        let mut count: Vec<Real> = vec![0.0; n];
        let mut settled = false;
        for _ in 0..REST_FIT_SWEEPS {
            sum.fill(Vector::ZERO);
            count.fill(0.0);
            if cells_flow {
                for c in &self.cells {
                    if c.rest_volume == 0.0 {
                        continue;
                    }
                    let rest0: [Vector; DIM + 1] = core::array::from_fn(|k| {
                        self.particles[c.vertices[k] as usize].initial_rest_position
                    });
                    let dm = c.plastic_stretch * Self::cell_edge_matrix(rest0);
                    let mut offsets = [Vector::ZERO; DIM + 1];
                    let mut centroid = Vector::ZERO;
                    for k in 0..DIM {
                        offsets[k + 1] = dm.col(k);
                    }
                    let mean = offsets.iter().sum::<Vector>() / (DIM + 1) as Real;
                    for &v in &c.vertices {
                        centroid += self.particles[v as usize].rest_position;
                    }
                    centroid /= (DIM + 1) as Real;
                    for (k, &v) in c.vertices.iter().enumerate() {
                        sum[v as usize] += centroid + offsets[k] - mean;
                        count[v as usize] += 1.0;
                    }
                }
            }
            if edges_flow {
                for e in &self.edges {
                    if e.rest_length <= 0.0 {
                        continue;
                    }
                    let [a, b] = e.vertices;
                    let (ra, rb) = (
                        self.particles[a as usize].rest_position,
                        self.particles[b as usize].rest_position,
                    );
                    let d = rb - ra;
                    let len = d.length();
                    if len <= Real::EPSILON {
                        continue;
                    }
                    let half = d * (0.5 * e.rest_length / len);
                    let mid = (ra + rb) * 0.5;
                    sum[a as usize] += mid - half;
                    count[a as usize] += 1.0;
                    sum[b as usize] += mid + half;
                    count[b as usize] += 1.0;
                }
            }
            let mut max_move: Real = 0.0;
            for (i, p) in self.particles.iter_mut().enumerate() {
                if count[i] > 0.0 {
                    let target = sum[i] / count[i];
                    max_move = max_move.max((target - p.rest_position).length());
                    p.rest_position = target;
                }
            }
            if max_move <= tolerance {
                settled = true;
                break;
            }
        }
        // The rest positions stay relative to the rest center of mass.
        let mut com = Vector::ZERO;
        let mut mass = 0.0;
        for p in &self.particles {
            com += p.rest_position * p.mass;
            mass += p.mass;
        }
        if mass > 0.0 {
            com /= mass;
            for p in &mut self.particles {
                p.rest_position -= com;
            }
        }
        settled
    }

    /// Resets every plastic flow (edge rest lengths, dihedral rest angles, cell rest shapes,
    /// particle rest positions) to the creation state; the particles stay put and spring back
    /// elastically from there.
    pub fn reset_plasticity(&mut self) {
        for p in &mut self.particles {
            p.rest_position = p.initial_rest_position;
        }
        for e in &mut self.edges {
            e.rest_length = e.initial_rest_length();
            e.plastic_strain = 0.0;
        }
        #[cfg(feature = "dim3")]
        for d in &mut self.dihedrals {
            d.rest_angle = d.initial_rest_angle();
            d.plastic_set = 0.0;
        }
        for c in &mut self.cells {
            let rest0: [Vector; DIM + 1] = core::array::from_fn(|k| {
                self.particles[c.vertices[k] as usize].initial_rest_position
            });
            let volume = Self::cell_volume(rest0);
            c.plastic_stretch = Matrix::IDENTITY;
            c.rest_volume = volume;
            c.inv_rest_matrix = if volume.abs() > Real::EPSILON {
                Self::cell_edge_matrix(rest0).inverse()
            } else {
                Matrix::ZERO
            };
        }
        self.rest_fit_pending = false;
        self.modified = true;
    }
}
