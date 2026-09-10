//! Soft-body plasticity: edge flow modes, the rest-length and rest-angle flow rules, the fit of
//! the rest positions to the flowed rest shapes, and the reset. The cells' flow rule runs on the
//! solver's strain rows (`soft_constraints_set::plastic_flow`).
#[cfg(feature = "dim3")]
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
    pub(crate) fn plastic_flow(&mut self, length: Real, material: &SoftBodyMaterial, dt: Real) -> bool {
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
        let signed_yield = if strain > 0.0 { yield_strain } else { -yield_strain };
        let target = length / (1.0 + signed_yield);
        let blend = (material.edge_plastic_creep * dt).min(1.0);
        let initial = self.initial_rest_length();
        let max = material.edge_plastic_max.max(0.0);
        let new_rest = (self.rest_length + (target - self.rest_length) * blend).clamp(
            initial * (1.0 - max).max(Real::EPSILON),
            initial * (1.0 + max),
        );
        let increment = (new_rest - self.rest_length).abs() / initial;
        self.rest_length = new_rest;
        self.plastic_strain = new_rest / initial - 1.0;
        increment > 1.0e-4
    }
}
