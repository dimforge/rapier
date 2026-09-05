//! The vertex-vs-surface pass of a pair of soft surfaces (or of a mesh against itself), with the crossing tests it relies on.

use crate::alloc_prelude::*;

#[cfg(feature = "dim2")]
use crate::dynamics::soft_body_crossing_tests::segments_cross;
use crate::dynamics::{SoftBody, SoftCollisionMesh};
use crate::math::{DIM, Real, Vector};
use crate::utils::DotProduct;
use parry::bounding_volume::BoundingVolume;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_contacts_classify::{classify_inside, classify_inside_self, project_on_element, ring_depths};
use super::soft_contacts_volume::{VolumeSide, fill_depths, patch_vertices, volume_bins, volume_split};
use super::{SelfTangles, Side, SoftDetectionCtx, SoftVertexPass, SoftVertexCandidate, SoftVertexHits};

/// Whether two elements' boundaries cross transversally, whatever their arities (a
/// segment through a triangle covers a wire through a surface; two wires cross with
/// measure zero).
pub(crate) fn elements_cross(pa: &[Vector], pb: &[Vector]) -> bool {
    #[cfg(feature = "dim2")]
    {
        segments_cross([pa[0], pa[1]], [pb[0], pb[1]])
    }
    #[cfg(feature = "dim3")]
    match (pa.len(), pb.len()) {
        (3, 3) => {
            let (ta, tb) = ([pa[0], pa[1], pa[2]], [pb[0], pb[1], pb[2]]);
            crate::dynamics::soft_body_crossing_tests::tri_pair_crosses(
                &[0, 1, 2],
                &ta,
                &[3, 4, 5],
                &tb,
                false,
            )
        }
        (2, 3) => crate::dynamics::soft_body_crossing_tests::segment_crosses_triangle(
            pa[0], pa[1], pb[0], pb[1], pb[2],
        ),
        (3, 2) => crate::dynamics::soft_body_crossing_tests::segment_crosses_triangle(
            pb[0], pb[1], pa[0], pa[1], pa[2],
        ),
        _ => false,
    }
}

/// The positions of an element's vertices (an element has at most `DIM` vertices; the unused
/// entries stay zero).
#[inline]
pub(crate) fn element_points(mesh: &SoftCollisionMesh, element: &[u32]) -> [Vector; DIM] {
    let mut points = [Vector::ZERO; DIM];
    for (k, &v) in element.iter().enumerate().take(DIM) {
        points[k] = mesh.cached_vertex(v as usize);
    }
    points
}

    // each other, the pair's keep-apart rows are wrong-sided by construction and freeze
    // opened), so they stand down around it, like the self rows do around a
    // The two contact skins (a soft surface's is its vertices' thickness; between two pieces of
    // one torn body, capped per pair by the gap at rest, see `rest_gap_skins`); the
    // Crossing repulsion (see `repel_row`): the detected crossing pairs also get rows
            if len >= reach || len < 1.0e-6 {
            }
            let dist = len - skins;
            // Beyond the prediction distance, only a contact closing fast enough to happen
            // `params.dt` is the substep): the rest of the reach is the bodies' motion
            // margin, and a resting vertex must not carry rows (warm-started) against every
            // a ghost of that internal feature (the neighbor carries the actual contact);
        // Deterministic order, and one constraint per surface vertex touched: the elements sharing

        // A foreign vertex that crossed a closed surface: seen from behind (reversed
        // fast incoming vertex also sees the far side of a closed surface as reversed
        // through the speculative reach (its near-side sightings often land next to
