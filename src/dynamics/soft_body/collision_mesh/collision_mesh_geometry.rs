//! Geometric queries on a collision mesh: inside tests, self-contact exclusion, ghost contacts,
//! element inversion and outward normals.

use crate::math::{DIM, Real, Vector};
use parry::query::PointQuery;
use parry::shape::Segment;
#[cfg(feature = "dim3")]
use parry::shape::Triangle;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::{SoftBody, SoftCollisionMesh};

impl SoftCollisionMesh {
    /// Whether the point lies inside this closed surface, by ray-crossing parity: a brute
    /// scan over the elements (callers use it rarely, on a handful of vertices), robust to
    /// inverted windings since parity ignores orientation.
    pub(crate) fn contains_point_parity(&self, body: &SoftBody, p: Vector) -> bool {
        #[cfg(feature = "dim2")]
        let dir = Vector::new(0.9063, 0.4226);
        #[cfg(feature = "dim3")]
        let dir = Vector::new(0.5341, 0.6432, 0.5487);
        let mut reach: Real = 1.0;
        for v in 0..self.vertex_count() {
            let x = self.vertex(body, v);
            if x.is_finite() {
                reach = reach.max((x - p).length());
            }
        }
        let q = p + dir * (2.0 * reach + 1.0);
        let mut crossings = 0usize;
        for e in 0..self.indices().len() {
            let ids = self.element(e);
            #[cfg(feature = "dim2")]
            {
                let (a, b) = (
                    self.vertex(body, ids[0] as usize),
                    self.vertex(body, ids[1] as usize),
                );
                if a.is_finite()
                    && b.is_finite()
                    && crate::dynamics::soft_body::soft_body_crossing_tests::segments_cross([p, q], [a, b])
                {
                    crossings += 1;
                }
            }
            #[cfg(feature = "dim3")]
            {
                let (a, b, c) = (
                    self.vertex(body, ids[0] as usize),
                    self.vertex(body, ids[1] as usize),
                    self.vertex(body, ids[2] as usize),
                );
                if a.is_finite()
                    && b.is_finite()
                    && c.is_finite()
                    && crate::dynamics::soft_body::soft_body_crossing_tests::segment_crosses_triangle(p, q, a, b, c)
                {
                    crossings += 1;
                }
            }
        }
        crossings % 2 == 1
    }

    /// Whether a self contact between `vertex` and `element` is ignored: the element holds the
    /// vertex or a surface neighbor, or (FEM bodies) sits within the skins' clearance of it at
    /// rest, except across a crack (see `across_crack`) where the material between is gone.
    pub(crate) fn self_contact_excluded(
        &self,
        body: &SoftBody,
        vertex: u32,
        element: &[u32],
        skin: Real,
    ) -> bool {
        if element.contains(&vertex) {
            return true;
        }
    }
    /// Whether a contact on `element_id` at barycentric `weights`, with the other shape's point at
    /// `point`, is a ghost of an internal vertex/edge: the point projects inside a neighboring
    /// element, which reports the real contact, so this one only brings a spurious normal.
    pub(crate) fn contact_is_ghost(
        &self,
        body: &SoftBody,
        element_id: usize,
        weights: &[Real; DIM],
        point: Vector,
    ) -> bool {
        const EPS: Real = 1.0e-3;
        let element = self.element(element_id);
        let mut touched = [u32::MAX; DIM];
        let mut num_touched = 0;
        for (k, v) in element.iter().enumerate() {
            if weights[k] > EPS {
                touched[num_touched] = *v;
                num_touched += 1;
            }
        }
        if num_touched == element.len() {
            // Interior contact.
            return false;
        }
        let touched = &touched[..num_touched];
        let pos = |v: u32| self.vertex(body, v as usize);
        // Candidates: the elements incident to every touched vertex, other than this one.
        let Some(&first) = touched.first() else {
            return false;
        };
        let (start, end) = match (
            self.vertex_elements_offsets.get(first as usize),
            self.vertex_elements_offsets.get(first as usize + 1),
        ) {
            (Some(&s), Some(&e)) => (s as usize, e as usize),
            _ => return false,
        };
        for &candidate in &self.vertex_elements[start..end] {
            if candidate as usize == element_id {
                continue;
            }
            let other = self.element(candidate as usize);
            if !touched.iter().all(|v| other.contains(v)) {
                continue;
            }
            // A segment neighbor is tested as a segment, whatever the dimension.
            if other.len() < DIM {
                let (a, b) = (
                    self.vertex(body, other[0] as usize),
                    self.vertex(body, other[1] as usize),
                );
                let ab = b - a;
                let t = (point - a).dot(ab) / ab.length_squared().max(1.0e-12);
                if t > EPS && t < 1.0 - EPS {
                    return true;
                }
                continue;
            }
            #[cfg(feature = "dim2")]
            {
                let (a, b) = (pos(other[0]), pos(other[1]));
                let ab = b - a;
                let t = (point - a).dot(ab) / ab.length_squared().max(1.0e-12);
                if t > EPS && t < 1.0 - EPS {
                    return true;
                }
            }
            #[cfg(feature = "dim3")]
            {
                let (a, b, c) = (pos(other[0]), pos(other[1]), pos(other[2]));
                let n = (b - a).cross(c - a);
                let nn = n.length_squared().max(1.0e-12);
                let inside = |p: Vector, q: Vector| n.dot((q - p).cross(point - p)) / nn;
                let (u, v, w) = (inside(a, b), inside(b, c), inside(c, a));
                if u > EPS && v > EPS && w > EPS {
                    return true;
                }
            }
        }
        false
    }

    /// Whether the cell owning `element_id` is currently inverted (negative volume relative to
    /// rest): its material is locally inside out, so contacts read from its geometry have a
    /// meaningless side. `false` when no cell backs the element (wires, skinned meshes).
    pub(crate) fn element_cell_inverted(&self, body: &SoftBody, element_id: usize) -> bool {
        if let Some(&cell) = self.element_cell_ids(body).get(element_id) {
            if let Some(cell) = body.cells.get(cell as usize) {
                let x: [Vector; DIM + 1] =
                    core::array::from_fn(|k| body.particles[cell.vertices[k] as usize].position);
                return SoftBody::cell_volume(x) * cell.rest_volume <= 0.0;
            }
        }
        false
    }

    /// Outward normal (not normalized) of an element from the cached vertex positions, only
    /// meaningful for a closed mesh (`None` otherwise, and for an element whose cell is
    /// currently inverted: its winding is mirrored, so it cannot orient its contacts).
    pub(crate) fn element_outward_normal(
        &self,
        body: &SoftBody,
        element_id: usize,
    ) -> Option<Vector> {
        if !self.closed {
            return None;
        }
        // A likely self-crossed loop (an O squeezed into an 8) has part of its winding mirrored;
        // cells still orient each element, but without them contacts stand down to two-sided (a
        // one-sided "expel" read from a mirrored winding drags bodies through the surface).
        if self.orientation_unreliable && (body.cells.is_empty() || self.is_skinned()) {
            return None;
        }
        let element = self.element(element_id);
        // A skinned mesh has no cell of its own to check for inversion; the cells' boundary does.
        if self.element_cell_inverted(body, element_id) {
            return None;
        }
        // Only a closed mesh gets here, and a wire is never closed: the element is a facet.
        debug_assert_eq!(element.len(), DIM);
        let x: [Vector; DIM] = core::array::from_fn(|k| self.vertex(body, element[k] as usize));
        #[cfg(feature = "dim2")]
        let n = {
            let d = x[1] - x[0];
            Vector::new(d.y, -d.x)
        };
        #[cfg(feature = "dim3")]
        let n = (x[1] - x[0]).cross(x[2] - x[0]);
        // A negative rest volume means the mesh is oriented inward; a mesh turned inside out is
        // mirrored as a whole.
        Some(if (self.rest_signed_volume < 0.0) != self.inverted {
            -n
        } else {
            n
        })
    }

}
