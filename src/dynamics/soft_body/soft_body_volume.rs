//! Volume preservation per piece: the pieces of material enclosed by a closed boundary, each with its own volume constraint.
use super::SoftBody;
use super::soft_body_builder::surface_is_closed;
use crate::alloc_prelude::*;
use crate::math::{DIM, Real, Vector};

/// A piece of a soft body's material enclosed by closed boundary elements (see
/// [`SoftBody::volume_pieces`]): its volume constraint holds the area/volume it encloses at its
/// rest one times the body's volume factor.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftVolumePiece {
    /// The piece's boundary elements (increasing indices into the body's boundary).
    pub(crate) elements: Vec<u32>,
    /// The particles of those elements (increasing).
    pub(crate) particles: Vec<u32>,
    /// The signed area (2D) or volume (3D) the elements enclose at rest.
    pub(crate) rest_volume: Real,
    /// Accumulated volume impulse (warm-start state).
    pub(crate) impulse: Real,
    /// Whether the piece is currently turned inside out (its signed volume has the opposite sign
    /// to the rest one): its pressure target flips with it (a mirrored balloon is the same
    /// balloon).
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) inverted: bool,
}

impl SoftVolumePiece {
    /// The piece's boundary elements, as indices into [`SoftBody::boundary`].
    pub fn elements(&self) -> &[u32] {
        &self.elements
    }

    /// The particles of the piece's boundary elements, in increasing order.
    pub fn particles(&self) -> &[u32] {
        &self.particles
    }

    /// The signed area (2D) or volume (3D) enclosed by the piece at rest.
    pub fn rest_volume(&self) -> Real {
        self.rest_volume
    }

    /// The signed area (2D) or volume (3D) currently enclosed by this piece of `body`.
    pub fn volume(&self, body: &SoftBody) -> Real {
        SoftBody::elements_volume(self.boundary_elements(&body.boundary), |i| {
            body.particles[i as usize].position
        })
    }

    /// The piece's elements, read from the body's `boundary`.
    pub(crate) fn boundary_elements<'a>(
        &'a self,
        boundary: &'a [[u32; DIM]],
    ) -> impl Iterator<Item = &'a [u32; DIM]> + Clone + 'a {
        self.elements.iter().map(move |&e| &boundary[e as usize])
    }
}

impl SoftBody {
    /// Rebuilds the volume pieces after a topology change (rest volumes from the rest positions)
    /// and disables volume preservation if none is left. `remap` maps the previous particle
    /// indices to the current ones (empty: unchanged; `u32::MAX`: removed).
    pub(crate) fn rebuild_volume_pieces(&mut self, remap: &[u32]) {
        self.volume_pieces =
            self.compute_volume_pieces(remap, |i| self.particles[i as usize].rest_position);
        self.volume_preservation &= !self.volume_pieces.is_empty();
    }

    /// The volume pieces of the current topology: the particles connected by edges, cells or
    /// boundary elements, kept when their boundary elements are closed. Each piece takes the
    /// warm-start impulse and orientation of the previous piece holding its first particle.
    pub(crate) fn compute_volume_pieces(
        &self,
        remap: &[u32],
        rest_position: impl Fn(u32) -> Vector,
    ) -> Vec<SoftVolumePiece> {
        fn find(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                let up = parent[parent[i as usize] as usize];
                parent[i as usize] = up;
                i = up;
            }
            i
        }
        fn link(parent: &mut [u32], vertices: &[u32]) {
            let n = parent.len();
            let mut vertices = vertices.iter().copied().filter(|&v| (v as usize) < n);
            let Some(first) = vertices.next() else {
                return;
            };
            let mut root = find(parent, first);
            for v in vertices {
                let other = find(parent, v);
                if other != root {
                    let (low, high) = (root.min(other), root.max(other));
                    parent[high as usize] = low;
                    root = low;
                }
            }
        }

        let n = self.particles.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        for edge in &self.edges {
            link(&mut parent, &edge.vertices);
        }
        for cell in &self.cells {
            link(&mut parent, &cell.vertices);
        }
        for element in &self.boundary {
            link(&mut parent, element);
        }

        // The boundary elements of each connected piece, in boundary order.
        let mut group_of_root = vec![u32::MAX; n];
        let mut groups: Vec<Vec<u32>> = Vec::new();
        for (e, element) in self.boundary.iter().enumerate() {
            let Some(&v) = element.iter().find(|&&v| (v as usize) < n) else {
                continue;
            };
            let root = find(&mut parent, v) as usize;
            if group_of_root[root] == u32::MAX {
                group_of_root[root] = groups.len() as u32;
                groups.push(Vec::new());
            }
            groups[group_of_root[root] as usize].push(e as u32);
        }

        // The previous piece of every current particle, for the warm start.
        let mut previous = vec![u32::MAX; n];
        for (pi, piece) in self.volume_pieces.iter().enumerate() {
            for &q in &piece.particles {
                let p = if remap.is_empty() {
                    q
                } else {
                    remap.get(q as usize).copied().unwrap_or(u32::MAX)
                };
                if let Some(slot) = previous.get_mut(p as usize) {
                    *slot = pi as u32;
                }
            }
        }

        let mut pieces = Vec::new();
        let mut elements: Vec<[u32; DIM]> = Vec::new();
        for group in groups {
            elements.clear();
            elements.extend(group.iter().map(|&e| self.boundary[e as usize]));
            if !surface_is_closed(&elements) {
                continue;
            }
            let mut particles: Vec<u32> = elements.iter().flatten().copied().collect();
            particles.sort_unstable();
            particles.dedup();
            let rest_volume = Self::boundary_volume(&elements, &rest_position);
            let old = self.volume_pieces.get(previous[particles[0] as usize] as usize);
            pieces.push(SoftVolumePiece {
                elements: group,
                particles,
                rest_volume,
                impulse: old.map_or(0.0, |piece| piece.impulse),
                inverted: old.is_some_and(|piece| piece.inverted),
            });
        }
        pieces
    }
}
