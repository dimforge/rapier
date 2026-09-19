//! The tear event and the stress level past which a tear is never paced.

use crate::alloc_prelude::*;
use crate::dynamics::{ImpulseJointHandle, RigidBodyHandle};
use crate::math::{DIM, Real};

use super::super::SoftBodyHandle;

/// A cluster piece a tear left: where a cluster the crack ran through ended up (see
/// [`SoftBodyTearEvent::clusters`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SoftClusterSplit {
    /// The index of the cluster that was split, in the torn body, before the tear.
    pub source_cluster: u32,
    /// The soft body holding the piece.
    pub soft_body: SoftBodyHandle,
    /// The piece's cluster index in that body.
    pub cluster: u32,
    /// The piece's proxy rigid body.
    pub proxy: RigidBodyHandle,
    /// Whether the piece kept the source cluster's proxy (the joints and colliders that were not
    /// moved stay on it); the other pieces got a fresh one.
    pub keeps_proxy: bool,
}

/// A soft body a tear left: the torn body itself, or a body split off it (see
/// [`SoftBodyTearEvent::pieces`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SoftBodyPiece {
    /// The soft body holding the piece.
    pub soft_body: SoftBodyHandle,
    /// The particles of the piece: `particles[i]` is the index, in the torn body after the tear
    /// (the indices the other fields of the event use), of the piece's `i`-th particle.
    pub particles: Vec<u32>,
    /// The clusters of the piece, as `[index in the torn body before the split, index in the
    /// piece]` (a cluster split by the tear is listed under the index its piece got, see
    /// [`SoftBodyTearEvent::clusters`]).
    pub clusters: Vec<[u32; 2]>,
}

/// An impulse joint a tear re-attached: its proxy was split, and the piece closest to the joint's
/// anchor in the rest shape is not the one that kept the proxy (see
/// [`SoftBodyTearEvent::moved_joints`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SoftJointMove {
    /// The joint.
    pub joint: ImpulseJointHandle,
    /// The proxy it was attached to.
    pub from: RigidBodyHandle,
    /// The proxy it is attached to now.
    pub to: RigidBodyHandle,
}

/// What a tear or a cut did to a soft body (see `EventHandler::handle_soft_body_tear_event`,
/// [`crate::dynamics::SoftBodySet::tear`] and [`crate::dynamics::SoftBodySet::cut`]); the
/// indices of `torn_edges`/`torn_cells` predate the tear, the others follow it, before the split.
#[derive(Clone, Debug, Default)]
pub struct SoftBodyTearEvent {
    /// The soft body that tore.
    pub soft_body: SoftBodyHandle,
    /// The particle pairs of the torn edges a crack opened across. In a cut, the edges the blade
    /// meets (see [`crate::dynamics::SoftBody::crossing_elements`]).
    pub torn_edges: Vec<[u32; 2]>,
    /// The vertices of the torn cells a crack opened through (a tear removes no cell). In a cut,
    /// the cells the blade meets.
    pub torn_cells: Vec<[u32; DIM + 1]>,
    /// The particle pairs of the edges removed for straddling an opened crack: bending edges over
    /// a split particle, quad diagonals across an opened triangle side, edges joining two pieces.
    pub removed_edges: Vec<[u32; 2]>,
    /// The particles the tear passed through, duplicated one copy per piece: `(copy, source)`.
    pub split_particles: Vec<(u32, u32)>,
    /// The particles a cut inserted where it crosses a rope, polyline or wire segment: two per
    /// crossed segment, both at the crossing, the first joined to the segment's first particle.
    pub inserted_particles: Vec<u32>,
    /// The soft bodies the torn body came apart into, the piece with the largest rest measure
    /// (which keeps the handle) first, the others new soft bodies remembering their origin. Empty
    /// when nothing split off; else the first entry maps the retained body's compacted particles.
    pub pieces: Vec<SoftBodyPiece>,
    /// The pieces of every cluster the crack split (see
    /// [`crate::dynamics::SoftBody::seeded_components`]): one piece keeps the proxy, the others
    /// get a fresh one. Empty when no cluster was split.
    pub clusters: Vec<SoftClusterSplit>,
    /// The impulse joints moved to another proxy by a cluster split: a joint follows the piece
    /// closest to its anchor in the rest shape.
    pub moved_joints: Vec<SoftJointMove>,
}

impl SoftBodyTearEvent {
    /// The soft bodies the torn body is in after the tear, the one keeping the handle first: the
    /// torn body alone when nothing was split off, the [`Self::pieces`] otherwise.
    pub fn bodies(&self) -> impl Iterator<Item = SoftBodyHandle> + '_ {
        let whole = self.pieces.is_empty().then_some(self.soft_body);
        whole
            .into_iter()
            .chain(self.pieces.iter().map(|piece| piece.soft_body))
    }

    /// Where the particle `particle` of the torn body (a post-tear index, as the other fields
    /// use) is now: the soft body holding it and its index there. `None` for an index the torn
    /// body never had.
    pub fn particle_destination(&self, particle: u32) -> Option<(SoftBodyHandle, u32)> {
        if self.pieces.is_empty() {
            return Some((self.soft_body, particle));
        }
        self.pieces.iter().find_map(|piece| {
            piece
                .particles
                .binary_search(&particle)
                .ok()
                .map(|i| (piece.soft_body, i as u32))
        })
    }

    /// The particle pairs this tear or cut separated: every split copy with its source, and the
    /// two particles a cut inserted on each side of a crossed segment. The seeds of
    /// [`crate::dynamics::SoftBody::seeded_components`] (post-tear indices).
    pub fn seeds(&self) -> Vec<[u32; 2]> {
        let mut seeds: Vec<[u32; 2]> = self
            .split_particles
            .iter()
            .map(|&(copy, source)| [copy, source])
            .collect();
        seeds.extend(
            self.inserted_particles
                .chunks_exact(2)
                .map(|pair| [pair[0], pair[1]]),
        );
        seeds
    }
}

/// The smoothed load (a fraction of the tear threshold) past which a torn edge is never paced
/// by `SoftBodyMaterial::max_tears_per_step`.
pub(super) const CATASTROPHIC_STRESS: Real = 2.0;
