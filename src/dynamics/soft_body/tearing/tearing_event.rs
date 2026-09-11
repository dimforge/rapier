//! The tear event and the stress level past which a tear is never paced.

use crate::alloc_prelude::*;
use crate::math::{DIM, Real};

use super::super::SoftBodyHandle;

/// What a tear did to a soft body (see `EventHandler::handle_soft_body_tear_event` and
/// [`crate::dynamics::SoftBodySet::tear`]).
///
/// The particle indices of `torn_edges` and `torn_cells` are the ones before the tear; those of
/// `split_particles` and `detached_particles` the ones right after it, before any detached
/// particle was dropped (see [`crate::dynamics::SoftBody::set_drop_detached_particles`]): apply `particle_remap`
/// to get the final ones.
#[derive(Clone, Debug, Default)]
pub struct SoftBodyTearEvent {
    /// The soft body that tore.
    pub soft_body: SoftBodyHandle,
    /// The particle pairs of the removed edges (the ones marked torn, and the ones removed with
    /// the torn cells).
    pub torn_edges: Vec<[u32; 2]>,
    /// The vertices of the removed cells (the ones marked torn, and the ones spanning a torn
    /// edge).
    pub torn_cells: Vec<[u32; DIM + 1]>,
    /// The particles the tear passed through, duplicated one copy per piece: `(copy, source)`.
    pub split_particles: Vec<(u32, u32)>,
    /// The particles the tear left without any element.
    /// The number of connected pieces of the body after the tear, the dropped particles gone
    /// (see [`crate::dynamics::SoftBody::connected_pieces`]).
    pub pieces: Vec<SoftBodyPiece>,
}

/// The smoothed load (a fraction of the tear threshold) past which a torn edge is never paced
/// by `SoftBodyMaterial::max_tears_per_step`.
pub(super) const CATASTROPHIC_STRESS: Real = 2.0;
