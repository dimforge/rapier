use crate::dynamics::RigidBodySet;
use crate::geometry::{BroadPhasePairEvent, ColliderHandle, ColliderSet};
use parry::math::Real;

/// An internal index stored in colliders by some broad-phase algorithms.
pub type BroadPhaseProxyIndex = u32;

/// Trait implemented by broad-phase algorithms supported by Rapier.
///
/// The task of a broad-phase algorithm is to detect potential collision pairs, usually based on
/// bounding volumes. The pairs must be conservative: it is OK to create a collision pair if
/// two objects don’t actually touch, but it is incorrect to remove a pair between two objects
/// that are still touching. In other words, it can have false-positive (though these induce
/// some computational overhead on the narrow-phase), but cannot have false-negative.
pub trait BroadPhase: Send + Sync + 'static {
    /// Updates the broad-phase.
    ///
    /// The results must be output through the `events` struct. The broad-phase algorithm is only
    /// required to generate new events (i.e. no need to re-send an `AddPair` event if it was already
    /// sent previously and no `RemovePair` happened since then). Sending redundant events is allowed
    /// but can result in a slight computational overhead.
    ///
    /// The `colliders` set is mutable only to provide access to
    /// [`collider.set_internal_broad_phase_proxy_index`]. Other properties of the collider should
    /// **not** be modified during the broad-phase update.
    ///
    /// # Parameters
    /// - `prediction_distance`: colliders that are not exactly touching, but closer to this
    ///   distance must form a collision pair.
    /// - `colliders`: the set of colliders. Change detection with `collider.needs_broad_phase_update()`
    ///   can be relied on at this stage.
    /// - `modified_colliders`: colliders that are know to be modified since the last update.
    /// - `removed_colliders`: colliders that got removed since the last update. Any associated data
    ///   in the broad-phase should be removed by this call to `update`.
    /// - `events`: the broad-phase’s output. They indicate what collision pairs need to be created
    ///   and what pairs need to be removed. It is OK to create pairs for colliders that don’t
    ///   actually collide (though this can increase computational overhead in the narrow-phase)
    ///   but it is important not to indicate removal of a collision pair if the underlying colliders
    ///   are still touching or closer than `prediction_distance`.
    fn update(
        &mut self,
        dt: Real,
        prediction_distance: Real,
        colliders: &mut ColliderSet,
        bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    );
}
