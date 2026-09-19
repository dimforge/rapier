//! The `SoftBodySet` struct, its island event, its accessors and its `Index` impls.
use crate::alloc_prelude::*;
use crate::data::arena::Arena;
use crate::dynamics::RigidBodyHandle;
#[allow(unused_imports)]
use crate::dynamics::RigidBodySet;
use crate::dynamics::soft_body::{SoftBody, SoftBodyHandle};
#[allow(unused_imports)]
use crate::geometry::ColliderSet;
use parry::utils::hashmap::HashMap;

/// Deferred island-connectivity event emitted by [`SoftBodySet`] on insertion, removal and
/// attachment changes, drained at the start of the next step.
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftBodyIslandEvent {
    /// The soft body whose attachment links must be updated (relinked if it still exists,
    /// unlinked otherwise).
    pub handle: SoftBodyHandle,
}

/// A set of soft bodies that can be handled by a physics pipeline. Inserting a soft body creates
/// its hidden root rigid body in the [`RigidBodySet`] (see [`SoftBody`]) and its colliders in the
/// [`ColliderSet`]; removing it removes them, so never remove those yourself.
#[derive(Clone, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SoftBodySet {
    pub(crate) bodies: Arena<SoftBody>,
    /// Island-connectivity events (soft body inserted/removed, attachments changed), drained at
    /// the start of the next timestep.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) island_events: Vec<SoftBodyIslandEvent>,
    /// The soft bodies attached to each rigid body, with the attachment's index (rebuilt when
    /// attachments change; the island manager's adjacency).
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) attached_to: HashMap<RigidBodyHandle, Vec<(SoftBodyHandle, u32)>>,
    /// Whether `attached_to` must be rebuilt (a body with attachments was removed, or the set
    /// was deserialized).
    #[cfg_attr(feature = "serde-serialize", serde(skip, default = "default_true"))]
    pub(super) attached_to_stale: bool,
}

#[cfg(feature = "serde-serialize")]
fn default_true() -> bool {
    true
}

impl SoftBodySet {
    /// Creates an empty set.
    pub fn new() -> Self {
        Self::default()
    }

    /// The number of soft bodies in this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// Whether this set is empty.
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Whether the given handle identifies a soft body of this set.
    pub fn contains(&self, handle: SoftBodyHandle) -> bool {
        self.bodies.contains(handle.0)
    }

    /// The soft body with the given handle.
    pub fn get(&self, handle: SoftBodyHandle) -> Option<&SoftBody> {
        self.bodies.get(handle.0)
    }

    /// The soft body with the given handle.
    pub fn get_mut(&mut self, handle: SoftBodyHandle) -> Option<&mut SoftBody> {
        self.bodies.get_mut(handle.0)
    }

    /// Iterates over the soft bodies of this set.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (SoftBodyHandle, &SoftBody)> {
        self.bodies.iter().map(|(h, b)| (SoftBodyHandle(h), b))
    }

    /// Iterates mutably over the soft bodies of this set.
    pub fn iter_mut(&mut self) -> impl ExactSizeIterator<Item = (SoftBodyHandle, &mut SoftBody)> {
        self.bodies.iter_mut().map(|(h, b)| (SoftBodyHandle(h), b))
    }

    /// The soft bodies with a particle attached to the given rigid body, with the attachment's
    /// index in their attachment list (as of the start of the current step).
    pub(crate) fn attached_to(&self, body: RigidBodyHandle) -> &[(SoftBodyHandle, u32)] {
        self.attached_to.get(&body).map_or(&[], |v| v.as_slice())
    }
}

impl core::ops::Index<SoftBodyHandle> for SoftBodySet {
    type Output = SoftBody;

    fn index(&self, index: SoftBodyHandle) -> &SoftBody {
        &self.bodies[index.0]
    }
}

impl core::ops::IndexMut<SoftBodyHandle> for SoftBodySet {
    fn index_mut(&mut self, index: SoftBodyHandle) -> &mut SoftBody {
        &mut self.bodies[index.0]
    }
}
