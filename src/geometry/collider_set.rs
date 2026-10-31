use crate::alloc_prelude::*;
use crate::data::arena::Arena;
use crate::data::{HasModifiedFlag, ModifiedObjects};
use crate::dynamics::{
    IslandManager, RigidBodyHandle, RigidBodySet, SoftBindingError, SoftBodySet, SoftMeshBinding,
};
use crate::geometry::{Collider, ColliderChanges, ColliderHandle, ColliderParent};
use crate::math::{DIM, Pose, Vector};
use core::ops::{Index, IndexMut};

/// A set of modified colliders
pub type ModifiedColliders = ModifiedObjects<ColliderHandle, Collider>;

impl HasModifiedFlag for Collider {
    #[inline]
    fn has_modified_flag(&self) -> bool {
        self.changes.contains(ColliderChanges::IN_MODIFIED_SET)
    }

    #[inline]
    fn set_modified_flag(&mut self) {
        self.changes |= ColliderChanges::IN_MODIFIED_SET;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
/// The collection that stores all colliders (collision shapes) in your physics world.
///
/// Similar to [`RigidBodySet`](crate::dynamics::RigidBodySet), this is the "database" where
/// all your collision shapes live. Each collider can be attached to a rigid body or exist
/// independently.
///
/// # Example
/// ```
/// # use rapier3d::prelude::*;
/// let mut colliders = ColliderSet::new();
/// # let mut bodies = RigidBodySet::new();
/// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
///
/// // Add a standalone collider (no parent body)
/// let handle = colliders.insert(ColliderBuilder::ball(0.5));
///
/// // Or attach it to a body
/// let handle = colliders.insert_with_parent(
///     ColliderBuilder::cuboid(1.0, 1.0, 1.0),
///     body_handle,
///     &mut bodies
/// );
/// ```
pub struct ColliderSet {
    #[cfg_attr(feature = "serde-serialize", serde(with = "interned_colliders"))]
    pub(crate) colliders: Arena<Collider>,
    pub(crate) modified_colliders: ModifiedColliders,
    pub(crate) removed_colliders: Vec<ColliderHandle>,
    #[cfg(feature = "serde-serialize")]
    #[serde(skip)]
    delta_state: snapshot_delta::ColliderDeltaState,
}

/// Serialization of the collider arena with shape interning: each distinct shape is written
/// once into a table, and colliders reference it by index (restoring shares one `Arc` per entry).
///
/// Shapes are grouped by serialized content (see `utils::content_hash`), not by `Arc` identity,
/// which is process-local and would make a restored world's snapshots differ.
#[cfg(feature = "serde-serialize")]
mod interned_colliders {
    use super::Collider;
    use crate::data::arena::{Arena, Entry};
    use crate::geometry::ColliderShape;
    use crate::geometry::collider::InternedCollider;
    use crate::utils::content_hash::{ContentKey, content_key};
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use serde::de::Error;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    use std::collections::HashMap;

    #[derive(Serialize, Deserialize)]
    #[allow(clippy::large_enum_variant)] // Transient serialization form, never stored.
    enum EntryRepr {
        Free {
            next_free: Option<u32>,
        },
        Occupied {
            generation: u32,
            value: InternedCollider,
        },
    }

    #[derive(Serialize, Deserialize)]
    struct ArenaRepr {
        shapes: Vec<ColliderShape>,
        items: Vec<EntryRepr>,
        generation: u32,
        free_list_head: Option<u32>,
        len: u64,
    }

    pub(super) fn serialize<S: Serializer>(
        arena: &Arena<Collider>,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        let (entries, generation, free_list_head, len) = arena.raw_entries();
        let mut shapes = Vec::new();
        // Grouping is by content key; the identity map only memoizes the (relatively
        // costly) content hashing per distinct `Arc`. A shape whose content cannot be
        // hashed (a custom shape refusing serialization) falls back to one table entry
        // per `Arc`, keyed on a unique per-pointer id.
        let mut by_identity: HashMap<*const u8, u32> = HashMap::new();
        let mut by_content: HashMap<ContentKey, u32> = HashMap::new();
        let items = entries
            .iter()
            .map(|entry| match entry {
                Entry::Free { next_free } => EntryRepr::Free {
                    next_free: *next_free,
                },
                Entry::Occupied { generation, value } => {
                    let ptr = Arc::as_ptr(&value.shape.0).cast::<u8>();
                    let id = match by_identity.get(&ptr) {
                        Some(id) => *id,
                        None => {
                            let id = match content_key(&value.shape).map(|k| by_content.entry(k)) {
                                Some(std::collections::hash_map::Entry::Occupied(e)) => *e.get(),
                                Some(std::collections::hash_map::Entry::Vacant(e)) => {
                                    shapes.push(value.shape.clone());
                                    *e.insert(shapes.len() as u32 - 1)
                                }
                                None => {
                                    shapes.push(value.shape.clone());
                                    shapes.len() as u32 - 1
                                }
                            };
                            by_identity.insert(ptr, id);
                            id
                        }
                    };
                    EntryRepr::Occupied {
                        generation: *generation,
                        value: InternedCollider::from_collider(value, id),
                    }
                }
            })
            .collect();
        ArenaRepr {
            shapes,
            items,
            generation,
            free_list_head,
            len: len as u64,
        }
        .serialize(serializer)
    }

    pub(super) fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Arena<Collider>, D::Error> {
        let repr = ArenaRepr::deserialize(deserializer)?;
        let shapes = repr.shapes;
        let items = repr
            .items
            .into_iter()
            .map(|entry| match entry {
                EntryRepr::Free { next_free } => Ok(Entry::Free { next_free }),
                EntryRepr::Occupied { generation, value } => {
                    let shape = shapes
                        .get(value.shape_id() as usize)
                        .ok_or_else(|| {
                            D::Error::custom("collider shape table index out of bounds")
                        })?
                        .clone();
                    Ok(Entry::Occupied {
                        generation,
                        value: value.into_collider(shape),
                    })
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Arena::from_raw_entries(
            items,
            repr.generation,
            repr.free_list_head,
            repr.len as usize,
        ))
    }
}

impl ColliderSet {
    /// Creates a new empty collection of colliders.
    pub fn new() -> Self {
        ColliderSet {
            colliders: Arena::new(),
            modified_colliders: Default::default(),
            removed_colliders: Vec::new(),
            #[cfg(feature = "serde-serialize")]
            delta_state: Default::default(),
        }
    }

    /// Creates a new collection with pre-allocated space for the given number of colliders.
    ///
    /// Use this if you know approximately how many colliders you'll need.
    pub fn with_capacity(capacity: usize) -> Self {
        ColliderSet {
            colliders: Arena::with_capacity(capacity),
            modified_colliders: ModifiedColliders::with_capacity(capacity),
            removed_colliders: Vec::new(),
            #[cfg(feature = "serde-serialize")]
            delta_state: Default::default(),
        }
    }

    /// Fetch the set of colliders modified since the last call to
    /// `take_modified`
    ///
    /// Provides a value that can be passed to the `modified_colliders` argument
    /// of [`BroadPhaseBvh::update`](crate::geometry::BroadPhaseBvh::update).
    ///
    /// Should not be used if this [`ColliderSet`] will be used with a
    /// [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline), which handles
    /// broadphase updates automatically.
    pub fn take_modified(&mut self) -> ModifiedColliders {
        core::mem::take(&mut self.modified_colliders)
    }

    pub(crate) fn set_modified(&mut self, mut modified: ModifiedColliders) {
        // Keep the colliders flagged mid-step (quarantine, soft-body sync): only reuse the
        // returned buffer when nothing was pushed in the meantime.
        if self.modified_colliders.is_empty() {
            self.modified_colliders = modified;
        } else {
            // Still-flagged entries of `modified` cannot also be in our list (`push_once`).
            self.modified_colliders.append(&mut modified);
        }
    }

    /// Fetch the set of colliders removed since the last call to `take_removed`
    ///
    /// Provides a value that can be passed to the `removed_colliders` argument
    /// of [`BroadPhaseBvh::update`](crate::geometry::BroadPhaseBvh::update).
    ///
    /// Should not be used if this [`ColliderSet`] will be used with a
    /// [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline), which handles
    /// broadphase updates automatically.
    pub fn take_removed(&mut self) -> Vec<ColliderHandle> {
        core::mem::take(&mut self.removed_colliders)
    }

    /// Returns a handle that's guaranteed to be invalid.
    ///
    /// Useful as a sentinel/placeholder value.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(crate::INVALID_U32, crate::INVALID_U32)
    }

    /// Iterates over all colliders in this collection.
    ///
    /// Yields `(handle, &Collider)` pairs for each collider (including disabled ones).
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(h, c)| (ColliderHandle(h), c))
    }

    /// Iterates over only the enabled colliders.
    ///
    /// Disabled colliders are excluded from physics simulation and queries.
    pub fn iter_enabled(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders
            .iter()
            .map(|(h, c)| (ColliderHandle(h), c))
            .filter(|(_, c)| c.is_enabled())
    }

    /// Iterates over all colliders with mutable access.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        let modified_colliders = &mut self.modified_colliders;
        self.colliders.iter_mut().map(move |(h, co)| {
            // Lazily: a partially consumed iterator must not drop already-listed colliders.
            modified_colliders.push_once(ColliderHandle(h), co);
            (ColliderHandle(h), co)
        })
    }

    /// Iterates over only the enabled colliders with mutable access.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_enabled_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.iter_mut().filter(|(_, c)| c.is_enabled())
    }

    /// Returns how many colliders are currently in this collection.
    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    /// Returns `true` if there are no colliders in this collection.
    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }

    /// Checks if the given handle points to a valid collider that still exists.
    pub fn contains(&self, handle: ColliderHandle) -> bool {
        self.colliders.contains(handle.0)
    }

    /// Adds a standalone collider (not attached to any body) and returns its handle.
    ///
    /// Most colliders should be attached to rigid bodies using [`insert_with_parent()`](Self::insert_with_parent) instead.
    /// Standalone colliders are useful for sensors or static collision geometry that doesn't need a body.
    pub fn insert(&mut self, coll: impl Into<Collider>) -> ColliderHandle {
        let mut coll = coll.into();
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        coll.reset_internal_references();
        coll.parent = None;
        let handle = ColliderHandle(self.colliders.insert(coll));
        // NOTE: we push unchecked because this is a brand-new collider
        //       so it was initialized with the changed flag but isn’t in
        //       the set yet.
        self.modified_colliders
            .push_unchecked(handle, &mut self.colliders[handle.0]);
        handle
    }

    /// Adds a collider attached to a rigid body and returns its handle.
    ///
    /// This is the most common way to add colliders. The collider's position is relative
    /// to its parent body, so when the body moves, the collider moves with it.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut colliders = ColliderSet::new();
    /// # let mut bodies = RigidBodySet::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// // Create a ball collider attached to a dynamic body
    /// let collider_handle = colliders.insert_with_parent(
    ///     ColliderBuilder::ball(0.5),
    ///     body_handle,
    ///     &mut bodies
    /// );
    /// ```
    pub fn insert_with_parent(
        &mut self,
        coll: impl Into<Collider>,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        let mut coll = coll.into();
        // Make sure the internal links are reset, they may not be
        // if this collider was obtained by cloning another one.
        coll.reset_internal_references();

        if let Some(prev_parent) = &mut coll.parent {
            prev_parent.handle = parent_handle;
        } else {
            coll.parent = Some(ColliderParent {
                handle: parent_handle,
                pos_wrt_parent: coll.pos.0,
            });
        }

        // NOTE: we use `get_mut` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        let parent = bodies
            .get_mut_internal_with_modification_tracking(parent_handle)
            .expect("Parent rigid body not found.");
        let handle = ColliderHandle(self.colliders.insert(coll));
        let coll = self.colliders.get_mut(handle.0).unwrap();
        // NOTE: we push unchecked because this is a brand-new collider
        //       so it was initialized with the changed flag but isn’t in
        //       the set yet.
        self.modified_colliders.push_unchecked(handle, coll);

        parent.add_collider_internal(
            handle,
            coll.parent.as_mut().unwrap(),
            &mut coll.pos,
            &coll.shape,
            &coll.mprops,
        );
        handle
    }

    /// Inserts a collider holding a soft body's deformable collision mesh: a polyline (2D) or
    /// triangle mesh (3D) whose vertices follow the cluster proxy `parent_handle` via `binding`
    /// ([`crate::dynamics::SoftBodyCluster::proxy`]); a failed call leaves both sets untouched.
    pub fn insert_deformable(
        &mut self,
        coll: impl Into<Collider>,
        binding: SoftMeshBinding,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
        soft_bodies: &mut SoftBodySet,
    ) -> Result<ColliderHandle, SoftBindingError> {
        let mut coll = coll.into();
        // The parent must be a live cluster proxy: it says which body and cluster the mesh
        // joins, and its pose places the collider like any attached one.
        let (body_handle, cluster_id, frame) = {
            let rb = bodies
                .get(parent_handle)
                .ok_or(SoftBindingError::NotAClusterProxy)?;
            let body = rb.soft_body().ok_or(SoftBindingError::NotAClusterProxy)?;
            (body, rb.soft_cluster, rb.pos.position)
        };
        let sb = soft_bodies
            .get_mut(body_handle)
            .ok_or(SoftBindingError::NotAClusterProxy)?;
        if sb
            .cluster(cluster_id)
            .is_none_or(|cluster| cluster.proxy() != parent_handle)
        {
            return Err(SoftBindingError::NotAClusterProxy);
        }

        let (vertices, indices) = mesh_geometry(coll.shape())?;
        // The shape is kept as given, in the collider's own frame: the binding reads its
        // vertices where the collider's pose relative to the proxy puts them.
        let pos_wrt_parent = coll
            .parent
            .as_ref()
            .map_or(coll.pos.0, |parent| parent.pos_wrt_parent);
        let pose = frame * pos_wrt_parent;
        let vertices: Vec<Vector> = vertices.iter().map(|v| pose * *v).collect();
        let mut mesh = sb.bind_mesh(cluster_id, &binding, vertices, indices)?;
        // The shape's `ORIENTED` flag says whether the closed mesh encloses solid matter.
        #[cfg(feature = "dim2")]
        let oriented = coll.shape().as_polyline().is_some_and(|polyline| {
            polyline
                .flags()
                .contains(parry::shape::PolylineFlags::ORIENTED)
        });
        #[cfg(feature = "dim3")]
        let oriented = coll.shape().as_trimesh().is_some_and(|trimesh| {
            trimesh
                .flags()
                .contains(parry::shape::TriMeshFlags::ORIENTED)
        });
        mesh.oriented = oriented;
        coll.set_density(0.0);
        // The contact skin is the thickness of the mesh's vertices in the soft contact
        // passes: a collider left without one gets the body's particle radius.
        if coll.contact_skin() <= 0.0 {
            coll.set_contact_skin(sb.particle_radius());
        }

        let id = sb.push_mesh(cluster_id, mesh);
        let handle = self.insert_with_parent(coll, parent_handle, bodies);
        let co = self.index_mut_internal(handle);
        co.deformable_mesh_ref = Some(crate::dynamics::SoftMeshRef {
            body: body_handle,
            id,
        });
        soft_bodies[body_handle].set_mesh_collider(id, handle);
        Ok(handle)
    }

    /// Changes which rigid body a collider is attached to, or detaches it completely.
    ///
    /// Use this to move a collider from one body to another, or to make it standalone.
    ///
    /// # Parameters
    /// * `new_parent_handle` - `Some(handle)` to attach to a body, `None` to make standalone
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut colliders = ColliderSet::new();
    /// # let mut bodies = RigidBodySet::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let other_body = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let collider_handle = colliders.insert_with_parent(ColliderBuilder::ball(0.5).build(), body_handle, &mut bodies);
    /// // Detach collider from its current body
    /// colliders.set_parent(collider_handle, None, &mut bodies);
    ///
    /// // Attach it to a different body
    /// colliders.set_parent(collider_handle, Some(other_body), &mut bodies);
    /// ```
    pub fn set_parent(
        &mut self,
        handle: ColliderHandle,
        new_parent_handle: Option<RigidBodyHandle>,
        bodies: &mut RigidBodySet,
    ) {
        if let Some(collider) = self.get_mut(handle) {
            let curr_parent = collider.parent.map(|p| p.handle);
            if new_parent_handle == curr_parent {
                return; // Nothing to do, this is the same parent.
            }

            collider.changes |= ColliderChanges::PARENT;

            if let Some(parent_handle) = curr_parent {
                if let Some(rb) = bodies.get_mut(parent_handle) {
                    rb.remove_collider_internal(handle);
                }
            }

            match new_parent_handle {
                Some(new_parent_handle) => {
                    if let Some(parent) = &mut collider.parent {
                        parent.handle = new_parent_handle;
                    } else {
                        collider.parent = Some(ColliderParent {
                            handle: new_parent_handle,
                            pos_wrt_parent: Pose::IDENTITY,
                        })
                    };

                    if let Some(rb) = bodies.get_mut(new_parent_handle) {
                        rb.add_collider_internal(
                            handle,
                            collider.parent.as_ref().unwrap(),
                            &mut collider.pos,
                            &collider.shape,
                            &collider.mprops,
                        );
                    }
                }
                None => collider.parent = None,
            }
        }
    }

    /// Removes a collider from the world.
    ///
    /// The collider is detached from its parent body (if any) and removed from all
    /// collision detection structures. Returns the removed collider if it existed.
    ///
    /// # Parameters
    /// * `wake_up` - If `true`, wakes up the parent body (useful when collider removal
    ///   changes the body's mass or collision behavior significantly)
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut colliders = ColliderSet::new();
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut islands = IslandManager::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic().build());
    /// # let mut soft_bodies = SoftBodySet::new();
    /// # let handle = colliders.insert_with_parent(ColliderBuilder::ball(0.5).build(), body_handle, &mut bodies);
    /// if let Some(collider) = colliders.remove(
    ///     handle,
    ///     &mut islands,
    ///     &mut bodies,
    ///     &mut soft_bodies,
    ///     true  // Wake up the parent body
    /// ) {
    ///     println!("Removed collider with shape: {:?}", collider.shared_shape());
    /// }
    /// ```
    pub fn remove(
        &mut self,
        handle: ColliderHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        soft_bodies: &mut SoftBodySet,
        wake_up: bool,
    ) -> Option<Collider> {
        let collider = self.remove_internal(handle, islands, bodies, wake_up)?;
        // A removed deformable collider takes its mesh with it: the soft body keeps simulating,
        // without that mesh's collisions.
        if let Some(mesh) = collider.deformable_mesh_ref() {
            if let Some(sb) = soft_bodies.get_mut(mesh.body) {
                sb.remove_mesh(mesh.id);
            }
        }
        Some(collider)
    }

    /// Like [`Self::remove`], for internal callers that maintain the soft-body back-references
    /// themselves.
    pub(crate) fn remove_internal(
        &mut self,
        handle: ColliderHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        wake_up: bool,
    ) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;

        /*
         * Delete the collider from its parent body.
         */
        // NOTE: we use `get_mut_internal_with_modification_tracking` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        if let Some(parent) = &collider.parent {
            if let Some(parent_rb) =
                bodies.get_mut_internal_with_modification_tracking(parent.handle)
            {
                parent_rb.remove_collider_internal(handle);

                if wake_up {
                    islands.wake_up(bodies, parent.handle, true);
                }
            }
        }

        /*
         * Publish removal.
         */
        self.removed_colliders.push(handle);

        Some(collider)
    }

    /// Gets a collider by its index without knowing the generation number.
    ///
    /// ⚠️ **Advanced/unsafe usage** - prefer [`get()`](Self::get) instead! See [`RigidBodySet::get_unknown_gen`] for details.
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&Collider, ColliderHandle)> {
        self.colliders
            .get_unknown_gen(i)
            .map(|(c, h)| (c, ColliderHandle(h)))
    }

    /// Gets a mutable reference to a collider by its index without knowing the generation.
    ///
    /// ⚠️ **Advanced/unsafe usage** - prefer [`get_mut()`](Self::get_mut) instead!
    /// suffer form the ABA problem.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_unknown_gen_mut(&mut self, i: u32) -> Option<(&mut Collider, ColliderHandle)> {
        let (collider, handle) = self.colliders.get_unknown_gen_mut(i)?;
        let handle = ColliderHandle(handle);
        self.modified_colliders.push_once(handle, collider);
        Some((collider, handle))
    }

    /// Gets a read-only reference to the collider with the given handle.
    ///
    /// Returns `None` if the handle is invalid or the collider was removed.
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    /// Gets a mutable reference to the collider with the given handle.
    ///
    /// Returns `None` if the handle is invalid or the collider was removed.
    /// Use this to modify collider properties like friction, restitution, sensor status, etc.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        self.modified_colliders.push_once(handle, result);
        Some(result)
    }

    /// Gets mutable references to two different colliders at once.
    ///
    /// Useful when you need to modify two colliders simultaneously. If both handles
    /// are the same, only the first value will be `Some`.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_pair_mut(
        &mut self,
        handle1: ColliderHandle,
        handle2: ColliderHandle,
    ) -> (Option<&mut Collider>, Option<&mut Collider>) {
        if handle1 == handle2 {
            (self.get_mut(handle1), None)
        } else {
            let (mut co1, mut co2) = self.colliders.get2_mut(handle1.0, handle2.0);
            if let Some(co1) = co1.as_deref_mut() {
                self.modified_colliders.push_once(handle1, co1);
            }
            if let Some(co2) = co2.as_deref_mut() {
                self.modified_colliders.push_once(handle2, co2);
            }
            (co1, co2)
        }
    }

    pub(crate) fn index_mut_internal(&mut self, handle: ColliderHandle) -> &mut Collider {
        &mut self.colliders[handle.0]
    }

    /// Like [`Self::index_mut_internal`] without stamping the slot for snapshot deltas.
    ///
    /// For the parallel loops writing disjoint slots through a shared pointer; their writes
    /// are stamped by the pipeline's active-set stamping instead.
    #[cfg(feature = "parallel")]
    pub(crate) fn index_mut_unstamped(&mut self, handle: ColliderHandle) -> &mut Collider {
        self.colliders
            .get_mut_unstamped(handle.0)
            .expect("No element at index")
    }

    pub(crate) fn get_mut_internal(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    #[allow(dead_code)]
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: ColliderHandle,
    ) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        self.modified_colliders.push_once(handle, result);
        Some(result)
    }
}

impl Index<crate::data::Index> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: crate::data::Index) -> &Collider {
        &self.colliders[index]
    }
}

impl Index<ColliderHandle> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: ColliderHandle) -> &Collider {
        &self.colliders[index.0]
    }
}

#[cfg(not(feature = "dev-remove-slow-accessors"))]
impl IndexMut<ColliderHandle> for ColliderSet {
    fn index_mut(&mut self, handle: ColliderHandle) -> &mut Collider {
        let collider = &mut self.colliders[handle.0];
        self.modified_colliders.push_once(handle, collider);
        collider
    }
}

/// The vertices (world space) and elements of a deformable collider's shape.
fn mesh_geometry(
    shape: &dyn crate::geometry::Shape,
) -> Result<(Vec<Vector>, Vec<[u32; DIM]>), SoftBindingError> {
    #[cfg(feature = "dim2")]
    let (deformable, geometry) = {
        use parry::shape::PolylineFlags;
        let polyline = shape
            .as_polyline()
            .ok_or(SoftBindingError::UnsupportedShape)?;
        (
            polyline.flags().contains(PolylineFlags::DEFORMABLE),
            (polyline.vertices().to_vec(), polyline.indices().to_vec()),
        )
    };
    #[cfg(feature = "dim3")]
    let (deformable, geometry) = {
        use parry::shape::TriMeshFlags;
        let trimesh = shape
            .as_trimesh()
            .ok_or(SoftBindingError::UnsupportedShape)?;
        (
            trimesh.flags().contains(TriMeshFlags::DEFORMABLE),
            (trimesh.vertices().to_vec(), trimesh.indices().to_vec()),
        )
    };
    if !deformable {
        return Err(SoftBindingError::NotDeformable);
    }
    let (vertices, indices) = geometry;
    if vertices.is_empty() || indices.is_empty() {
        return Err(SoftBindingError::DegenerateMesh);
    }
    Ok((vertices, indices))
}

/// Snapshot deltas of the collider set. Shapes heavier than a few primitives (meshes, hulls,
/// heightfields, compounds...) are referenced by content key and only sent when the receiver
/// cannot hold them yet, so colliders on moving bodies don't resend their geometry every tick.
#[cfg(feature = "serde-serialize")]
mod snapshot_delta {
    use super::*;
    use crate::data::arena::ArenaDelta;
    use crate::data::change_stamps::stamp_is_newer;
    use crate::data::delta::{ChangeTracker, DeltaComponent, DeltaError};
    use crate::geometry::ColliderShape;
    use crate::geometry::collider::InternedCollider;
    use crate::utils::content_hash::{ContentKey, content_key};
    use alloc::sync::Arc;
    use parry::shape::ShapeType;
    use std::collections::HashMap;

    /// Whether a shape is worth referencing by content key instead of being sent inline.
    fn is_heavy(co: &Collider) -> bool {
        // Deformable surfaces change every frame, and holding extra references to them would
        // make every in-place deformation copy the mesh.
        co.deformable_mesh_ref.is_none()
            && !matches!(
                co.shape.shape_type(),
                ShapeType::Ball
                    | ShapeType::Cuboid
                    | ShapeType::Capsule
                    | ShapeType::Segment
                    | ShapeType::Triangle
                    | ShapeType::HalfSpace
                    | ShapeType::RoundCuboid
                    | ShapeType::RoundTriangle
            )
            && !is_light_3d(co.shape.shape_type())
    }

    #[cfg(feature = "dim3")]
    fn is_light_3d(ty: ShapeType) -> bool {
        matches!(
            ty,
            ShapeType::Cylinder | ShapeType::Cone | ShapeType::RoundCylinder | ShapeType::RoundCone
        )
    }

    #[cfg(feature = "dim2")]
    fn is_light_3d(_: ShapeType) -> bool {
        false
    }

    /// The content key of a heavy shape, memoized per arena slot by the collider's
    /// process-unique `shape_revision` (holding the shape would make `shape_mut` copy it).
    #[derive(Copy, Clone, Debug)]
    struct SlotKey {
        revision: u32,
        key: ContentKey,
    }

    /// Per-set delta bookkeeping (never serialized).
    #[derive(Clone, Debug, Default)]
    pub(super) struct ColliderDeltaState {
        tracker: ChangeTracker,
        /// Sender side: the tick up to which `slot_keys`/`held` reflect the stamped slots.
        synced: Option<u32>,
        slot_keys: Vec<Option<SlotKey>>,
        /// Per heavy shape content: the tick since which some collider continuously held it,
        /// and how many colliders hold it now.
        held: HashMap<ContentKey, (u32, u32)>,
        /// Receiver side: shapes the receiver holds or received, by content.
        received: Option<HashMap<ContentKey, ColliderShape>>,
    }

    impl ColliderDeltaState {
        /// Brings the sender-side shape index up to date with the slots stamped since the last
        /// sync (every slot on the first one).
        fn sync(&mut self, colliders: &Arena<Collider>) {
            let (entries, ..) = colliders.raw_entries();
            let stamps = &colliders.stamps;
            if self.slot_keys.len() < entries.len() {
                self.slot_keys.resize(entries.len(), None);
            }
            // Colliders sharing a shape are hashed once per sync (the pointers stay valid
            // since the colliders hold the shapes during the call).
            let mut keys_by_ptr: HashMap<*const (), Option<ContentKey>> = HashMap::new();
            for i in 0..entries.len() {
                if let Some(synced) = self.synced {
                    if !stamp_is_newer(stamps.stamp(i), synced) {
                        continue;
                    }
                }
                let current = match &entries[i] {
                    crate::data::arena::Entry::Occupied { value, .. } if is_heavy(value) => {
                        Some(value)
                    }
                    _ => None,
                };
                let unchanged = match (&self.slot_keys[i], current) {
                    (Some(memo), Some(co)) => memo.revision == co.shape_revision,
                    (None, None) => true,
                    _ => false,
                };
                if unchanged {
                    continue;
                }
                let now = stamps.stamp_now();
                if let Some(co) = current {
                    let key = *keys_by_ptr
                        .entry(Arc::as_ptr(&co.shape.0) as *const ())
                        .or_insert_with(|| content_key(&co.shape));
                    if let Some(key) = key {
                        // Count the new holder before releasing the old one, so content staying
                        // in this slot stays continuously held.
                        self.held.entry(key).or_insert((now, 0)).1 += 1;
                        if let Some(old) = self.slot_keys[i].take() {
                            self.release(&old.key);
                        }
                        self.slot_keys[i] = Some(SlotKey {
                            revision: co.shape_revision,
                            key,
                        });
                        continue;
                    }
                }
                if let Some(old) = self.slot_keys[i].take() {
                    self.release(&old.key);
                }
            }
            let truncated = entries.len().min(self.slot_keys.len());
            let dropped: Vec<_> = self.slot_keys.drain(truncated..).flatten().collect();
            for old in dropped {
                self.release(&old.key);
            }
            self.synced = Some(stamps.stamp_now());
        }

        fn release(&mut self, key: &ContentKey) {
            if let Some(held) = self.held.get_mut(key) {
                held.1 -= 1;
                if held.1 == 0 {
                    self.held.remove(key);
                }
            }
        }

        /// Whether a receiver at `since` already holds a shape of content `key`: some collider
        /// held it continuously since that tick (every issued tick is a sync point).
        fn receiver_holds(&self, key: &ContentKey, since: u32) -> bool {
            self.held
                .get(key)
                .is_some_and(|(held_since, _)| !stamp_is_newer(*held_since, since))
        }

        /// The receiver-side registry, built from this set's own colliders on first use.
        fn received(
            &mut self,
            colliders: &Arena<Collider>,
        ) -> &mut HashMap<ContentKey, ColliderShape> {
            self.received.get_or_insert_with(|| {
                let mut map = HashMap::new();
                for (_, co) in colliders.iter() {
                    if is_heavy(co) {
                        if let Some(key) = content_key(&co.shape) {
                            map.entry(key).or_insert_with(|| co.shape.clone());
                        }
                    }
                }
                map
            })
        }
    }

    /// A shape of a delta's table.
    #[derive(Clone, Debug, Serialize, Deserialize)]
    enum DeltaShape {
        /// A light (or unhashable) shape, sent as is.
        Inline(ColliderShape),
        /// A heavy shape by content key, with its bytes when the receiver may not hold it.
        Keyed {
            key: ContentKey,
            shape: Option<ColliderShape>,
        },
    }

    /// The content of a [`ColliderSet`] delta.
    #[doc(hidden)]
    #[derive(Clone, Debug, Serialize, Deserialize)]
    pub struct ColliderSetDelta {
        shapes: Vec<DeltaShape>,
        colliders: ArenaDelta<InternedCollider>,
        modified_colliders: ModifiedColliders,
        removed_colliders: Vec<ColliderHandle>,
    }

    impl DeltaComponent for ColliderSet {
        type Payload = ColliderSetDelta;

        fn change_tracker_mut(&mut self) -> &mut ChangeTracker {
            &mut self.delta_state.tracker
        }

        fn change_tracker(&self) -> &ChangeTracker {
            &self.delta_state.tracker
        }

        fn set_stamp_tick(&mut self, now: u32) {
            self.colliders.stamps.set_now(now);
            if now == 0 {
                let tracker = core::mem::take(&mut self.delta_state.tracker);
                self.delta_state = ColliderDeltaState {
                    tracker,
                    received: self.delta_state.received.take(),
                    ..Default::default()
                };
            }
        }

        fn sync_before_tick(&mut self) {
            // Every issued tick must be a sync point: `receiver_holds` relies on it.
            self.delta_state.sync(&self.colliders);
        }

        fn collect_delta(&mut self, since: Option<u32>) -> ColliderSetDelta {
            // Exhaustive: a field added later must be classified here (sent by deltas, or not state).
            let ColliderSet {
                colliders: _,
                modified_colliders: _,
                removed_colliders: _,
                delta_state: _,
            } = self;
            self.delta_state.sync(&self.colliders);
            let state = &self.delta_state;
            let colliders = &self.colliders;
            let mut shapes = Vec::new();
            let mut keyed: HashMap<ContentKey, u32> = HashMap::new();
            let mut inline: HashMap<*const (), u32> = HashMap::new();
            let delta = colliders.collect_delta_with(since, |i, co| {
                // Up to date after the sync above.
                let key = state.slot_keys[i].map(|memo| memo.key);
                let id = match key {
                    Some(key) => *keyed.entry(key).or_insert_with(|| {
                        let held = since.is_some_and(|since| state.receiver_holds(&key, since));
                        shapes.push(DeltaShape::Keyed {
                            key,
                            shape: (!held).then(|| co.shape.clone()),
                        });
                        shapes.len() as u32 - 1
                    }),
                    None => *inline
                        .entry(Arc::as_ptr(&co.shape.0) as *const ())
                        .or_insert_with(|| {
                            shapes.push(DeltaShape::Inline(co.shape.clone()));
                            shapes.len() as u32 - 1
                        }),
                };
                InternedCollider::from_collider(co, id)
            });
            ColliderSetDelta {
                shapes,
                colliders: delta,
                modified_colliders: self.modified_colliders.clone(),
                removed_colliders: self.removed_colliders.clone(),
            }
        }

        fn apply_delta_payload(&mut self, delta: ColliderSetDelta) -> Result<(), DeltaError> {
            // Resolve the table first: keyed shapes without bytes come from the shapes this
            // set held before the patch.
            let received = self.delta_state.received(&self.colliders);
            let mut table = Vec::with_capacity(delta.shapes.len());
            for shape in delta.shapes {
                table.push(match shape {
                    DeltaShape::Inline(shape) => shape,
                    DeltaShape::Keyed {
                        key,
                        shape: Some(shape),
                    } => received.entry(key).or_insert(shape).clone(),
                    DeltaShape::Keyed { key, shape: None } => {
                        received.get(&key).ok_or(DeltaError::MissingShape)?.clone()
                    }
                });
            }

            self.colliders.apply_delta_with(delta.colliders, |co| {
                let shape = table
                    .get(co.shape_id() as usize)
                    .ok_or(DeltaError::Malformed)?;
                Ok(co.into_collider(shape.clone()))
            })?;
            self.modified_colliders = delta.modified_colliders;
            self.removed_colliders = delta.removed_colliders;

            // Forget the received shapes no collider holds anymore: the sender never omits the
            // bytes of content the receiver's colliders didn't hold at its baseline.
            if let Some(received) = &mut self.delta_state.received {
                received.retain(|_, shape| Arc::strong_count(&shape.0) > 1);
            }
            Ok(())
        }
    }
}
