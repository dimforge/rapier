use crate::data::{BundleSet, ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{
    JointSet, RigidBodyActivation, RigidBodyColliders, RigidBodyHandle, RigidBodyIds,
    RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{ColliderParent, NarrowPhase};
use crate::math::Real;

/// Structure responsible for maintaining the set of active rigid-bodies, and
/// putting non-moving rigid-bodies to sleep to save computation times.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct IslandManager {
    pub(crate) active_dynamic_set: Vec<RigidBodyHandle>,
    pub(crate) active_kinematic_set: Vec<RigidBodyHandle>,
    pub(crate) active_islands: Vec<usize>,
    active_set_timestamp: u32,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    can_sleep: Vec<RigidBodyHandle>, // Workspace.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    stack: Vec<RigidBodyHandle>, // Workspace.
}

impl IslandManager {
    /// Creates a new empty island manager.
    pub fn new() -> Self {
        Self {
            active_dynamic_set: vec![],
            active_kinematic_set: vec![],
            active_islands: vec![],
            active_set_timestamp: 0,
            can_sleep: vec![],
            stack: vec![],
        }
    }

    pub(crate) fn num_islands(&self) -> usize {
        self.active_islands.len() - 1
    }

    /// Update this data-structure after one or multiple rigid-bodies have been removed for `bodies`.
    pub fn cleanup_removed_rigid_bodies(
        &mut self,
        bodies: &mut impl ComponentSetMut<RigidBodyIds>,
    ) {
        let mut active_sets = [&mut self.active_kinematic_set, &mut self.active_dynamic_set];

        for active_set in &mut active_sets {
            let mut i = 0;

            while i < active_set.len() {
                let handle = active_set[i];
                if bodies.get(handle.0).is_none() {
                    // This rigid-body no longer exists, so we need to remove it from the active set.
                    active_set.swap_remove(i);

                    if i < active_set.len() {
                        bodies.map_mut_internal(active_set[i].0, |rb_ids| rb_ids.active_set_id = i);
                    }
                } else {
                    i += 1;
                }
            }
        }
    }

    pub(crate) fn rigid_body_removed(
        &mut self,
        removed_handle: RigidBodyHandle,
        removed_ids: &RigidBodyIds,
        bodies: &mut impl ComponentSetMut<RigidBodyIds>,
    ) {
        let mut active_sets = [&mut self.active_kinematic_set, &mut self.active_dynamic_set];

        for active_set in &mut active_sets {
            if active_set.get(removed_ids.active_set_id) == Some(&removed_handle) {
                active_set.swap_remove(removed_ids.active_set_id);

                if let Some(replacement) = active_set.get(removed_ids.active_set_id) {
                    bodies.map_mut_internal(replacement.0, |ids| {
                        ids.active_set_id = removed_ids.active_set_id;
                    });
                }
            }
        }
    }

    /// Forces the specified rigid-body to wake up if it is dynamic.
    ///
    /// If `strong` is `true` then it is assured that the rigid-body will
    /// remain awake during multiple subsequent timesteps.
    pub fn wake_up<Bodies>(&mut self, bodies: &mut Bodies, handle: RigidBodyHandle, strong: bool)
    where
        Bodies: ComponentSetMut<RigidBodyActivation>
            + ComponentSetOption<RigidBodyType>
            + ComponentSetMut<RigidBodyIds>,
    {
        // NOTE: the use an Option here because there are many legitimate cases (like when
        //       deleting a joint attached to an already-removed body) where we could be
        //       attempting to wake-up a rigid-body that has already been deleted.
        let rb_type: Option<RigidBodyType> = bodies.get(handle.0).copied();
        if rb_type == Some(RigidBodyType::Dynamic) {
            bodies.map_mut_internal(handle.0, |activation: &mut RigidBodyActivation| {
                activation.wake_up(strong)
            });
            bodies.map_mut_internal(handle.0, |ids: &mut RigidBodyIds| {
                if self.active_dynamic_set.get(ids.active_set_id) != Some(&handle) {
                    ids.active_set_id = self.active_dynamic_set.len();
                    self.active_dynamic_set.push(handle);
                }
            });
        }
    }

    /// Iter through all the active kinematic rigid-bodies on this set.
    pub fn active_kinematic_bodies(&self) -> &[RigidBodyHandle] {
        &self.active_kinematic_set[..]
    }

    /// Iter through all the active dynamic rigid-bodies on this set.
    pub fn active_dynamic_bodies(&self) -> &[RigidBodyHandle] {
        &self.active_dynamic_set[..]
    }

    pub(crate) fn active_island(&self, island_id: usize) -> &[RigidBodyHandle] {
        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        &self.active_dynamic_set[island_range]
    }

    #[inline(always)]
    pub(crate) fn iter_active_bodies<'a>(&'a self) -> impl Iterator<Item = RigidBodyHandle> + 'a {
        self.active_dynamic_set
            .iter()
            .copied()
            .chain(self.active_kinematic_set.iter().copied())
    }

    /*
    #[cfg(feature = "parallel")]
    #[inline(always)]
    #[allow(dead_code)]
    pub(crate) fn foreach_active_island_body_mut_internal_parallel<Set>(
        &self,
        island_id: usize,
        bodies: &mut Set,
        f: impl Fn(RigidBodyHandle, &mut RigidBody) + Send + Sync,
    ) where
        Set: ComponentSet<T>,
    {
        use std::sync::atomic::Ordering;

        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        let bodies = std::sync::atomic::AtomicPtr::new(&mut bodies as *mut _);
        self.active_dynamic_set[island_range]
            .par_iter()
            .for_each_init(
                || bodies.load(Ordering::Relaxed),
                |bodies, handle| {
                    let bodies: &mut Set = unsafe { std::mem::transmute(*bodies) };
                    if let Some(rb) = bodies.get_mut_internal(handle.0) {
                        f(*handle, rb)
                    }
                },
            );
    }
     */

    #[cfg(feature = "parallel")]
    pub(crate) fn active_island_range(&self, island_id: usize) -> std::ops::Range<usize> {
        self.active_islands[island_id]..self.active_islands[island_id + 1]
    }

    pub(crate) fn update_active_set_with_contacts<Bodies, Colliders>(
        &mut self,
        bodies: &mut Bodies,
        colliders: &Colliders,
        narrow_phase: &NarrowPhase,
        joints: &JointSet,
        min_island_size: usize,
    ) where
        Bodies: ComponentSetMut<RigidBodyIds>
            + ComponentSetMut<RigidBodyActivation>
            + ComponentSetMut<RigidBodyVelocity>
            + ComponentSet<RigidBodyColliders>
            + ComponentSet<RigidBodyType>,
        Colliders: ComponentSetOption<ColliderParent>,
    {
        assert!(
            min_island_size > 0,
            "The minimum island size must be at least 1."
        );

        // Update the energy of every rigid body and
        // keep only those that may not sleep.
        //        let t = instant::now();
        self.active_set_timestamp += 1;
        self.stack.clear();
        self.can_sleep.clear();

        // NOTE: the `.rev()` is here so that two successive timesteps preserve
        // the order of the bodies in the `active_dynamic_set` vec. This reversal
        // does not seem to affect performances nor stability. However it makes
        // debugging slightly nicer so we keep this rev.
        for h in self.active_dynamic_set.drain(..).rev() {
            let can_sleep = &mut self.can_sleep;
            let stack = &mut self.stack;

            let vels: &RigidBodyVelocity = bodies.index(h.0);
            let pseudo_kinetic_energy = vels.pseudo_kinetic_energy();

            bodies.map_mut_internal(h.0, |activation: &mut RigidBodyActivation| {
                update_energy(activation, pseudo_kinetic_energy);

                if activation.energy <= activation.threshold {
                    // Mark them as sleeping for now. This will
                    // be set to false during the graph traversal
                    // if it should not be put to sleep.
                    activation.sleeping = true;
                    can_sleep.push(h);
                } else {
                    stack.push(h);
                }
            });
        }

        // Read all the contacts and push objects touching touching this rigid-body.
        #[inline(always)]
        fn push_contacting_bodies(
            rb_colliders: &RigidBodyColliders,
            colliders: &impl ComponentSetOption<ColliderParent>,
            narrow_phase: &NarrowPhase,
            stack: &mut Vec<RigidBodyHandle>,
        ) {
            for collider_handle in &rb_colliders.0 {
                for inter in narrow_phase.contacts_with(*collider_handle) {
                    for manifold in &inter.manifolds {
                        if !manifold.data.solver_contacts.is_empty() {
                            let other = crate::utils::select_other(
                                (inter.collider1, inter.collider2),
                                *collider_handle,
                            );
                            if let Some(other_body) = colliders.get(other.0) {
                                stack.push(other_body.handle);
                            }
                            break;
                        }
                    }
                }
            }
        }

        // Now iterate on all active kinematic bodies and push all the bodies
        // touching them to the stack so they can be woken up.
        for h in self.active_kinematic_set.iter() {
            let (vels, rb_colliders): (&RigidBodyVelocity, _) = bodies.index_bundle(h.0);

            if vels.is_zero() {
                // If the kinematic body does not move, it does not have
                // to wake up any dynamic body.
                continue;
            }

            push_contacting_bodies(rb_colliders, colliders, narrow_phase, &mut self.stack);
        }

        //        println!("Selection: {}", instant::now() - t);

        //        let t = instant::now();
        // Propagation of awake state and awake island computation through the
        // traversal of the interaction graph.
        self.active_islands.clear();
        self.active_islands.push(0);

        // The max avoid underflow when the stack is empty.
        let mut island_marker = self.stack.len().max(1) - 1;

        while let Some(handle) = self.stack.pop() {
            let (rb_status, rb_ids, rb_colliders): (
                &RigidBodyType,
                &RigidBodyIds,
                &RigidBodyColliders,
            ) = bodies.index_bundle(handle.0);

            if rb_ids.active_set_timestamp == self.active_set_timestamp || !rb_status.is_dynamic() {
                // We already visited this body and its neighbors.
                // Also, we don't propagate awake state through static bodies.
                continue;
            }

            if self.stack.len() < island_marker {
                if self.active_dynamic_set.len() - *self.active_islands.last().unwrap()
                    >= min_island_size
                {
                    // We are starting a new island.
                    self.active_islands.push(self.active_dynamic_set.len());
                }

                island_marker = self.stack.len();
            }

            // Transmit the active state to all the rigid-bodies with colliders
            // in contact or joined with this collider.
            push_contacting_bodies(rb_colliders, colliders, narrow_phase, &mut self.stack);

            for inter in joints.joints_with(handle) {
                let other = crate::utils::select_other((inter.0, inter.1), handle);
                self.stack.push(other);
            }

            bodies.map_mut_internal(handle.0, |activation: &mut RigidBodyActivation| {
                activation.wake_up(false);
            });
            bodies.map_mut_internal(handle.0, |ids: &mut RigidBodyIds| {
                ids.active_island_id = self.active_islands.len() - 1;
                ids.active_set_id = self.active_dynamic_set.len();
                ids.active_set_offset =
                    ids.active_set_id - self.active_islands[ids.active_island_id];
                ids.active_set_timestamp = self.active_set_timestamp;
            });

            self.active_dynamic_set.push(handle);
        }

        self.active_islands.push(self.active_dynamic_set.len());
        //        println!(
        //            "Extraction: {}, num islands: {}",
        //            instant::now() - t,
        //            self.active_islands.len() - 1
        //        );

        // Actually put to sleep bodies which have not been detected as awake.
        for h in &self.can_sleep {
            let activation: &RigidBodyActivation = bodies.index(h.0);
            if activation.sleeping {
                bodies.set_internal(h.0, RigidBodyVelocity::zero());
                bodies.map_mut_internal(h.0, |activation: &mut RigidBodyActivation| {
                    activation.sleep()
                });
            }
        }
    }
}

fn update_energy(activation: &mut RigidBodyActivation, pseudo_kinetic_energy: Real) {
    let mix_factor = 0.01;
    let new_energy = (1.0 - mix_factor) * activation.energy + mix_factor * pseudo_kinetic_energy;
    activation.energy = new_energy.min(activation.threshold.abs() * 4.0);
}
