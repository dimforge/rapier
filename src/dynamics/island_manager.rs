use crate::dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyActivation, RigidBodyChanges, RigidBodyColliders,
    RigidBodyHandle, RigidBodyIds, RigidBodySet, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::math::Real;
use crate::utils::SimdDot;

/// Structure responsible for maintaining the set of active rigid-bodies, and
/// putting non-moving rigid-bodies to sleep to save computation times.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
pub struct IslandManager {
    pub(crate) active_dynamic_set: Vec<RigidBodyHandle>,
    pub(crate) active_kinematic_set: Vec<RigidBodyHandle>,
    pub(crate) active_islands: Vec<usize>,
    pub(crate) active_islands_additional_solver_iterations: Vec<usize>,
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
            active_islands_additional_solver_iterations: vec![],
            active_set_timestamp: 0,
            can_sleep: vec![],
            stack: vec![],
        }
    }

    pub(crate) fn num_islands(&self) -> usize {
        self.active_islands.len().saturating_sub(1)
    }

    /// Update this data-structure after one or multiple rigid-bodies have been removed for `bodies`.
    pub fn cleanup_removed_rigid_bodies(&mut self, bodies: &mut RigidBodySet) {
        let mut active_sets = [&mut self.active_kinematic_set, &mut self.active_dynamic_set];

        for active_set in &mut active_sets {
            let mut i = 0;

            while i < active_set.len() {
                let handle = active_set[i];
                if bodies.get(handle).is_none() {
                    // This rigid-body no longer exists, so we need to remove it from the active set.
                    active_set.swap_remove(i);

                    if i < active_set.len() {
                        // Update the active_set_id for the body that has been swapped.
                        if let Some(swapped_rb) = bodies.get_mut_internal(active_set[i]) {
                            swapped_rb.ids.active_set_id = i;
                        }
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
        bodies: &mut RigidBodySet,
    ) {
        let mut active_sets = [&mut self.active_kinematic_set, &mut self.active_dynamic_set];

        for active_set in &mut active_sets {
            if active_set.get(removed_ids.active_set_id) == Some(&removed_handle) {
                active_set.swap_remove(removed_ids.active_set_id);

                if let Some(replacement) = active_set
                    .get(removed_ids.active_set_id)
                    .and_then(|h| bodies.get_mut_internal(*h))
                {
                    replacement.ids.active_set_id = removed_ids.active_set_id;
                }
            }
        }
    }

    /// Forces the specified rigid-body to wake up if it is dynamic.
    ///
    /// If `strong` is `true` then it is assured that the rigid-body will
    /// remain awake during multiple subsequent timesteps.
    pub fn wake_up(&mut self, bodies: &mut RigidBodySet, handle: RigidBodyHandle, strong: bool) {
        // NOTE: the use an Option here because there are many legitimate cases (like when
        //       deleting a joint attached to an already-removed body) where we could be
        //       attempting to wake-up a rigid-body that has already been deleted.
        if bodies.get(handle).map(|rb| rb.body_type()) == Some(RigidBodyType::Dynamic) {
            let rb = bodies.index_mut_internal(handle);

            // Check that the user didn’t change the sleeping state explicitly, in which
            // case we don’t overwrite it.
            if !rb.changes.contains(RigidBodyChanges::SLEEP) {
                rb.activation.wake_up(strong);

                if rb.is_enabled()
                    && self.active_dynamic_set.get(rb.ids.active_set_id) != Some(&handle)
                {
                    rb.ids.active_set_id = self.active_dynamic_set.len();
                    self.active_dynamic_set.push(handle);
                }
            }
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

    pub(crate) fn active_island_additional_solver_iterations(&self, island_id: usize) -> usize {
        self.active_islands_additional_solver_iterations[island_id]
    }

    #[inline(always)]
    pub(crate) fn iter_active_bodies(&self) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        self.active_dynamic_set
            .iter()
            .copied()
            .chain(self.active_kinematic_set.iter().copied())
    }

    #[cfg(feature = "parallel")]
    #[allow(dead_code)] // That will likely be useful when we re-introduce intra-island parallelism.
    pub(crate) fn active_island_range(&self, island_id: usize) -> std::ops::Range<usize> {
        self.active_islands[island_id]..self.active_islands[island_id + 1]
    }

    pub(crate) fn update_active_set_with_contacts(
        &mut self,
        dt: Real,
        length_unit: Real,
        bodies: &mut RigidBodySet,
        colliders: &ColliderSet,
        narrow_phase: &NarrowPhase,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        min_island_size: usize,
    ) {
        assert!(
            min_island_size > 0,
            "The minimum island size must be at least 1."
        );

        // Update the energy of every rigid body and
        // keep only those that may not sleep.
        //        let t = Instant::now();
        self.active_set_timestamp += 1;
        self.stack.clear();
        self.can_sleep.clear();

        // NOTE: the `.rev()` is here so that two successive timesteps preserve
        // the order of the bodies in the `active_dynamic_set` vec. This reversal
        // does not seem to affect performances nor stability. However it makes
        // debugging slightly nicer.
        for h in self.active_dynamic_set.drain(..).rev() {
            let can_sleep = &mut self.can_sleep;
            let stack = &mut self.stack;

            let rb = bodies.index_mut_internal(h);
            let sq_linvel = rb.vels.linvel.norm_squared();
            let sq_angvel = rb.vels.angvel.gdot(rb.vels.angvel);

            update_energy(length_unit, &mut rb.activation, sq_linvel, sq_angvel, dt);

            if rb.activation.time_since_can_sleep >= rb.activation.time_until_sleep {
                // Mark them as sleeping for now. This will
                // be set to false during the graph traversal
                // if it should not be put to sleep.
                rb.activation.sleeping = true;
                can_sleep.push(h);
            } else {
                stack.push(h);
            }
        }

        // Read all the contacts and push objects touching touching this rigid-body.
        #[inline(always)]
        fn push_contacting_bodies(
            rb_colliders: &RigidBodyColliders,
            colliders: &ColliderSet,
            narrow_phase: &NarrowPhase,
            stack: &mut Vec<RigidBodyHandle>,
        ) {
            for collider_handle in &rb_colliders.0 {
                for inter in narrow_phase.contact_pairs_with(*collider_handle) {
                    for manifold in &inter.manifolds {
                        if !manifold.data.solver_contacts.is_empty() {
                            let other = crate::utils::select_other(
                                (inter.collider1, inter.collider2),
                                *collider_handle,
                            );
                            if let Some(other_body) = colliders[other].parent {
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
            let rb = &bodies[*h];

            if rb.vels.is_zero() {
                // If the kinematic body does not move, it does not have
                // to wake up any dynamic body.
                continue;
            }

            push_contacting_bodies(&rb.colliders, colliders, narrow_phase, &mut self.stack);
        }

        //        println!("Selection: {}", Instant::now() - t);

        //        let t = Instant::now();
        // Propagation of awake state and awake island computation through the
        // traversal of the interaction graph.
        self.active_islands_additional_solver_iterations.clear();
        self.active_islands.clear();
        self.active_islands.push(0);

        // saturating_sub(1) prevents underflow when the stack is empty.
        let mut island_marker = self.stack.len().saturating_sub(1);

        // NOTE: islands containing a body with non-standard number of iterations won’t
        //       be merged with another island, unless another island with standard
        //       iterations number already started before and got continued due to the
        //       `min_island_size`. That could be avoided by pushing bodies with non-standard
        //       iterations on top of the stack (and other bodies on the back). Not sure it’s
        //       worth it though.
        let mut additional_solver_iterations = 0;

        while let Some(handle) = self.stack.pop() {
            let rb = bodies.index_mut_internal(handle);

            if rb.ids.active_set_timestamp == self.active_set_timestamp || !rb.is_dynamic() {
                // We already visited this body and its neighbors.
                // Also, we don't propagate awake state through fixed bodies.
                continue;
            }

            if self.stack.len() < island_marker {
                if additional_solver_iterations != rb.additional_solver_iterations
                    || self.active_dynamic_set.len() - *self.active_islands.last().unwrap()
                        >= min_island_size
                {
                    // We are starting a new island.
                    self.active_islands_additional_solver_iterations
                        .push(additional_solver_iterations);
                    self.active_islands.push(self.active_dynamic_set.len());
                    additional_solver_iterations = 0;
                }

                island_marker = self.stack.len();
            }

            additional_solver_iterations =
                additional_solver_iterations.max(rb.additional_solver_iterations);

            // Transmit the active state to all the rigid-bodies with colliders
            // in contact or joined with this collider.
            push_contacting_bodies(&rb.colliders, colliders, narrow_phase, &mut self.stack);

            for inter in impulse_joints.attached_enabled_joints(handle) {
                let other = crate::utils::select_other((inter.0, inter.1), handle);
                self.stack.push(other);
            }

            for other in multibody_joints.bodies_attached_with_enabled_joint(handle) {
                self.stack.push(other);
            }

            rb.activation.wake_up(false);
            rb.ids.active_island_id = self.active_islands.len() - 1;
            rb.ids.active_set_id = self.active_dynamic_set.len();
            rb.ids.active_set_offset =
                rb.ids.active_set_id - self.active_islands[rb.ids.active_island_id];
            rb.ids.active_set_timestamp = self.active_set_timestamp;

            self.active_dynamic_set.push(handle);
        }

        self.active_islands_additional_solver_iterations
            .push(additional_solver_iterations);
        self.active_islands.push(self.active_dynamic_set.len());
        //        println!(
        //            "Extraction: {}, num islands: {}",
        //            Instant::now() - t,
        //            self.active_islands.len() - 1
        //        );

        // Actually put to sleep bodies which have not been detected as awake.
        for handle in &self.can_sleep {
            let rb = bodies.index_mut_internal(*handle);
            if rb.activation.sleeping {
                rb.vels = RigidBodyVelocity::zero();
                rb.activation.sleep();
            }
        }
    }
}

fn update_energy(
    length_unit: Real,
    activation: &mut RigidBodyActivation,
    sq_linvel: Real,
    sq_angvel: Real,
    dt: Real,
) {
    let linear_threshold = activation.normalized_linear_threshold * length_unit;
    if sq_linvel < linear_threshold * linear_threshold.abs()
        && sq_angvel < activation.angular_threshold * activation.angular_threshold.abs()
    {
        activation.time_since_can_sleep += dt;
    } else {
        activation.time_since_can_sleep = 0.0;
    }
}
