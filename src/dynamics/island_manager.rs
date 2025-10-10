use crate::dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyActivation, RigidBodyChanges, RigidBodyColliders,
    RigidBodyHandle, RigidBodyIds, RigidBodySet, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{ColliderSet, NarrowPhase};
use crate::math::Real;
use crate::utils::SimdDot;

/// System that manages which bodies are active (awake) vs sleeping to optimize performance.
///
/// ## Sleeping Optimization
///
/// Bodies at rest automatically "sleep" - they're excluded from simulation until something
/// disturbs them (collision, joint connection to moving body, manual wake-up). This can
/// dramatically improve performance in scenes with many static/resting objects.
///
/// ## Islands
///
/// Connected bodies (via contacts or joints) are grouped into "islands" that are solved together.
/// This allows parallel solving and better organization.
///
/// You rarely interact with this directly - it's automatically managed by [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline).
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
pub struct IslandManager {
    pub(crate) active_set: Vec<RigidBodyHandle>,
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
            active_set: vec![],
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
        let mut i = 0;

        while i < self.active_set.len() {
            let handle = self.active_set[i];
            if bodies.get(handle).is_none() {
                // This rigid-body no longer exists, so we need to remove it from the active set.
                self.active_set.swap_remove(i);

                if i < self.active_set.len() {
                    // Update the self.active_set_id for the body that has been swapped.
                    if let Some(swapped_rb) = bodies.get_mut_internal(self.active_set[i]) {
                        swapped_rb.ids.active_set_id = i;
                    }
                }
            } else {
                i += 1;
            }
        }
    }

    pub(crate) fn rigid_body_removed(
        &mut self,
        removed_handle: RigidBodyHandle,
        removed_ids: &RigidBodyIds,
        bodies: &mut RigidBodySet,
    ) {
        if self.active_set.get(removed_ids.active_set_id) == Some(&removed_handle) {
            self.active_set.swap_remove(removed_ids.active_set_id);

            if let Some(replacement) = self
                .active_set
                .get(removed_ids.active_set_id)
                .and_then(|h| bodies.get_mut_internal(*h))
            {
                replacement.ids.active_set_id = removed_ids.active_set_id;
            }
        }
    }

    /// Wakes up a sleeping body, forcing it back into the active simulation.
    ///
    /// Use this when you want to ensure a body is active (useful after manually moving
    /// a sleeping body, or to prevent it from sleeping in the next few frames).
    ///
    /// # Parameters
    /// * `strong` - If `true`, the body is guaranteed to stay awake for multiple frames.
    ///   If `false`, it might sleep again immediately if conditions are met.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut islands = IslandManager::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// islands.wake_up(&mut bodies, body_handle, true);
    /// let body = bodies.get_mut(body_handle).unwrap();
    /// // Wake up a body before applying force to it
    /// body.add_force(vector![100.0, 0.0, 0.0], false);
    /// ```
    ///
    /// Only affects dynamic bodies (kinematic and fixed bodies don't sleep).
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

                if rb.is_enabled() && self.active_set.get(rb.ids.active_set_id) != Some(&handle) {
                    rb.ids.active_set_id = self.active_set.len();
                    self.active_set.push(handle);
                }
            }
        }
    }

    pub(crate) fn active_island(&self, island_id: usize) -> &[RigidBodyHandle] {
        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        &self.active_set[island_range]
    }

    pub(crate) fn active_island_additional_solver_iterations(&self, island_id: usize) -> usize {
        self.active_islands_additional_solver_iterations[island_id]
    }

    /// Handls of dynamic and kinematic rigid-bodies that are currently active (i.e. not sleeping).
    #[inline]
    pub fn active_bodies(&self) -> &[RigidBodyHandle] {
        &self.active_set
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
        // the order of the bodies in the `active_set` vec. This reversal
        // does not seem to affect performances nor stability. However it makes
        // debugging slightly nicer.
        for h in self.active_set.drain(..).rev() {
            let can_sleep = &mut self.can_sleep;
            let stack = &mut self.stack;

            let rb = bodies.index_mut_internal(h);
            let sq_linvel = rb.vels.linvel.norm_squared();
            let sq_angvel = rb.vels.angvel.gdot(rb.vels.angvel);

            update_energy(
                &mut rb.activation,
                rb.body_type,
                length_unit,
                sq_linvel,
                sq_angvel,
                dt,
            );

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
        #[inline]
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

            if rb.ids.active_set_timestamp == self.active_set_timestamp
                || !rb.is_dynamic_or_kinematic()
            {
                // We already visited this body and its neighbors.
                // Also, we don't propagate awake state through fixed bodies.
                continue;
            }

            if self.stack.len() < island_marker {
                if additional_solver_iterations != rb.additional_solver_iterations
                    || self.active_set.len() - *self.active_islands.last().unwrap()
                        >= min_island_size
                {
                    // We are starting a new island.
                    self.active_islands_additional_solver_iterations
                        .push(additional_solver_iterations);
                    self.active_islands.push(self.active_set.len());
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
            rb.ids.active_set_id = self.active_set.len();
            rb.ids.active_set_offset =
                (rb.ids.active_set_id - self.active_islands[rb.ids.active_island_id]) as u32;
            rb.ids.active_set_timestamp = self.active_set_timestamp;

            self.active_set.push(handle);
        }

        self.active_islands_additional_solver_iterations
            .push(additional_solver_iterations);
        self.active_islands.push(self.active_set.len());
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
    activation: &mut RigidBodyActivation,
    body_type: RigidBodyType,
    length_unit: Real,
    sq_linvel: Real,
    sq_angvel: Real,
    dt: Real,
) {
    let can_sleep = match body_type {
        RigidBodyType::Dynamic => {
            let linear_threshold = activation.normalized_linear_threshold * length_unit;
            sq_linvel < linear_threshold * linear_threshold.abs()
                && sq_angvel < activation.angular_threshold * activation.angular_threshold.abs()
        }
        RigidBodyType::KinematicPositionBased | RigidBodyType::KinematicVelocityBased => {
            // Platforms only sleep if both velocities are exactly zero. If it’s not exactly
            // zero, then the user really wants them to move.
            sq_linvel == 0.0 && sq_angvel == 0.0
        }
        RigidBodyType::Fixed => true,
    };

    if can_sleep {
        activation.time_since_can_sleep += dt;
    } else {
        activation.time_since_can_sleep = 0.0;
    }
}
