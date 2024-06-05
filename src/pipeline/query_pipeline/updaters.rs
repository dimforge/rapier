//! This module contains structs implementing [`QbvhDataGenerator<ColliderHandle>`].
//!
//! These structs are designed to be passed as a parameters to
//! [`super::QueryPipeline::update_with_mode`] to update the query pipeline.

use parry::partitioning::QbvhDataGenerator;

use crate::math::Real;
use crate::prelude::{Aabb, ColliderHandle, ColliderSet, RigidBodySet};

/// Designed to be passed as a parameters to
/// [`super::QueryPipeline::update_with_mode`] to update the query pipeline.
///
/// The `RigidBody::predict_position_using_velocity_and_forces * Collider::position_wrt_parent`
/// is taken into account for the colliders position.
pub struct SweepTestWithPredictedPosition<'a> {
    /// The rigid bodies of your simulation.
    pub bodies: &'a RigidBodySet,
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
    /// The delta time to compute predicted position.
    /// You probably want to set it to [`crate::dynamics::IntegrationParameter::dt`].
    pub dt: Real,
}
impl<'a> QbvhDataGenerator<ColliderHandle> for SweepTestWithPredictedPosition<'a> {
    fn size_hint(&self) -> usize {
        self.colliders.len()
    }

    #[inline(always)]
    fn for_each(&mut self, mut f: impl FnMut(ColliderHandle, Aabb)) {
        for (h, co) in self.colliders.iter_enabled() {
            if let Some(co_parent) = co.parent {
                let rb = &self.bodies[co_parent.handle];
                let predicted_pos = rb
                    .pos
                    .integrate_forces_and_velocities(self.dt, &rb.forces, &rb.vels, &rb.mprops);

                let next_position = predicted_pos * co_parent.pos_wrt_parent;
                f(h, co.shape.compute_swept_aabb(&co.pos, &next_position))
            } else {
                f(h, co.shape.compute_aabb(&co.pos))
            }
        }
    }
}

/// Designed to be passed as a parameters to update the query pipeline,
/// through [`super::QueryPipeline::update_with_mode`].
///
/// The `RigidBody::next_position * Collider::position_wrt_parent` is taken into account for
/// the colliders positions.
pub struct SweepTestWithNextPosition<'a> {
    /// The rigid bodies of your simulation.
    pub bodies: &'a RigidBodySet,
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
}

impl<'a> QbvhDataGenerator<ColliderHandle> for SweepTestWithNextPosition<'a> {
    fn size_hint(&self) -> usize {
        self.colliders.len()
    }

    #[inline(always)]
    fn for_each(&mut self, mut f: impl FnMut(ColliderHandle, Aabb)) {
        for (h, co) in self.colliders.iter_enabled() {
            if let Some(co_parent) = co.parent {
                let rb_next_pos = &self.bodies[co_parent.handle].pos.next_position;
                let next_position = rb_next_pos * co_parent.pos_wrt_parent;
                f(h, co.shape.compute_swept_aabb(&co.pos, &next_position))
            } else {
                f(h, co.shape.compute_aabb(&co.pos))
            }
        }
    }
}

/// Designed to be passed as a parameters to update the query pipeline,
/// through [`super::QueryPipeline::update_with_mode`].
///
/// The [`super::Collider::position`] is taken into account.
pub struct CurrentPosition<'a> {
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
}

impl<'a> QbvhDataGenerator<ColliderHandle> for CurrentPosition<'a> {
    fn size_hint(&self) -> usize {
        self.colliders.len()
    }

    #[inline(always)]
    fn for_each(&mut self, mut f: impl FnMut(ColliderHandle, Aabb)) {
        for (h, co) in self.colliders.iter_enabled() {
            f(h, co.shape.compute_aabb(&co.pos))
        }
    }
}
