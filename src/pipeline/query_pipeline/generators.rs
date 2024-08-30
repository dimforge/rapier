//! Structs implementing [`QbvhDataGenerator<ColliderHandle>`] to be used with [`QueryPipeline::update_with_generator`].

use parry::partitioning::QbvhDataGenerator;

use crate::math::Real;
use crate::prelude::{Aabb, ColliderHandle, ColliderSet, RigidBodySet};

#[cfg(doc)]
use crate::{
    dynamics::{IntegrationParameters, RigidBody, RigidBodyPosition},
    pipeline::QueryPipeline,
    prelude::Collider,
};

/// Generates collider AABBs based on the union of their current AABB and the AABB predicted
/// from the velocity and forces of their parent rigid-body.
///
/// The main purpose of this struct is to be passed as a parameter to
/// [`QueryPipeline::update_with_generator`] to update the [`QueryPipeline`].
///
/// The predicted position is calculated as
/// `RigidBody::predict_position_using_velocity_and_forces * Collider::position_wrt_parent`.
pub struct SweptAabbWithPredictedPosition<'a> {
    /// The rigid bodies of your simulation.
    pub bodies: &'a RigidBodySet,
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
    /// The delta time to compute predicted position.
    ///
    /// You probably want to set it to [`IntegrationParameters::dt`].
    pub dt: Real,
}
impl<'a> QbvhDataGenerator<ColliderHandle> for SweptAabbWithPredictedPosition<'a> {
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

/// Generates collider AABBs based on the union of their AABB at their current [`Collider::position`]
/// and the AABB predicted from their parent’s [`RigidBody::next_position`].
///
/// The main purpose of this struct is to be passed as a parameter to
/// [`QueryPipeline::update_with_generator`] to update the [`QueryPipeline`].
///
/// The predicted position is calculated as
/// `RigidBody::next_position * Collider::position_wrt_parent`.
pub struct SweptAabbWithNextPosition<'a> {
    /// The rigid bodies of your simulation.
    pub bodies: &'a RigidBodySet,
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
}

impl<'a> QbvhDataGenerator<ColliderHandle> for SweptAabbWithNextPosition<'a> {
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

/// Generates collider AABBs based on the AABB at their current [`Collider::position`].
///
/// The main purpose of this struct is to be passed as a parameter to
/// [`QueryPipeline::update_with_generator`] to update the [`QueryPipeline`].
pub struct CurrentAabb<'a> {
    /// The colliders of your simulation.
    pub colliders: &'a ColliderSet,
}

impl<'a> QbvhDataGenerator<ColliderHandle> for CurrentAabb<'a> {
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
