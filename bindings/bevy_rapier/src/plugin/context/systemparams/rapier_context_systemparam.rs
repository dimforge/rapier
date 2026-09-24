use crate::math::{Rot, Vect};
use bevy::ecs::{query, system::SystemParam};
use bevy::prelude::*;
use rapier::prelude::Real;

pub(crate) const RAPIER_CONTEXT_EXPECT_ERROR: &str =
    "RapierContextEntityLink.0 refers to an entity missing components from RapierContextSimulation.";

use crate::plugin::context::{
    DefaultRapierContext, RapierContextColliders, RapierContextJoints, RapierContextSimulation,
    RapierQueryPipeline, RapierRigidBodySet,
};

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] immutably.
///
/// This uses the [`DefaultRapierContext`] filter by default, but you can use a custom query filter with the `T` type parameter.
#[derive(SystemParam)]
pub struct ReadRapierContext<'w, 's, T: query::QueryFilter + 'static = With<DefaultRapierContext>> {
    /// The query used to feed components into [`RapierContext`] struct through [`ReadRapierContext::single`].
    pub rapier_context: Query<
        'w,
        's,
        (
            &'static RapierContextSimulation,
            &'static RapierContextColliders,
            &'static RapierContextJoints,
            &'static RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> ReadRapierContext<'w, 's, T> {
    /// Returns a single [`RapierContext`] corresponding to the filter (T) of [`ReadRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`ReadRapierContext::rapier_context`] for finer grained queries.
    pub fn single(&self) -> Result<RapierContext<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single()?;
        Ok(RapierContext {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// Note: This is not a component, refer to [`ReadRapierContext`], [`WriteRapierContext`], or [`RapierContextSimulation`]
#[cfg_attr(feature = "serde-serialize", derive(Serialize))]
#[derive(query::QueryData)]
pub struct RapierContext<'a> {
    /// The Rapier context, containing all the state of the physics engine.
    pub simulation: &'a RapierContextSimulation,
    /// The set of colliders part of the simulation.
    pub colliders: &'a RapierContextColliders,
    /// The sets of joints part of the simulation.
    pub joints: &'a RapierContextJoints,
    /// The set of rigid-bodies part of the simulation.
    pub rigidbody_set: &'a RapierRigidBodySet,
}

/// Utility [`SystemParam`] to easily access every required components of a [`RapierContext`] mutably.
///
/// This uses the [`DefaultRapierContext`] filter by default, but you can use a custom query filter with the `T` type parameter.
#[derive(SystemParam)]
pub struct WriteRapierContext<'w, 's, T: query::QueryFilter + 'static = With<DefaultRapierContext>>
{
    /// The query used to feed components into [`RapierContext`] struct through [`ReadRapierContext::single`].
    pub rapier_context: Query<
        'w,
        's,
        (
            &'static mut RapierContextSimulation,
            &'static mut RapierContextColliders,
            &'static mut RapierContextJoints,
            &'static mut RapierRigidBodySet,
        ),
        T,
    >,
}

impl<'w, 's, T: query::QueryFilter + 'static> WriteRapierContext<'w, 's, T> {
    /// Returns a single [`RapierContext`] corresponding to the filter (T) of [`WriteRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`WriteRapierContext::rapier_context`] for finer grained queries.
    pub fn single(&self) -> Result<RapierContext<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single()?;
        Ok(RapierContext {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }

    /// Returns a single mutable [`RapierContextMut`] corresponding to the filter (T) of [`WriteRapierContext`].
    ///
    /// If the number of query items is not exactly one, a [`bevy::ecs::query::QuerySingleError`] is returned instead.
    ///
    /// You can also use the underlying query [`WriteRapierContext::rapier_context`] for finer grained queries.
    pub fn single_mut(&mut self) -> Result<RapierContextMut<'_>> {
        let (simulation, colliders, joints, rigidbody_set) = self.rapier_context.single_mut()?;
        Ok(RapierContextMut {
            simulation,
            colliders,
            joints,
            rigidbody_set,
        })
    }
}

/// A helper struct to avoid passing too many parameters to most rapier functions.
/// This helps with reducing boilerplate, at the (small) price of maybe getting too much information from the ECS.
///
/// If you need more granular control over mutability of each component, use a regular [`Query`]
pub struct RapierContextMut<'a> {
    /// The Rapier context, containing all the state of the physics engine.
    pub simulation: Mut<'a, RapierContextSimulation>,
    /// The set of colliders part of the simulation.
    pub colliders: Mut<'a, RapierContextColliders>,
    /// The sets of joints part of the simulation.
    pub joints: Mut<'a, RapierContextJoints>,
    /// The set of rigid-bodies part of the simulation.
    pub rigidbody_set: Mut<'a, RapierRigidBodySet>,
}

/// [`RapierRigidBodySet`] functions
mod simulation {
    use crate::control::CharacterCollision;
    use crate::control::MoveShapeOptions;
    use crate::control::MoveShapeOutput;
    use crate::geometry::AsShape;
    use crate::plugin::context::SimulationToRenderTime;
    use crate::plugin::ContactPairView;
    use crate::plugin::{SimulationMode, TimestepMode};
    use crate::prelude::CollisionEvent;
    use crate::prelude::ContactForceEvent;
    use crate::prelude::QueryFilter;
    use crate::prelude::RapierRigidBodyHandle;
    use crate::prelude::TransformInterpolation;
    use rapier::prelude::PhysicsHooks;

    use super::*;

    /// [`RapierContextSimulation`] functions for immutable accesses
    impl RapierContext<'_> {
        /// Shortcut to [`RapierContextSimulation::contact_pair`].
        pub fn contact_pair(
            &self,
            collider1: Entity,
            collider2: Entity,
        ) -> Option<ContactPairView<'_>> {
            self.simulation
                .contact_pair(self.colliders, self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView<'_>> {
            self.simulation
                .contact_pairs_with(self.colliders, self.rigidbody_set, collider)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pair`].
        pub fn intersection_pair(&self, collider1: Entity, collider2: Entity) -> Option<bool> {
            self.simulation
                .intersection_pair(self.colliders, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pairs_with`].
        pub fn intersection_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
            self.simulation
                .intersection_pairs_with(self.colliders, collider)
        }
    }

    /// [`RapierContextSimulation`] functions for mutable accesses
    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierContextSimulation::step_simulation`].
        #[expect(clippy::too_many_arguments)]
        pub fn step_simulation(
            &mut self,
            gravity: Vect,
            simulation_mode: SimulationMode,
            timestep_mode: TimestepMode,
            events: Option<(
                &MessageWriter<CollisionEvent>,
                &MessageWriter<ContactForceEvent>,
            )>,
            hooks: &dyn PhysicsHooks,
            time: &Time,
            sim_to_render_time: &mut SimulationToRenderTime,
            interpolation_query: Option<
                &mut Query<(&RapierRigidBodyHandle, &mut TransformInterpolation)>,
            >,
        ) {
            self.simulation.step_simulation(
                &mut self.colliders,
                &mut self.joints,
                &mut self.rigidbody_set,
                gravity,
                simulation_mode,
                timestep_mode,
                events,
                hooks,
                time,
                sim_to_render_time,
                interpolation_query,
            )
        }

        /// Shortcut to [`RapierContextSimulation::move_shape`].
        ///
        /// The scene queries are filtered with `filter`, like the other scene queries of this
        /// context.
        #[expect(clippy::too_many_arguments)]
        pub fn move_shape(
            &mut self,
            movement: Vect,
            shape: &(impl AsShape + ?Sized),
            shape_translation: Vect,
            shape_rotation: Rot,
            shape_mass: Real,
            options: &MoveShapeOptions,
            filter: QueryFilter,
            events: impl FnMut(CharacterCollision),
        ) -> MoveShapeOutput {
            self.simulation.move_shape(
                &mut self.colliders,
                &mut self.rigidbody_set,
                movement,
                shape,
                shape_translation,
                shape_rotation,
                shape_mass,
                options,
                filter,
                events,
            )
        }

        /// Shortcut to [`RapierContextSimulation::contact_pair`].
        pub fn contact_pair(
            &self,
            collider1: Entity,
            collider2: Entity,
        ) -> Option<ContactPairView<'_>> {
            self.simulation
                .contact_pair(&self.colliders, &self.rigidbody_set, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::contact_pairs_with`].
        pub fn contact_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = ContactPairView<'_>> {
            self.simulation
                .contact_pairs_with(&self.colliders, &self.rigidbody_set, collider)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pair`].
        pub fn intersection_pair(&self, collider1: Entity, collider2: Entity) -> Option<bool> {
            self.simulation
                .intersection_pair(&self.colliders, collider1, collider2)
        }

        /// Shortcut to [`RapierContextSimulation::intersection_pairs_with`].
        pub fn intersection_pairs_with(
            &self,
            collider: Entity,
        ) -> impl Iterator<Item = (Entity, Entity, bool)> + '_ {
            self.simulation
                .intersection_pairs_with(&self.colliders, collider)
        }
    }
}

mod query_pipeline {
    use rapier::{
        geometry::Collider as RapierCollider,
        parry::{partitioning::Bvh, query::ShapeCastOptions},
        prelude::FeatureId,
    };

    use crate::geometry::AsShape;

    use crate::prelude::{
        NonlinearMotion, PointProjection, QueryFilter, RayIntersection, ShapeCastHit,
        ShapeClosestPoints, ShapeContact,
    };

    use super::*;

    impl RapierContext<'_> {
        /// Shortcut to [RapierQueryPipeline::new_scoped].
        pub fn with_query_pipeline<'a, T>(
            &'a self,
            filter: QueryFilter<'a>,
            scoped_fn: impl FnOnce(RapierQueryPipeline<'_>) -> T,
        ) -> T {
            crate::prelude::RapierQueryPipeline::new_scoped(
                &self.simulation.broad_phase,
                self.colliders,
                self.rigidbody_set,
                &filter,
                self.simulation.query_dispatcher(),
                scoped_fn,
            )
        }
        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
        pub fn cast_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, Real)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_ray(ray_origin, ray_dir, max_toi, solid)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, RayIntersection)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_ray_and_get_normal(ray_origin, ray_dir, max_toi, solid)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_point`].
        ///
        /// Stops the query if `callback` returns false.
        pub fn intersect_point(
            &self,
            point: Vect,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_point(point) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_ray`].
        ///
        /// Stops the query if `callback` returns false.
        pub fn intersect_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider, RayIntersection) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co, intersection) in
                    query_pipeline.intersect_ray(ray_origin, ray_dir, max_toi, solid)
                {
                    if !callback(e, co, intersection) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_shape`].
        pub fn intersect_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_shape(shape_pos, shape_rot, shape) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_aabb_conservative`].
        pub fn intersect_aabb_conservative(
            &self,
            #[cfg(feature = "dim2")] aabb: bevy::shape::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::shape::Aabb3d,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_aabb_conservative(aabb) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &(impl AsShape + ?Sized),
            options: ShapeCastOptions,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_shape(shape_pos, shape_rot, shape_vel, shape, options)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            point: Vect,
            max_dist: f32,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.project_point(point, max_dist, solid)
            })
        }
    }

    impl RapierContext<'_> {
        /// Shortcut to [`RapierQueryPipeline::project_point_and_get_feature`].
        pub fn project_point_and_get_feature(
            &self,
            point: Vect,
            max_dist: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection, FeatureId)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.project_point_and_get_feature(point, max_dist)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape_nonlinear`].
        pub fn cast_shape_nonlinear(
            &self,
            shape_motion: &NonlinearMotion,
            shape: &(impl AsShape + ?Sized),
            start_time: Real,
            end_time: Real,
            stop_at_penetration: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_shape_nonlinear(
                    shape_motion,
                    shape,
                    start_time,
                    end_time,
                    stop_at_penetration,
                )
            })
        }

        /// Shortcut to [`RapierQueryPipeline::distance_to_shape`].
        pub fn distance_to_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            filter: QueryFilter,
        ) -> Option<(Entity, Real)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.distance_to_shape(shape_pos, shape_rot, shape)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::closest_points_to_shape`].
        pub fn closest_points_to_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            max_dist: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeClosestPoints)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.closest_points_to_shape(shape_pos, shape_rot, shape, max_dist)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::contact_with_shape`].
        pub fn contact_with_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            prediction: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeContact)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.contact_with_shape(shape_pos, shape_rot, shape, prediction)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::bvh`].
        pub fn bvh(&self) -> &Bvh {
            self.simulation
                .broad_phase
                .as_query_pipeline(
                    self.simulation.query_dispatcher(),
                    &self.rigidbody_set.bodies,
                    &self.colliders.colliders,
                    Default::default(),
                )
                .bvh
        }
    }

    impl RapierContextMut<'_> {
        /// Reborrows this context immutably.
        fn as_read(&self) -> RapierContext<'_> {
            RapierContext {
                simulation: &self.simulation,
                colliders: &self.colliders,
                joints: &self.joints,
                rigidbody_set: &self.rigidbody_set,
            }
        }

        /// Shortcut to [`RapierQueryPipeline::project_point_and_get_feature`].
        pub fn project_point_and_get_feature(
            &self,
            point: Vect,
            max_dist: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection, FeatureId)> {
            self.as_read()
                .project_point_and_get_feature(point, max_dist, filter)
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape_nonlinear`].
        pub fn cast_shape_nonlinear(
            &self,
            shape_motion: &NonlinearMotion,
            shape: &(impl AsShape + ?Sized),
            start_time: Real,
            end_time: Real,
            stop_at_penetration: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.as_read().cast_shape_nonlinear(
                shape_motion,
                shape,
                start_time,
                end_time,
                stop_at_penetration,
                filter,
            )
        }

        /// Shortcut to [`RapierQueryPipeline::distance_to_shape`].
        pub fn distance_to_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            filter: QueryFilter,
        ) -> Option<(Entity, Real)> {
            self.as_read()
                .distance_to_shape(shape_pos, shape_rot, shape, filter)
        }

        /// Shortcut to [`RapierQueryPipeline::closest_points_to_shape`].
        pub fn closest_points_to_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            max_dist: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeClosestPoints)> {
            self.as_read()
                .closest_points_to_shape(shape_pos, shape_rot, shape, max_dist, filter)
        }

        /// Shortcut to [`RapierQueryPipeline::contact_with_shape`].
        pub fn contact_with_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            prediction: Real,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeContact)> {
            self.as_read()
                .contact_with_shape(shape_pos, shape_rot, shape, prediction, filter)
        }

        /// Shortcut to [`RapierQueryPipeline::bvh`].
        pub fn bvh(&self) -> &Bvh {
            self.simulation
                .broad_phase
                .as_query_pipeline(
                    self.simulation.query_dispatcher(),
                    &self.rigidbody_set.bodies,
                    &self.colliders.colliders,
                    Default::default(),
                )
                .bvh
        }
    }

    // Copied from `RapierContext`.
    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierQueryPipeline::cast_ray`].
        pub fn cast_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, Real)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_ray(ray_origin, ray_dir, max_toi, solid)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::cast_ray_and_get_normal`].
        pub fn cast_ray_and_get_normal(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, RayIntersection)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_ray_and_get_normal(ray_origin, ray_dir, max_toi, solid)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_point`].
        ///
        /// Stops the query if `callback` returns false.
        pub fn intersect_point(
            &self,
            point: Vect,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_point(point) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_ray`].
        ///
        /// Stops the query if `callback` returns false.
        pub fn intersect_ray(
            &self,
            ray_origin: Vect,
            ray_dir: Vect,
            max_toi: Real,
            solid: bool,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider, RayIntersection) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co, intersection) in
                    query_pipeline.intersect_ray(ray_origin, ray_dir, max_toi, solid)
                {
                    if !callback(e, co, intersection) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_shape`].
        pub fn intersect_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape: &(impl AsShape + ?Sized),
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_shape(shape_pos, shape_rot, shape) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::intersect_aabb_conservative`].
        pub fn intersect_aabb_conservative(
            &self,
            #[cfg(feature = "dim2")] aabb: bevy::shape::Aabb2d,
            #[cfg(feature = "dim3")] aabb: bevy::shape::Aabb3d,
            filter: QueryFilter,
            mut callback: impl FnMut(Entity, &RapierCollider) -> bool,
        ) {
            self.with_query_pipeline(filter, |query_pipeline| {
                for (e, co) in query_pipeline.intersect_aabb_conservative(aabb) {
                    if !callback(e, co) {
                        break;
                    }
                }
            });
        }

        /// Shortcut to [`RapierQueryPipeline::cast_shape`].
        pub fn cast_shape(
            &self,
            shape_pos: Vect,
            shape_rot: Rot,
            shape_vel: Vect,
            shape: &(impl AsShape + ?Sized),
            options: ShapeCastOptions,
            filter: QueryFilter,
        ) -> Option<(Entity, ShapeCastHit)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.cast_shape(shape_pos, shape_rot, shape_vel, shape, options)
            })
        }

        /// Shortcut to [`RapierQueryPipeline::project_point`].
        pub fn project_point(
            &self,
            point: Vect,
            max_dist: f32,
            solid: bool,
            filter: QueryFilter,
        ) -> Option<(Entity, PointProjection)> {
            self.with_query_pipeline(filter, |query_pipeline| {
                query_pipeline.project_point(point, max_dist, solid)
            })
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [RapierQueryPipeline::new_scoped].
        pub fn with_query_pipeline<'a, T>(
            &'a self,
            filter: QueryFilter<'a>,
            scoped_fn: impl FnOnce(RapierQueryPipeline<'_>) -> T,
        ) -> T {
            crate::prelude::RapierQueryPipeline::new_scoped(
                &self.simulation.broad_phase,
                &self.colliders,
                &self.rigidbody_set,
                &filter,
                self.simulation.query_dispatcher(),
                scoped_fn,
            )
        }
    }
}

mod rigidbody_set {
    use std::collections::HashMap;

    use super::*;
    pub use rapier::prelude::RigidBodyHandle;

    impl RapierContext<'_> {
        /// Shortcut to [`RapierRigidBodySet::entity2body`].
        pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
            self.rigidbody_set.entity2body()
        }

        /// Shortcut to [`RapierRigidBodySet::rigid_body_entity`].
        pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
            self.rigidbody_set.rigid_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::impulse_revolute_joint_angle`].
        pub fn impulse_revolute_joint_angle(&self, entity: Entity) -> Option<f32> {
            self.rigidbody_set
                .impulse_revolute_joint_angle(self.joints, entity)
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierRigidBodySet::propagate_modified_body_positions_to_colliders`].
        pub fn propagate_modified_body_positions_to_colliders(&mut self) {
            self.rigidbody_set
                .propagate_modified_body_positions_to_colliders(&mut self.colliders)
        }

        /// Shortcut to [`RapierRigidBodySet::entity2body`].
        pub fn entity2body(&self) -> &HashMap<Entity, RigidBodyHandle> {
            self.rigidbody_set.entity2body()
        }

        /// Shortcut to [`RapierRigidBodySet::rigid_body_entity`].
        pub fn rigid_body_entity(&self, handle: RigidBodyHandle) -> Option<Entity> {
            self.rigidbody_set.rigid_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::impulse_revolute_joint_angle`].
        pub fn impulse_revolute_joint_angle(&self, entity: Entity) -> Option<f32> {
            self.rigidbody_set
                .impulse_revolute_joint_angle(&self.joints, entity)
        }
    }
}

mod soft_bodies {
    use super::*;
    use crate::dynamics::RapierSoftBody;
    use crate::pipeline::SoftBodyTearResult;
    use rapier::dynamics::{RigidBodyHandle, SoftBodyHandle};
    use std::collections::HashMap;

    impl RapierContext<'_> {
        /// Shortcut to [`RapierRigidBodySet::entity2soft_body`].
        pub fn entity2soft_body(&self) -> &HashMap<Entity, SoftBodyHandle> {
            self.rigidbody_set.entity2soft_body()
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_entity`].
        pub fn soft_body_entity(&self, handle: SoftBodyHandle) -> Option<Entity> {
            self.rigidbody_set.soft_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body`].
        pub fn soft_body(&self, entity: Entity) -> Option<&RapierSoftBody> {
            self.rigidbody_set.soft_body(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_particle_positions`].
        pub fn soft_body_particle_positions(
            &self,
            entity: Entity,
        ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
            self.rigidbody_set.soft_body_particle_positions(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_particle_velocities`].
        pub fn soft_body_particle_velocities(
            &self,
            entity: Entity,
        ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
            self.rigidbody_set.soft_body_particle_velocities(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_volume`].
        pub fn soft_body_volume(&self, entity: Entity) -> Option<Real> {
            self.rigidbody_set.soft_body_volume(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_mass`].
        pub fn soft_body_mass(&self, entity: Entity) -> Option<Real> {
            self.rigidbody_set.soft_body_mass(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_center_of_mass`].
        pub fn soft_body_center_of_mass(&self, entity: Entity) -> Option<Vect> {
            self.rigidbody_set.soft_body_center_of_mass(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::is_soft_body_sleeping`].
        pub fn is_soft_body_sleeping(&self, entity: Entity) -> Option<bool> {
            self.rigidbody_set.is_soft_body_sleeping(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_whole_proxy`].
        pub fn soft_body_whole_proxy(&self, entity: Entity) -> Option<RigidBodyHandle> {
            self.rigidbody_set.soft_body_whole_proxy(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_cluster_index`].
        pub fn soft_body_cluster_index(&self, entity: Entity) -> Option<(SoftBodyHandle, u32)> {
            self.rigidbody_set.soft_body_cluster_index(entity)
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierRigidBodySet::entity2soft_body`].
        pub fn entity2soft_body(&self) -> &HashMap<Entity, SoftBodyHandle> {
            self.rigidbody_set.entity2soft_body()
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_entity`].
        pub fn soft_body_entity(&self, handle: SoftBodyHandle) -> Option<Entity> {
            self.rigidbody_set.soft_body_entity(handle)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body`].
        pub fn soft_body(&self, entity: Entity) -> Option<&RapierSoftBody> {
            self.rigidbody_set.soft_body(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_particle_positions`].
        pub fn soft_body_particle_positions(
            &self,
            entity: Entity,
        ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
            self.rigidbody_set.soft_body_particle_positions(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_particle_velocities`].
        pub fn soft_body_particle_velocities(
            &self,
            entity: Entity,
        ) -> Option<impl ExactSizeIterator<Item = Vect> + '_> {
            self.rigidbody_set.soft_body_particle_velocities(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_volume`].
        pub fn soft_body_volume(&self, entity: Entity) -> Option<Real> {
            self.rigidbody_set.soft_body_volume(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_mass`].
        pub fn soft_body_mass(&self, entity: Entity) -> Option<Real> {
            self.rigidbody_set.soft_body_mass(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_center_of_mass`].
        pub fn soft_body_center_of_mass(&self, entity: Entity) -> Option<Vect> {
            self.rigidbody_set.soft_body_center_of_mass(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::is_soft_body_sleeping`].
        pub fn is_soft_body_sleeping(&self, entity: Entity) -> Option<bool> {
            self.rigidbody_set.is_soft_body_sleeping(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_whole_proxy`].
        pub fn soft_body_whole_proxy(&self, entity: Entity) -> Option<RigidBodyHandle> {
            self.rigidbody_set.soft_body_whole_proxy(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_cluster_index`].
        pub fn soft_body_cluster_index(&self, entity: Entity) -> Option<(SoftBodyHandle, u32)> {
            self.rigidbody_set.soft_body_cluster_index(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::soft_body_mut`].
        pub fn soft_body_mut(&mut self, entity: Entity) -> Option<&mut RapierSoftBody> {
            self.rigidbody_set.soft_body_mut(entity)
        }

        /// Shortcut to [`RapierRigidBodySet::wake_up_soft_body`].
        pub fn wake_up_soft_body(&mut self, entity: Entity, strong: bool) -> bool {
            self.rigidbody_set.wake_up_soft_body(entity, strong)
        }

        /// Shortcut to [`RapierContextSimulation::tear_soft_body`].
        pub fn tear_soft_body(
            &mut self,
            commands: &mut Commands,
            entity: Entity,
            edges: &[u32],
            cells: &[u32],
        ) -> Option<SoftBodyTearResult> {
            self.simulation.tear_soft_body(
                &mut self.colliders,
                &mut self.joints,
                &mut self.rigidbody_set,
                commands,
                entity,
                edges,
                cells,
            )
        }

        /// Shortcut to [`RapierContextSimulation::cut_soft_body`].
        pub fn cut_soft_body(
            &mut self,
            commands: &mut Commands,
            entity: Entity,
            blade: &[Vect; rapier::math::DIM],
        ) -> Option<SoftBodyTearResult> {
            self.simulation.cut_soft_body(
                &mut self.colliders,
                &mut self.joints,
                &mut self.rigidbody_set,
                commands,
                entity,
                blade,
            )
        }
    }
}

mod joints {
    use super::*;
    use crate::dynamics::{ImpulseJointImpulses, InverseKinematicsOption, MultibodyJointState};
    use rapier::dynamics::Multibody;
    use rapier::math::Jacobian;
    use std::ops::Range;

    impl RapierContext<'_> {
        /// Shortcut to [`RapierContextJoints::impulse_joint_impulses`].
        pub fn impulse_joint_impulses(&self, entity: Entity) -> Option<ImpulseJointImpulses> {
            self.joints.impulse_joint_impulses(entity)
        }

        /// Shortcut to [`RapierContextJoints::impulse_joints_between`].
        pub fn impulse_joints_between(
            &self,
            body1: Entity,
            body2: Entity,
        ) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .impulse_joints_between(self.rigidbody_set, body1, body2)
        }

        /// Shortcut to [`RapierContextJoints::attached_impulse_joints`].
        pub fn attached_impulse_joints(&self, body: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_impulse_joints(self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::attached_enabled_impulse_joints`].
        pub fn attached_enabled_impulse_joints(
            &self,
            body: Entity,
        ) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_enabled_impulse_joints(self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::multibody`].
        pub fn multibody(&self, entity: Entity) -> Option<(&Multibody, usize)> {
            self.joints.multibody(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_root`].
        pub fn multibody_root(&self, entity: Entity) -> Option<Entity> {
            self.joints.multibody_root(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_links`].
        pub fn multibody_links(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints.multibody_links(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_parent_link`].
        pub fn multibody_parent_link(&self, entity: Entity) -> Option<Entity> {
            self.joints
                .multibody_parent_link(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_joint_between`].
        pub fn multibody_joint_between(&self, body1: Entity, body2: Entity) -> Option<Entity> {
            self.joints
                .multibody_joint_between(self.rigidbody_set, body1, body2)
        }

        /// Shortcut to [`RapierContextJoints::attached_multibody_joints`].
        pub fn attached_multibody_joints(&self, body: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_multibody_joints(self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::multibody_link_dofs`].
        pub fn multibody_link_dofs(&self, entity: Entity) -> Option<Range<usize>> {
            self.joints.multibody_link_dofs(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_generalized_velocity`].
        pub fn multibody_generalized_velocity(&self, entity: Entity) -> Option<&[Real]> {
            self.joints
                .multibody_generalized_velocity(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_joint_state`].
        pub fn multibody_joint_state(&self, entity: Entity) -> Option<MultibodyJointState> {
            self.joints.multibody_joint_state(entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_body_jacobian`].
        pub fn multibody_body_jacobian(&self, entity: Entity) -> Option<&Jacobian<Real>> {
            self.joints
                .multibody_body_jacobian(self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_inverse_kinematics`].
        pub fn multibody_inverse_kinematics(
            &self,
            link: Entity,
            target: Transform,
            options: &InverseKinematicsOption,
            joint_can_move: impl Fn(Entity) -> bool,
            displacements: &mut Vec<Real>,
        ) -> bool {
            self.joints.multibody_inverse_kinematics(
                self.rigidbody_set,
                link,
                target,
                options,
                joint_can_move,
                displacements,
            )
        }
    }

    impl RapierContextMut<'_> {
        /// Shortcut to [`RapierContextJoints::impulse_joint_impulses`].
        pub fn impulse_joint_impulses(&self, entity: Entity) -> Option<ImpulseJointImpulses> {
            self.joints.impulse_joint_impulses(entity)
        }

        /// Shortcut to [`RapierContextJoints::impulse_joints_between`].
        pub fn impulse_joints_between(
            &self,
            body1: Entity,
            body2: Entity,
        ) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .impulse_joints_between(&self.rigidbody_set, body1, body2)
        }

        /// Shortcut to [`RapierContextJoints::attached_impulse_joints`].
        pub fn attached_impulse_joints(&self, body: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_impulse_joints(&self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::attached_enabled_impulse_joints`].
        pub fn attached_enabled_impulse_joints(
            &self,
            body: Entity,
        ) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_enabled_impulse_joints(&self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::multibody`].
        pub fn multibody(&self, entity: Entity) -> Option<(&Multibody, usize)> {
            self.joints.multibody(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_root`].
        pub fn multibody_root(&self, entity: Entity) -> Option<Entity> {
            self.joints.multibody_root(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_links`].
        pub fn multibody_links(&self, entity: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints.multibody_links(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_parent_link`].
        pub fn multibody_parent_link(&self, entity: Entity) -> Option<Entity> {
            self.joints
                .multibody_parent_link(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_joint_between`].
        pub fn multibody_joint_between(&self, body1: Entity, body2: Entity) -> Option<Entity> {
            self.joints
                .multibody_joint_between(&self.rigidbody_set, body1, body2)
        }

        /// Shortcut to [`RapierContextJoints::attached_multibody_joints`].
        pub fn attached_multibody_joints(&self, body: Entity) -> impl Iterator<Item = Entity> + '_ {
            self.joints
                .attached_multibody_joints(&self.rigidbody_set, body)
        }

        /// Shortcut to [`RapierContextJoints::multibody_link_dofs`].
        pub fn multibody_link_dofs(&self, entity: Entity) -> Option<Range<usize>> {
            self.joints.multibody_link_dofs(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_generalized_velocity`].
        pub fn multibody_generalized_velocity(&self, entity: Entity) -> Option<&[Real]> {
            self.joints
                .multibody_generalized_velocity(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_joint_state`].
        pub fn multibody_joint_state(&self, entity: Entity) -> Option<MultibodyJointState> {
            self.joints.multibody_joint_state(entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_body_jacobian`].
        pub fn multibody_body_jacobian(&self, entity: Entity) -> Option<&Jacobian<Real>> {
            self.joints
                .multibody_body_jacobian(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_inverse_kinematics`].
        pub fn multibody_inverse_kinematics(
            &self,
            link: Entity,
            target: Transform,
            options: &InverseKinematicsOption,
            joint_can_move: impl Fn(Entity) -> bool,
            displacements: &mut Vec<Real>,
        ) -> bool {
            self.joints.multibody_inverse_kinematics(
                &self.rigidbody_set,
                link,
                target,
                options,
                joint_can_move,
                displacements,
            )
        }

        /// Shortcut to [`RapierContextJoints::multibody_mut`].
        pub fn multibody_mut(&mut self, entity: Entity) -> Option<(&mut Multibody, usize)> {
            self.joints.multibody_mut(&self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_generalized_velocity_mut`].
        pub fn multibody_generalized_velocity_mut(
            &mut self,
            entity: Entity,
        ) -> Option<&mut [Real]> {
            self.joints
                .multibody_generalized_velocity_mut(&mut self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_joint_velocity_mut`].
        pub fn multibody_joint_velocity_mut(&mut self, entity: Entity) -> Option<&mut [Real]> {
            self.joints
                .multibody_joint_velocity_mut(&mut self.rigidbody_set, entity)
        }

        /// Shortcut to [`RapierContextJoints::multibody_apply_displacements`].
        pub fn multibody_apply_displacements(
            &mut self,
            entity: Entity,
            displacements: &[Real],
        ) -> bool {
            self.joints
                .multibody_apply_displacements(&self.rigidbody_set, entity, displacements)
        }

        /// Shortcut to [`RapierContextJoints::multibody_forward_kinematics`].
        pub fn multibody_forward_kinematics(
            &mut self,
            entity: Entity,
            read_root_pose_from_rigid_body: bool,
        ) -> bool {
            self.joints.multibody_forward_kinematics(
                &mut self.rigidbody_set,
                entity,
                read_root_pose_from_rigid_body,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Collider, NonlinearMotion, ShapeCastOptions, ShapeClosestPoints};
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use crate::prelude::QueryFilter;
    use approx::assert_relative_eq;
    use bevy::ecs::system::RunSystemOnce;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};

    #[cfg(feature = "dim2")]
    fn v(x: Real, y: Real) -> Vect {
        Vect::new(x, y)
    }
    #[cfg(feature = "dim3")]
    fn v(x: Real, y: Real) -> Vect {
        Vect::new(x, y, 0.0)
    }

    #[cfg(feature = "dim2")]
    fn quarter_turn() -> Rot {
        std::f32::consts::FRAC_PI_2
    }
    #[cfg(feature = "dim3")]
    fn quarter_turn() -> Rot {
        Rot::from_rotation_z(std::f32::consts::FRAC_PI_2)
    }

    #[cfg(feature = "dim2")]
    fn unit_cuboid() -> Collider {
        Collider::cuboid(0.5, 0.5)
    }
    #[cfg(feature = "dim3")]
    fn unit_cuboid() -> Collider {
        Collider::cuboid(0.5, 0.5, 0.5)
    }

    fn transform_at(x: Real, y: Real) -> Transform {
        Transform::from_xyz(x, y, 0.0)
    }

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
        app.finish();
        app.update();
        app
    }

    /// Spawns a fixed ball at the origin and a fixed cuboid at `x = 10`, then steps the world.
    fn ball_and_cuboid_app() -> (App, Entity, Entity) {
        let mut app = test_app();
        let ball = app
            .world_mut()
            .spawn((transform_at(0.0, 0.0), Collider::ball(0.5)))
            .id();
        let cuboid = app
            .world_mut()
            .spawn((transform_at(10.0, 0.0), unit_cuboid()))
            .id();
        app.update();
        app.update();
        (app, ball, cuboid)
    }

    fn run<T: Send + 'static>(
        app: &mut App,
        f: impl Fn(RapierContext) -> T + Send + Sync + 'static,
    ) -> T {
        app.world_mut()
            .run_system_once(move |context: ReadRapierContext| f(context.single().unwrap()))
            .unwrap()
    }

    #[test]
    fn cast_shape_nonlinear_hits_the_first_collider() {
        let (mut app, ball, _) = ball_and_cuboid_app();
        let (from_context, from_pipeline) = run(&mut app, |context| {
            let shape = Collider::ball(0.5);
            let mut motion = NonlinearMotion::constant_position(v(-5.0, 0.0), Rot::default());
            motion.linear_velocity = v(1.0, 0.0);
            let from_context = context.cast_shape_nonlinear(
                &motion,
                &*shape.raw,
                0.0,
                10.0,
                true,
                QueryFilter::default(),
            );
            let from_pipeline = context.with_query_pipeline(QueryFilter::default(), |pipeline| {
                pipeline.cast_shape_nonlinear(&motion, &*shape.raw, 0.0, 2.0, true)
            });
            (from_context, from_pipeline)
        });

        let (entity, hit) = from_context.unwrap();
        assert_eq!(entity, ball);
        assert_relative_eq!(hit.time_of_impact, 4.0, epsilon = 1.0e-2);
        // The end time is too short to reach the ball.
        assert!(from_pipeline.is_none());
    }

    #[test]
    fn scene_queries_report_subshapes() {
        let mut app = test_app();
        let compound = app
            .world_mut()
            .spawn((
                transform_at(0.0, 0.0),
                Collider::compound(vec![
                    (v(0.0, 0.0), Rot::default(), Collider::ball(0.5)),
                    (v(3.0, 0.0), Rot::default(), Collider::ball(0.5)),
                ]),
            ))
            .id();
        app.update();
        app.update();

        run(&mut app, move |context| {
            let filter = QueryFilter::default();
            let (e, inter) = context
                .cast_ray_and_get_normal(v(3.0, 5.0), v(0.0, -1.0), Real::MAX, true, filter)
                .unwrap();
            assert_eq!(e, compound);
            assert_eq!(inter.subshape, 1);

            let mut ray_subshapes = vec![];
            context.intersect_ray(
                v(3.0, 5.0),
                v(0.0, -1.0),
                Real::MAX,
                true,
                filter,
                |_, _, i| {
                    ray_subshapes.push(i.subshape);
                    true
                },
            );
            assert_eq!(ray_subshapes, vec![1]);

            let (_, proj) = context
                .project_point(v(3.0, 2.0), Real::MAX, true, filter)
                .unwrap();
            assert_eq!(proj.subshape, 1);
            let (_, proj) = context
                .project_point(v(0.0, 2.0), Real::MAX, true, filter)
                .unwrap();
            assert_eq!(proj.subshape, 0);

            let shape = Collider::ball(0.25);
            let (_, hit) = context
                .cast_shape(
                    v(3.0, 5.0),
                    Rot::default(),
                    v(0.0, -1.0),
                    &*shape.raw,
                    ShapeCastOptions::default(),
                    filter,
                )
                .unwrap();
            assert_eq!(hit.subshape1, 1);
            assert_eq!(hit.subshape2, 0);

            let (_, contact) = context
                .contact_with_shape(v(3.0, 1.0), Rot::default(), &*shape.raw, 1.0, filter)
                .unwrap();
            assert_eq!(contact.subshape1, 1);
        });
    }

    #[test]
    fn project_point_and_get_feature_honors_max_dist() {
        let (mut app, ball, _) = ball_and_cuboid_app();
        run(&mut app, move |context| {
            let filter = QueryFilter::default();
            assert!(context
                .project_point_and_get_feature(v(0.0, 2.5), 1.0, filter)
                .is_none());
            let (e, proj, _) = context
                .project_point_and_get_feature(v(0.0, 2.5), 3.0, filter)
                .unwrap();
            assert_eq!(e, ball);
            assert_relative_eq!(proj.point, v(0.0, 0.5), epsilon = 1.0e-4);

            let from_pipeline = context.with_query_pipeline(filter, |pipeline| {
                pipeline.project_point_and_get_feature(v(0.0, 2.5), 1.0)
            });
            assert!(from_pipeline.is_none());
        });
    }

    #[test]
    fn predicate_receives_the_collider() {
        let (mut app, ball, cuboid) = ball_and_cuboid_app();
        run(&mut app, move |context| {
            let hits = |filter: QueryFilter| {
                let mut hits = vec![];
                context.intersect_ray(
                    v(-5.0, 0.0),
                    v(1.0, 0.0),
                    Real::MAX,
                    true,
                    filter,
                    |e, _, _| {
                        hits.push(e);
                        true
                    },
                );
                hits.sort();
                hits
            };
            let mut all = vec![ball, cuboid];
            all.sort();

            let is_cuboid =
                |_: Entity, co: &rapier::geometry::Collider| co.shape().as_cuboid().is_some();
            let not_cuboid = |e: Entity, _: &rapier::geometry::Collider| e != cuboid;

            assert_eq!(hits(QueryFilter::default()), all);
            assert_eq!(
                hits(QueryFilter::default().predicate(&is_cuboid)),
                vec![cuboid]
            );
            assert_eq!(
                hits(QueryFilter::default().predicate(&not_cuboid)),
                vec![ball]
            );
        });
    }

    #[test]
    fn intersect_queries_yield_colliders() {
        let (mut app, ball, cuboid) = ball_and_cuboid_app();
        run(&mut app, move |context| {
            let filter = QueryFilter::default();
            let is_ball = |co: &rapier::geometry::Collider| co.shape().as_ball().is_some();

            let mut point_hits = vec![];
            context.intersect_point(v(0.0, 0.0), filter, |e, co| {
                point_hits.push((e, is_ball(co)));
                true
            });
            assert_eq!(point_hits, vec![(ball, true)]);

            let mut ray_hits = vec![];
            context.intersect_ray(
                v(10.0, 5.0),
                v(0.0, -1.0),
                Real::MAX,
                true,
                filter,
                |e, co, inter| {
                    ray_hits.push((e, is_ball(co), inter.time_of_impact));
                    true
                },
            );
            assert_eq!(ray_hits.len(), 1);
            assert_eq!(ray_hits[0].0, cuboid);
            assert!(!ray_hits[0].1);
            assert_relative_eq!(ray_hits[0].2, 4.5, epsilon = 1.0e-4);

            let shape = Collider::ball(1.0);
            let mut shape_hits = vec![];
            context.intersect_shape(
                v(10.0, 1.0),
                Rot::default(),
                &*shape.raw,
                filter,
                |e, co| {
                    shape_hits.push((e, is_ball(co)));
                    true
                },
            );
            assert_eq!(shape_hits, vec![(cuboid, false)]);

            let aabb = Collider::ball(0.1).aabb(v(0.0, 0.0), Rot::default());
            let mut aabb_hits = vec![];
            context.intersect_aabb_conservative(aabb, filter, |e, co| {
                aabb_hits.push((e, is_ball(co)));
                true
            });
            assert_eq!(aabb_hits, vec![(ball, true)]);

            // The pipeline iterators yield the same colliders.
            context.with_query_pipeline(filter, |pipeline| {
                let hits: Vec<_> = pipeline
                    .intersect_point(v(10.0, 0.0))
                    .map(|(e, co)| (e, is_ball(co)))
                    .collect();
                assert_eq!(hits, vec![(cuboid, false)]);
            });
        });
    }

    #[test]
    fn distance_closest_points_and_contact_with_scene() {
        let (mut app, ball, cuboid) = ball_and_cuboid_app();
        run(&mut app, move |context| {
            let filter = QueryFilter::default();
            let shape = Collider::ball(0.5);
            // The shape is rotated to check that its local results are moved to world-space.
            let rot = quarter_turn();

            let (e, dist) = context
                .distance_to_shape(v(2.0, 0.0), rot, &*shape.raw, filter)
                .unwrap();
            assert_eq!(e, ball);
            assert_relative_eq!(dist, 1.0, epsilon = 1.0e-4);

            let (e, dist) = context
                .distance_to_shape(
                    v(2.0, 0.0),
                    rot,
                    &*shape.raw,
                    QueryFilter::default().exclude_collider(ball),
                )
                .unwrap();
            assert_eq!(e, cuboid);
            assert_relative_eq!(dist, 7.0, epsilon = 1.0e-4);

            assert!(context
                .closest_points_to_shape(v(2.0, 0.0), rot, &*shape.raw, 0.5, filter)
                .is_none());
            let (e, pts) = context
                .closest_points_to_shape(v(2.0, 0.0), rot, &*shape.raw, 2.0, filter)
                .unwrap();
            assert_eq!(e, ball);
            let ShapeClosestPoints::WithinMargin(p1, p2) = pts else {
                panic!("unexpected closest points: {pts:?}");
            };
            assert_relative_eq!(p1, v(0.5, 0.0), epsilon = 1.0e-4);
            assert_relative_eq!(p2, v(1.5, 0.0), epsilon = 1.0e-4);
            let (_, pts) = context
                .closest_points_to_shape(v(0.5, 0.0), rot, &*shape.raw, 2.0, filter)
                .unwrap();
            assert_eq!(pts, ShapeClosestPoints::Intersecting);

            assert!(context
                .contact_with_shape(v(2.0, 0.0), rot, &*shape.raw, 0.5, filter)
                .is_none());
            let (e, contact) = context
                .contact_with_shape(v(2.0, 0.0), rot, &*shape.raw, 2.0, filter)
                .unwrap();
            assert_eq!(e, ball);
            assert_relative_eq!(contact.distance, 1.0, epsilon = 1.0e-4);
            assert_relative_eq!(contact.point1, v(0.5, 0.0), epsilon = 1.0e-4);
            assert_relative_eq!(contact.point2, v(1.5, 0.0), epsilon = 1.0e-4);
            assert_relative_eq!(contact.normal1, v(1.0, 0.0), epsilon = 1.0e-4);
            assert_relative_eq!(contact.normal2, v(-1.0, 0.0), epsilon = 1.0e-4);
        });
    }

    #[test]
    fn bvh_leaves_resolve_to_filtered_colliders() {
        let (mut app, ball, cuboid) = ball_and_cuboid_app();
        run(&mut app, move |context| {
            let leaves: Vec<u32> = context.bvh().leaves(|_| true).collect();
            assert_eq!(leaves.len(), 2);

            let filter = QueryFilter::default().exclude_collider(cuboid);
            context.with_query_pipeline(filter, |pipeline| {
                let mut resolved: Vec<_> = leaves
                    .iter()
                    .filter_map(|leaf| pipeline.bvh_leaf_collider(*leaf))
                    .map(|(e, _)| e)
                    .collect();
                resolved.sort();
                assert_eq!(resolved, vec![ball]);
                assert_eq!(pipeline.bvh().leaves(|_| true).count(), 2);
            });
        });
    }

    #[test]
    fn scene_shape_casts_report_world_space_collider_witnesses() {
        let (mut app, _, cuboid) = ball_and_cuboid_app();
        let (entity, hit) = run(&mut app, |context| {
            let shape = Collider::ball(0.5);
            context.cast_shape(
                v(5.0, 0.0),
                quarter_turn(),
                v(1.0, 0.0),
                &shape,
                ShapeCastOptions::default(),
                QueryFilter::default(),
            )
        })
        .unwrap();
        assert_eq!(entity, cuboid);
        let details = hit.details.unwrap();
        // The collider hit (the first shape) is in world-space.
        assert_relative_eq!(details.witness1, v(9.5, 0.0), epsilon = 1.0e-2);
        assert_relative_eq!(details.normal1, v(-1.0, 0.0), epsilon = 1.0e-2);
        // The cast shape (the second shape), rotated by a quarter turn, is in its local-space.
        assert_relative_eq!(details.witness2, v(0.0, -0.5), epsilon = 1.0e-2);
        assert_relative_eq!(details.normal2, v(0.0, -1.0), epsilon = 1.0e-2);
    }

    #[test]
    fn move_shape_from_a_user_system() {
        let (mut app, ball, _) = ball_and_cuboid_app();
        let (output, hits) = app
            .world_mut()
            .run_system_once(|mut context: WriteRapierContext| {
                let mut context = context.single_mut().unwrap();
                let options = crate::control::MoveShapeOptions {
                    snap_to_ground: None,
                    ..Default::default()
                };
                let mut hits = vec![];
                let output = context.move_shape(
                    v(8.0, 0.0),
                    &rapier::parry::shape::Ball::new(0.5),
                    v(-5.0, 0.0),
                    Rot::default(),
                    1.0,
                    &options,
                    QueryFilter::default(),
                    |collision| hits.push(collision.entity),
                );
                (output, hits)
            })
            .unwrap();

        // The ball at the origin stops the shape before it reaches `x = -1`.
        assert!(output.effective_translation.x < 4.0);
        assert!(output.effective_translation.x > 3.5);
        assert!(hits.contains(&ball));
    }

    /// A query dispatcher counting its shape-casts before forwarding every query to the default
    /// dispatcher.
    struct CountingDispatcher(std::sync::Arc<std::sync::atomic::AtomicUsize>);

    mod counting_dispatcher {
        use super::CountingDispatcher;
        use rapier::geometry::{ContactData, ContactManifoldData};
        use rapier::math::{Pose, Real, Vector};
        use rapier::parry::query::{
            details::NormalConstraints, ClosestPoints, Contact, ContactManifold,
            ContactManifoldsWorkspace, DefaultQueryDispatcher, NonlinearRigidMotion,
            PersistentQueryDispatcher, QueryDispatcher, ShapeCastHit, ShapeCastOptions,
            ShapeDistance, ShapeIntersection, Unsupported,
        };
        use rapier::parry::shape::Shape;
        use std::sync::atomic::Ordering;

        impl QueryDispatcher for CountingDispatcher {
            fn intersection_test(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
            ) -> Result<ShapeIntersection, Unsupported> {
                DefaultQueryDispatcher.intersection_test(pos12, g1, g2)
            }

            fn distance(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
            ) -> Result<ShapeDistance, Unsupported> {
                DefaultQueryDispatcher.distance(pos12, g1, g2)
            }

            fn contact(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
                prediction: Real,
            ) -> Result<Option<Contact>, Unsupported> {
                DefaultQueryDispatcher.contact(pos12, g1, g2, prediction)
            }

            fn closest_points(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
                max_dist: Real,
            ) -> Result<ClosestPoints, Unsupported> {
                DefaultQueryDispatcher.closest_points(pos12, g1, g2, max_dist)
            }

            fn cast_shapes(
                &self,
                pos12: &Pose,
                local_vel12: Vector,
                g1: &dyn Shape,
                g2: &dyn Shape,
                options: ShapeCastOptions,
            ) -> Result<Option<ShapeCastHit>, Unsupported> {
                self.0.fetch_add(1, Ordering::Relaxed);
                DefaultQueryDispatcher.cast_shapes(pos12, local_vel12, g1, g2, options)
            }

            fn cast_shapes_nonlinear(
                &self,
                motion1: &NonlinearRigidMotion,
                g1: &dyn Shape,
                motion2: &NonlinearRigidMotion,
                g2: &dyn Shape,
                start_time: Real,
                end_time: Real,
                stop_at_penetration: bool,
            ) -> Result<Option<ShapeCastHit>, Unsupported> {
                DefaultQueryDispatcher.cast_shapes_nonlinear(
                    motion1,
                    g1,
                    motion2,
                    g2,
                    start_time,
                    end_time,
                    stop_at_penetration,
                )
            }
        }

        impl PersistentQueryDispatcher<ContactManifoldData, ContactData> for CountingDispatcher {
            fn contact_manifolds(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
                prediction: Real,
                manifolds: &mut Vec<ContactManifold<ContactManifoldData, ContactData>>,
                workspace: &mut Option<ContactManifoldsWorkspace>,
            ) -> Result<(), Unsupported> {
                DefaultQueryDispatcher
                    .contact_manifolds(pos12, g1, g2, prediction, manifolds, workspace)
            }

            fn contact_manifold_convex_convex(
                &self,
                pos12: &Pose,
                g1: &dyn Shape,
                g2: &dyn Shape,
                normal_constraints1: Option<&dyn NormalConstraints>,
                normal_constraints2: Option<&dyn NormalConstraints>,
                prediction: Real,
                manifold: &mut ContactManifold<ContactManifoldData, ContactData>,
            ) -> Result<(), Unsupported> {
                DefaultQueryDispatcher.contact_manifold_convex_convex(
                    pos12,
                    g1,
                    g2,
                    normal_constraints1,
                    normal_constraints2,
                    prediction,
                    manifold,
                )
            }
        }
    }

    #[test]
    fn scene_queries_use_the_context_query_dispatcher() {
        let mut app = test_app();
        let casts = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let dispatcher = CountingDispatcher(casts.clone());
        app.world_mut()
            .run_system_once(move |mut context: WriteRapierContext| {
                context
                    .single_mut()
                    .unwrap()
                    .simulation
                    .set_query_dispatcher(CountingDispatcher(dispatcher.0.clone()));
            })
            .unwrap();
        let ball = app
            .world_mut()
            .spawn((transform_at(0.0, 0.0), Collider::ball(0.5)))
            .id();
        app.update();
        app.update();

        let casts_before = casts.load(std::sync::atomic::Ordering::Relaxed);
        let hit = run(&mut app, |context| {
            let shape = Collider::ball(0.5);
            context.cast_shape(
                v(-5.0, 0.0),
                Rot::default(),
                v(1.0, 0.0),
                &*shape.raw,
                ShapeCastOptions::default(),
                QueryFilter::default(),
            )
        });
        assert_eq!(hit.map(|(e, _)| e), Some(ball));
        assert!(casts.load(std::sync::atomic::Ordering::Relaxed) > casts_before);

        // The character-controller shortcut goes through the same dispatcher.
        let casts_before = casts.load(std::sync::atomic::Ordering::Relaxed);
        app.world_mut()
            .run_system_once(|mut context: WriteRapierContext| {
                let shape = Collider::ball(0.5);
                context.single_mut().unwrap().move_shape(
                    v(8.0, 0.0),
                    &*shape.raw,
                    v(-5.0, 0.0),
                    Rot::default(),
                    1.0,
                    &Default::default(),
                    QueryFilter::default(),
                    |_| {},
                );
            })
            .unwrap();
        assert!(casts.load(std::sync::atomic::Ordering::Relaxed) > casts_before);
    }
}
