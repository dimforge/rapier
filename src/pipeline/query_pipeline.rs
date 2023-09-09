use crate::dynamics::RigidBodyHandle;
use crate::geometry::{
    Aabb, Collider, ColliderHandle, InteractionGroups, PointProjection, Qbvh, Ray, RayIntersection,
};
use crate::math::{Isometry, Point, Real, Vector};
use crate::{dynamics::RigidBodySet, geometry::ColliderSet};
use parry::partitioning::{QbvhDataGenerator, QbvhUpdateWorkspace};
use parry::query::details::{
    NonlinearTOICompositeShapeShapeBestFirstVisitor, PointCompositeShapeProjBestFirstVisitor,
    PointCompositeShapeProjWithFeatureBestFirstVisitor,
    RayCompositeShapeToiAndNormalBestFirstVisitor, RayCompositeShapeToiBestFirstVisitor,
    TOICompositeShapeShapeBestFirstVisitor,
};
use parry::query::visitors::{
    BoundingVolumeIntersectionsVisitor, PointIntersectionsVisitor, RayIntersectionsVisitor,
};
use parry::query::{DefaultQueryDispatcher, NonlinearRigidMotion, QueryDispatcher, TOI};
use parry::shape::{FeatureId, Shape, TypedSimdCompositeShape};
use parry::utils::DefaultStorage;
use std::sync::Arc;

/// A pipeline for performing queries on all the colliders of a scene.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct QueryPipeline {
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "crate::geometry::default_query_dispatcher")
    )]
    query_dispatcher: Arc<dyn QueryDispatcher>,
    qbvh: Qbvh<ColliderHandle>,
    dilation_factor: Real,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    workspace: QbvhUpdateWorkspace,
}

struct QueryPipelineAsCompositeShape<'a> {
    query_pipeline: &'a QueryPipeline,
    bodies: &'a RigidBodySet,
    colliders: &'a ColliderSet,
    filter: QueryFilter<'a>,
}

bitflags::bitflags! {
    #[derive(Default)]
    /// Flags for excluding whole sets of colliders from a scene query.
    pub struct QueryFilterFlags: u32 {
        /// Exclude from the query any collider attached to a fixed rigid-body and colliders with no rigid-body attached.
        const EXCLUDE_FIXED = 1 << 1;
        /// Exclude from the query any collider attached to a kinematic rigid-body.
        const EXCLUDE_KINEMATIC = 1 << 2;
        /// Exclude from the query any collider attached to a dynamic rigid-body.
        const EXCLUDE_DYNAMIC = 1 << 3;
        /// Exclude from the query any collider that is a sensor.
        const EXCLUDE_SENSORS = 1 << 4;
        /// Exclude from the query any collider that is not a sensor.
        const EXCLUDE_SOLIDS = 1 << 5;
        /// Excludes all colliders not attached to a dynamic rigid-body.
        const ONLY_DYNAMIC = Self::EXCLUDE_FIXED.bits | Self::EXCLUDE_KINEMATIC.bits;
        /// Excludes all colliders not attached to a kinematic rigid-body.
        const ONLY_KINEMATIC = Self::EXCLUDE_DYNAMIC.bits | Self::EXCLUDE_FIXED.bits;
        /// Exclude all colliders attached to a non-fixed rigid-body
        /// (this will not exclude colliders not attached to any rigid-body).
        const ONLY_FIXED = Self::EXCLUDE_DYNAMIC.bits | Self::EXCLUDE_KINEMATIC.bits;
    }
}

impl QueryFilterFlags {
    /// Tests if the given collider should be taken into account by a scene query, based
    /// on the flags on `self`.
    #[inline]
    pub fn test(&self, bodies: &RigidBodySet, collider: &Collider) -> bool {
        if self.is_empty() {
            // No filter.
            return true;
        }

        if (self.contains(QueryFilterFlags::EXCLUDE_SENSORS) && collider.is_sensor())
            || (self.contains(QueryFilterFlags::EXCLUDE_SOLIDS) && !collider.is_sensor())
        {
            return false;
        }

        if self.contains(QueryFilterFlags::EXCLUDE_FIXED) && collider.parent.is_none() {
            return false;
        }

        if let Some(parent) = collider.parent.and_then(|p| bodies.get(p.handle)) {
            let parent_type = parent.body_type();

            if (self.contains(QueryFilterFlags::EXCLUDE_FIXED) && parent_type.is_fixed())
                || (self.contains(QueryFilterFlags::EXCLUDE_KINEMATIC)
                    && parent_type.is_kinematic())
                || (self.contains(QueryFilterFlags::EXCLUDE_DYNAMIC) && parent_type.is_dynamic())
            {
                return false;
            }
        }

        true
    }
}

/// A filter tha describes what collider should be included or excluded from a scene query.
#[derive(Copy, Clone, Default)]
pub struct QueryFilter<'a> {
    /// Flags indicating what particular type of colliders should be excluded from the scene query.
    pub flags: QueryFilterFlags,
    /// If set, only colliders with collision groups compatible with this one will
    /// be included in the scene query.
    pub groups: Option<InteractionGroups>,
    /// If set, this collider will be excluded from the scene query.
    pub exclude_collider: Option<ColliderHandle>,
    /// If set, any collider attached to this rigid-body will be excluded from the scene query.
    pub exclude_rigid_body: Option<RigidBodyHandle>,
    /// If set, any collider for which this closure returns false will be excluded from the scene query.
    pub predicate: Option<&'a dyn Fn(ColliderHandle, &Collider) -> bool>,
}

impl<'a> QueryFilter<'a> {
    /// Applies the filters described by `self` to a collider to determine if it has to be
    /// included in a scene query (`true`) or not (`false`).
    #[inline]
    pub fn test(&self, bodies: &RigidBodySet, handle: ColliderHandle, collider: &Collider) -> bool {
        self.exclude_collider != Some(handle)
            && (self.exclude_rigid_body.is_none() // NOTE: deal with the `None` case separately otherwise the next test is incorrect if the collider’s parent is `None` too.
                || self.exclude_rigid_body != collider.parent.map(|p| p.handle))
            && self
                .groups
                .map(|grps| collider.flags.collision_groups.test(grps))
                .unwrap_or(true)
            && self.flags.test(bodies, collider)
            && self.predicate.map(|f| f(handle, collider)).unwrap_or(true)
    }
}

impl<'a> From<QueryFilterFlags> for QueryFilter<'a> {
    fn from(flags: QueryFilterFlags) -> Self {
        Self {
            flags,
            ..QueryFilter::default()
        }
    }
}

impl<'a> From<InteractionGroups> for QueryFilter<'a> {
    fn from(groups: InteractionGroups) -> Self {
        Self {
            groups: Some(groups),
            ..QueryFilter::default()
        }
    }
}

impl<'a> QueryFilter<'a> {
    /// A query filter that doesn’t exclude any collider.
    pub fn new() -> Self {
        Self::default()
    }

    /// Exclude from the query any collider attached to a fixed rigid-body and colliders with no rigid-body attached.
    pub fn exclude_fixed() -> Self {
        QueryFilterFlags::EXCLUDE_FIXED.into()
    }

    /// Exclude from the query any collider attached to a kinematic rigid-body.
    pub fn exclude_kinematic() -> Self {
        QueryFilterFlags::EXCLUDE_KINEMATIC.into()
    }

    /// Exclude from the query any collider attached to a dynamic rigid-body.
    pub fn exclude_dynamic() -> Self {
        QueryFilterFlags::EXCLUDE_DYNAMIC.into()
    }

    /// Excludes all colliders not attached to a dynamic rigid-body.
    pub fn only_dynamic() -> Self {
        QueryFilterFlags::ONLY_DYNAMIC.into()
    }

    /// Excludes all colliders not attached to a kinematic rigid-body.
    pub fn only_kinematic() -> Self {
        QueryFilterFlags::ONLY_KINEMATIC.into()
    }

    /// Exclude all colliders attached to a non-fixed rigid-body
    /// (this will not exclude colliders not attached to any rigid-body).
    pub fn only_fixed() -> Self {
        QueryFilterFlags::ONLY_FIXED.into()
    }

    /// Exclude from the query any collider that is a sensor.
    pub fn exclude_sensors(mut self) -> Self {
        self.flags |= QueryFilterFlags::EXCLUDE_SENSORS;
        self
    }

    /// Exclude from the query any collider that is not a sensor.
    pub fn exclude_solids(mut self) -> Self {
        self.flags |= QueryFilterFlags::EXCLUDE_SOLIDS;
        self
    }

    /// Only colliders with collision groups compatible with this one will
    /// be included in the scene query.
    pub fn groups(mut self, groups: InteractionGroups) -> Self {
        self.groups = Some(groups);
        self
    }

    /// Set the collider that will be excluded from the scene query.
    pub fn exclude_collider(mut self, collider: ColliderHandle) -> Self {
        self.exclude_collider = Some(collider);
        self
    }

    /// Set the rigid-body that will be excluded from the scene query.
    pub fn exclude_rigid_body(mut self, rigid_body: RigidBodyHandle) -> Self {
        self.exclude_rigid_body = Some(rigid_body);
        self
    }

    /// Set the predicate to apply a custom collider filtering during the scene query.
    pub fn predicate(mut self, predicate: &'a impl Fn(ColliderHandle, &Collider) -> bool) -> Self {
        self.predicate = Some(predicate);
        self
    }
}

/// Indicates how the colliders position should be taken into account when
/// updating the query pipeline.
pub enum QueryPipelineMode {
    /// The `Collider::position` is taken into account.
    CurrentPosition,
    /// The `RigidBody::next_position * Collider::position_wrt_parent` is taken into account for
    /// the colliders positions.
    SweepTestWithNextPosition,
    /// The `RigidBody::predict_position_using_velocity_and_forces * Collider::position_wrt_parent`
    /// is taken into account for the colliders position.
    SweepTestWithPredictedPosition {
        /// The time used to integrate the rigid-body's velocity and acceleration.
        dt: Real,
    },
}

impl<'a> TypedSimdCompositeShape for QueryPipelineAsCompositeShape<'a> {
    type PartShape = dyn Shape;
    type PartId = ColliderHandle;
    type QbvhStorage = DefaultStorage;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(Option<&Isometry<Real>>, &Self::PartShape),
    ) {
        if let Some(co) = self.colliders.get(shape_id) {
            if self.filter.test(self.bodies, shape_id, co) {
                f(Some(&co.pos), &*co.shape)
            }
        }
    }

    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        f: impl FnMut(Option<&Isometry<Real>>, &Self::PartShape),
    ) {
        self.map_typed_part_at(shape_id, f);
    }

    fn typed_qbvh(&self) -> &Qbvh<ColliderHandle> {
        &self.query_pipeline.qbvh
    }
}

impl Default for QueryPipeline {
    fn default() -> Self {
        Self::new()
    }
}

impl QueryPipeline {
    /// Initializes an empty query pipeline.
    pub fn new() -> Self {
        Self::with_query_dispatcher(DefaultQueryDispatcher)
    }

    fn as_composite_shape<'a>(
        &'a self,
        bodies: &'a RigidBodySet,
        colliders: &'a ColliderSet,
        filter: QueryFilter<'a>,
    ) -> QueryPipelineAsCompositeShape<'a> {
        QueryPipelineAsCompositeShape {
            query_pipeline: self,
            bodies,
            colliders,
            filter,
        }
    }

    /// Initializes an empty query pipeline with a custom `QueryDispatcher`.
    ///
    /// Use this constructor in order to use a custom `QueryDispatcher` that is
    /// aware of your own user-defined shapes.
    pub fn with_query_dispatcher<D>(d: D) -> Self
    where
        D: 'static + QueryDispatcher,
    {
        Self {
            query_dispatcher: Arc::new(d),
            qbvh: Qbvh::new(),
            dilation_factor: 0.01,
            workspace: QbvhUpdateWorkspace::default(),
        }
    }

    /// The query dispatcher used by this query pipeline for running scene queries.
    pub fn query_dispatcher(&self) -> &dyn QueryDispatcher {
        &*self.query_dispatcher
    }

    /// Update the query pipeline incrementally, avoiding a complete rebuild of its
    /// internal data-structure.
    pub fn update_incremental(
        &mut self,
        colliders: &ColliderSet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        refit_and_rebalance: bool,
    ) {
        // We remove first. This is needed to avoid the ABA problem: if a collider was removed
        // and another added right after with the same handle index, we can remove first, and
        // then update the new one (but only if its actually exists, to address the case where
        // a collider was added/modified and then removed during the same frame).
        for removed in removed_colliders {
            self.qbvh.remove(*removed);
        }

        for modified in modified_colliders {
            // Check that the collider still exists as it may have been removed.
            if colliders.contains(*modified) {
                self.qbvh.pre_update_or_insert(*modified);
            }
        }

        if refit_and_rebalance {
            let _ = self.qbvh.refit(0.0, &mut self.workspace, |handle| {
                colliders[*handle].compute_aabb()
            });
            self.qbvh.rebalance(0.0, &mut self.workspace);
        }
    }

    /// Update the acceleration structure on the query pipeline.
    pub fn update(&mut self, bodies: &RigidBodySet, colliders: &ColliderSet) {
        self.update_with_mode(bodies, colliders, QueryPipelineMode::CurrentPosition)
    }

    /// Update the acceleration structure on the query pipeline.
    pub fn update_with_mode(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        mode: QueryPipelineMode,
    ) {
        struct DataGenerator<'a> {
            bodies: &'a RigidBodySet,
            colliders: &'a ColliderSet,
            mode: QueryPipelineMode,
        }

        impl<'a> QbvhDataGenerator<ColliderHandle> for DataGenerator<'a> {
            fn size_hint(&self) -> usize {
                self.colliders.len()
            }

            #[inline(always)]
            fn for_each(&mut self, mut f: impl FnMut(ColliderHandle, Aabb)) {
                match self.mode {
                    QueryPipelineMode::CurrentPosition => {
                        for (h, co) in self.colliders.iter_enabled() {
                            f(h, co.shape.compute_aabb(&co.pos))
                        }
                    }
                    QueryPipelineMode::SweepTestWithNextPosition => {
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
                    QueryPipelineMode::SweepTestWithPredictedPosition { dt } => {
                        for (h, co) in self.colliders.iter_enabled() {
                            if let Some(co_parent) = co.parent {
                                let rb = &self.bodies[co_parent.handle];
                                let predicted_pos = rb.pos.integrate_forces_and_velocities(
                                    dt, &rb.forces, &rb.vels, &rb.mprops,
                                );

                                let next_position = predicted_pos * co_parent.pos_wrt_parent;
                                f(h, co.shape.compute_swept_aabb(&co.pos, &next_position))
                            } else {
                                f(h, co.shape.compute_aabb(&co.pos))
                            }
                        }
                    }
                }
            }
        }

        let generator = DataGenerator {
            bodies,
            colliders,
            mode,
        };
        self.qbvh.clear_and_rebuild(generator, self.dilation_factor);
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn cast_ray(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, Real)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let mut visitor =
            RayCompositeShapeToiBestFirstVisitor::new(&pipeline_shape, ray, max_toi, solid);

        self.qbvh.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn cast_ray_and_get_normal(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, RayIntersection)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            ray,
            max_toi,
            solid,
        );

        self.qbvh.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Find the all intersections between a ray and a set of collider and passes them to a callback.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside of a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback`: function executed on each collider for which a ray intersection has been found.
    ///               There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///               this method will exit early, ignore any further raycast.
    pub fn intersections_with_ray<'a>(
        &self,
        bodies: &'a RigidBodySet,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        filter: QueryFilter,
        mut callback: impl FnMut(ColliderHandle, RayIntersection) -> bool,
    ) {
        let mut leaf_callback = &mut |handle: &ColliderHandle| {
            if let Some(co) = colliders.get(*handle) {
                if filter.test(bodies, *handle, co) {
                    if let Some(hit) = co
                        .shape
                        .cast_ray_and_get_normal(&co.pos, ray, max_toi, solid)
                    {
                        return callback(*handle, hit);
                    }
                }
            }

            true
        };

        let mut visitor = RayIntersectionsVisitor::new(ray, max_toi, &mut leaf_callback);
        self.qbvh.traverse_depth_first(&mut visitor);
    }

    /// Gets the handle of up to one collider intersecting the given shape.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `shape_pos` - The position of the shape used for the intersection test.
    /// * `shape` - The shape used for the intersection test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn intersection_with_shape(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        filter: QueryFilter,
    ) -> Option<ColliderHandle> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        #[allow(deprecated)]
        // TODO: replace this with IntersectionCompositeShapeShapeVisitor when it
        //        can return the shape part id.
        let mut visitor =
            parry::query::details::IntersectionCompositeShapeShapeBestFirstVisitor::new(
                &*self.query_dispatcher,
                shape_pos,
                &pipeline_shape,
                shape,
            );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .0))
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `point` - The point to project.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn project_point(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        point: &Point<Real>,
        solid: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, PointProjection)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let mut visitor =
            PointCompositeShapeProjBestFirstVisitor::new(&pipeline_shape, point, solid);

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .1, h.1 .0))
    }

    /// Find all the colliders containing the given point.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `point` - The point used for the containment test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback` - A function called with each collider with a shape
    ///                containing the `point`.
    pub fn intersections_with_point(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        point: &Point<Real>,
        filter: QueryFilter,
        mut callback: impl FnMut(ColliderHandle) -> bool,
    ) {
        let mut leaf_callback = &mut |handle: &ColliderHandle| {
            if let Some(co) = colliders.get(*handle) {
                if filter.test(bodies, *handle, co) && co.shape.contains_point(&co.pos, point) {
                    return callback(*handle);
                }
            }

            true
        };

        let mut visitor = PointIntersectionsVisitor::new(point, &mut leaf_callback);

        self.qbvh.traverse_depth_first(&mut visitor);
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// The results include the ID of the feature hit by the point.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `point` - The point to project.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn project_point_and_get_feature(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        point: &Point<Real>,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, PointProjection, FeatureId)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let mut visitor =
            PointCompositeShapeProjWithFeatureBestFirstVisitor::new(&pipeline_shape, point, false);
        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .1 .0, h.1 .0, h.1 .1 .1))
    }

    /// Finds all handles of all the colliders with an Aabb intersecting the given Aabb.
    pub fn colliders_with_aabb_intersecting_aabb(
        &self,
        aabb: &Aabb,
        mut callback: impl FnMut(&ColliderHandle) -> bool,
    ) {
        let mut visitor = BoundingVolumeIntersectionsVisitor::new(aabb, &mut callback);
        self.qbvh.traverse_depth_first(&mut visitor);
    }

    /// Casts a shape at a constant linear velocity and retrieve the first collider it hits.
    ///
    /// This is similar to ray-casting except that we are casting a whole shape instead of just a
    /// point (the ray origin). In the resulting `TOI`, witness and normal 1 refer to the world
    /// collider, and are in world space.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `shape_pos` - The initial position of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the distance traveled by the shape to `shapeVel.norm() * maxToi`.
    /// * `stop_at_penetration` - If set to `false`, the linear shape-cast won’t immediately stop if
    ///   the shape is penetrating another shape at its starting point **and** its trajectory is such
    ///   that it’s on a path to exist that penetration state.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn cast_shape<'a>(
        &self,
        bodies: &RigidBodySet,
        colliders: &'a ColliderSet,
        shape_pos: &Isometry<Real>,
        shape_vel: &Vector<Real>,
        shape: &dyn Shape,
        max_toi: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, TOI)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_dispatcher,
            shape_pos,
            shape_vel,
            &pipeline_shape,
            shape,
            max_toi,
            stop_at_penetration,
        );
        self.qbvh.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Casts a shape with an arbitrary continuous motion and retrieve the first collider it hits.
    ///
    /// In the resulting `TOI`, witness and normal 1 refer to the world collider, and are in world
    /// space.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `shape_motion` - The motion of the shape.
    /// * `shape` - The shape to cast.
    /// * `start_time` - The starting time of the interval where the motion takes place.
    /// * `end_time` - The end time of the interval where the motion takes place.
    /// * `stop_at_penetration` - If the casted shape starts in a penetration state with any
    ///    collider, two results are possible. If `stop_at_penetration` is `true` then, the
    ///    result will have a `toi` equal to `start_time`. If `stop_at_penetration` is `false`
    ///    then the nonlinear shape-casting will see if further motion with respect to the penetration normal
    ///    would result in tunnelling. If it does not (i.e. we have a separating velocity along
    ///    that normal) then the nonlinear shape-casting will attempt to find another impact,
    ///    at a time `> start_time` that could result in tunnelling.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    pub fn nonlinear_cast_shape(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        shape_motion: &NonlinearRigidMotion,
        shape: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
        filter: QueryFilter,
    ) -> Option<(ColliderHandle, TOI)> {
        let pipeline_shape = self.as_composite_shape(bodies, colliders, filter);
        let pipeline_motion = NonlinearRigidMotion::identity();
        let mut visitor = NonlinearTOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_dispatcher,
            &pipeline_motion,
            &pipeline_shape,
            shape_motion,
            shape,
            start_time,
            end_time,
            stop_at_penetration,
        );
        self.qbvh.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Retrieve all the colliders intersecting the given shape.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `shapePos` - The position of the shape to test.
    /// * `shapeRot` - The orientation of the shape to test.
    /// * `shape` - The shape to test.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    /// * `callback` - A function called with the handles of each collider intersecting the `shape`.
    pub fn intersections_with_shape<'a>(
        &self,
        bodies: &RigidBodySet,
        colliders: &'a ColliderSet,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        filter: QueryFilter,
        mut callback: impl FnMut(ColliderHandle) -> bool,
    ) {
        let dispatcher = &*self.query_dispatcher;
        let inv_shape_pos = shape_pos.inverse();

        let mut leaf_callback = &mut |handle: &ColliderHandle| {
            if let Some(co) = colliders.get(*handle) {
                if filter.test(bodies, *handle, co) {
                    let pos12 = inv_shape_pos * co.pos.as_ref();

                    if dispatcher.intersection_test(&pos12, shape, &*co.shape) == Ok(true) {
                        return callback(*handle);
                    }
                }
            }

            true
        };

        let shape_aabb = shape.compute_aabb(shape_pos);
        let mut visitor = BoundingVolumeIntersectionsVisitor::new(&shape_aabb, &mut leaf_callback);

        self.qbvh.traverse_depth_first(&mut visitor);
    }
}
