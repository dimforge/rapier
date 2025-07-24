use crate::dynamics::RigidBodyHandle;
use crate::geometry::{Aabb, Collider, ColliderHandle, PointProjection, Ray, RayIntersection};
use crate::geometry::{BroadPhaseBvh, InteractionGroups};
use crate::math::{Isometry, Point, Real, Vector};
use crate::{dynamics::RigidBodySet, geometry::ColliderSet};
use parry::bounding_volume::BoundingVolume;
use parry::partitioning::{Bvh, BvhNode};
use parry::query::details::{NormalConstraints, ShapeCastOptions};
use parry::query::{NonlinearRigidMotion, QueryDispatcher, RayCast, ShapeCastHit};
use parry::shape::{CompositeShape, CompositeShapeRef, FeatureId, Shape, TypedCompositeShape};

/// The query pipeline responsible for running scene queries on the physics world.
///
/// This structure is generally obtained by calling [`BroadPhaseBvh::as_query_pipeline_mut`].
#[derive(Copy, Clone)]
pub struct QueryPipeline<'a> {
    /// The query dispatcher for running geometric queries on leaf geometries.
    pub dispatcher: &'a dyn QueryDispatcher,
    /// A bvh containing collider indices at its leaves.
    pub bvh: &'a Bvh,
    /// Rigid-bodies potentially involved in the scene queries.
    pub bodies: &'a RigidBodySet,
    /// Colliders potentially involved in the scene queries.
    pub colliders: &'a ColliderSet,
    /// The query filters for controlling what colliders should be ignored by the queries.
    pub filter: QueryFilter<'a>,
}

/// Same as [`QueryPipeline`] but holds mutable references to the body and collider sets.
///
/// This structure is generally obtained by calling [`BroadPhaseBvh::as_query_pipeline_mut`].
/// This is useful for argument passing. Call `.as_ref()` for obtaining a `QueryPipeline`
/// to run the scene queries.
pub struct QueryPipelineMut<'a> {
    /// The query dispatcher for running geometric queries on leaf geometries.
    pub dispatcher: &'a dyn QueryDispatcher,
    /// A bvh containing collider indices at its leaves.
    pub bvh: &'a Bvh,
    /// Rigid-bodies potentially involved in the scene queries.
    pub bodies: &'a mut RigidBodySet,
    /// Colliders potentially involved in the scene queries.
    pub colliders: &'a mut ColliderSet,
    /// The query filters for controlling what colliders should be ignored by the queries.
    pub filter: QueryFilter<'a>,
}

impl QueryPipelineMut<'_> {
    /// Downgrades the mutable reference to an immutable reference.
    pub fn as_ref(&self) -> QueryPipeline {
        QueryPipeline {
            dispatcher: self.dispatcher,
            bvh: self.bvh,
            bodies: &*self.bodies,
            colliders: &*self.colliders,
            filter: self.filter,
        }
    }
}

impl CompositeShape for QueryPipeline<'_> {
    fn map_part_at(
        &self,
        shape_id: u32,
        f: &mut dyn FnMut(Option<&Isometry<Real>>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        self.map_untyped_part_at(shape_id, f);
    }
    fn bvh(&self) -> &Bvh {
        self.bvh
    }
}

impl TypedCompositeShape for QueryPipeline<'_> {
    type PartNormalConstraints = ();
    type PartShape = dyn Shape;
    fn map_typed_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(
            Option<&Isometry<Real>>,
            &Self::PartShape,
            Option<&Self::PartNormalConstraints>,
        ) -> T,
    ) -> Option<T> {
        let (co, co_handle) = self.colliders.get_unknown_gen(shape_id)?;

        if self.filter.test(self.bodies, co_handle, co) {
            Some(f(Some(co.position()), co.shape(), None))
        } else {
            None
        }
    }

    fn map_untyped_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Isometry<Real>>, &dyn Shape, Option<&dyn NormalConstraints>) -> T,
    ) -> Option<T> {
        let (co, co_handle) = self.colliders.get_unknown_gen(shape_id)?;

        if self.filter.test(self.bodies, co_handle, co) {
            Some(f(Some(co.position()), co.shape(), None))
        } else {
            None
        }
    }
}

impl BroadPhaseBvh {
    /// Initialize a [`QueryPipeline`] for scene queries from this broad-phase.
    pub fn as_query_pipeline<'a>(
        &'a self,
        dispatcher: &'a dyn QueryDispatcher,
        bodies: &'a RigidBodySet,
        colliders: &'a ColliderSet,
        filter: QueryFilter<'a>,
    ) -> QueryPipeline<'a> {
        QueryPipeline {
            dispatcher,
            bvh: &self.tree,
            bodies,
            colliders,
            filter,
        }
    }

    /// Initialize a [`QueryPipelineMut`] for scene queries from this broad-phase.
    pub fn as_query_pipeline_mut<'a>(
        &'a self,
        dispatcher: &'a dyn QueryDispatcher,
        bodies: &'a mut RigidBodySet,
        colliders: &'a mut ColliderSet,
        filter: QueryFilter<'a>,
    ) -> QueryPipelineMut<'a> {
        QueryPipelineMut {
            dispatcher,
            bvh: &self.tree,
            bodies,
            colliders,
            filter,
        }
    }
}

impl<'a> QueryPipeline<'a> {
    fn id_to_handle<T>(&self, (id, data): (u32, T)) -> Option<(ColliderHandle, T)> {
        self.colliders.get_unknown_gen(id).map(|(_, h)| (h, data))
    }

    /// Replaces [`Self::filter`] with different filtering rules.
    pub fn with_filter(self, filter: QueryFilter<'a>) -> Self {
        Self { filter, ..self }
    }

    /// Find the closest intersection between a ray and a set of colliders.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    #[profiling::function]
    pub fn cast_ray(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, Real)> {
        CompositeShapeRef(self)
            .cast_local_ray(ray, max_toi, solid)
            .and_then(|hit| self.id_to_handle(hit))
    }

    /// Find the closest intersection between a ray and a set of colliders.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    /// * `filter`: set of rules used to determine which collider is taken into account by this scene query.
    #[profiling::function]
    pub fn cast_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, RayIntersection)> {
        CompositeShapeRef(self)
            .cast_local_ray_and_get_normal(ray, max_toi, solid)
            .and_then(|hit| self.id_to_handle(hit))
    }

    /// Iterates through all the colliders intersecting a given ray.
    ///
    /// # Parameters
    /// * `colliders` - The set of colliders taking part in this pipeline.
    /// * `ray`: the ray to cast.
    /// * `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// * `solid`: if this is `true` an impact at time 0.0 (i.e. at the ray origin) is returned if
    ///            it starts inside a shape. If this `false` then the ray will hit the shape's boundary
    ///            even if its starts inside of it.
    #[profiling::function]
    pub fn intersect_ray(
        &'a self,
        ray: Ray,
        max_toi: Real,
        solid: bool,
    ) -> impl Iterator<Item = (ColliderHandle, &'a Collider, RayIntersection)> + 'a {
        // TODO: add this to CompositeShapeRef?
        self.bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects_local_ray(&ray, max_toi))
            .filter_map(move |leaf| {
                let (co, co_handle) = self.colliders.get_unknown_gen(leaf)?;
                if self.filter.test(self.bodies, co_handle, co) {
                    if let Some(intersection) =
                        co.shape
                            .cast_ray_and_get_normal(co.position(), &ray, max_toi, solid)
                    {
                        return Some((co_handle, co, intersection));
                    }
                }

                None
            })
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    /// * `solid` - If this is set to `true` then the collider shapes are considered to
    ///   be plain (if the point is located inside of a plain shape, its projection is the point
    ///   itself). If it is set to `false` the collider shapes are considered to be hollow
    ///   (if the point is located inside of an hollow shape, it is projected on the shape's
    ///   boundary).
    #[profiling::function]
    pub fn project_point(
        &self,
        point: &Point<Real>,
        _max_dist: Real,
        solid: bool,
    ) -> Option<(ColliderHandle, PointProjection)> {
        self.id_to_handle(CompositeShapeRef(self).project_local_point(point, solid))
    }

    /// Find all the colliders containing the given point.
    ///
    /// # Parameters
    /// * `point` - The point used for the containment test.
    #[profiling::function]
    pub fn intersect_point(
        &'a self,
        point: Point<Real>,
    ) -> impl Iterator<Item = (ColliderHandle, &'a Collider)> + 'a {
        // TODO: add to CompositeShapeRef?
        self.bvh
            .leaves(move |node: &BvhNode| node.aabb().contains_local_point(&point))
            .filter_map(move |leaf| {
                let (co, co_handle) = self.colliders.get_unknown_gen(leaf)?;
                if self.filter.test(self.bodies, co_handle, co)
                    && co.shape.contains_point(co.position(), &point)
                {
                    return Some((co_handle, co));
                }

                None
            })
    }

    /// Find the projection of a point on the closest collider.
    ///
    /// The results include the ID of the feature hit by the point.
    ///
    /// # Parameters
    /// * `point` - The point to project.
    #[profiling::function]
    pub fn project_point_and_get_feature(
        &self,
        point: &Point<Real>,
    ) -> Option<(ColliderHandle, PointProjection, FeatureId)> {
        let (id, (proj, feat)) = CompositeShapeRef(self).project_local_point_and_get_feature(point);
        let handle = self.colliders.get_unknown_gen(id)?.1;
        Some((handle, proj, feat))
    }

    /// Finds all handles of all the colliders with an [`Aabb`] intersecting the given [`Aabb`].
    ///
    /// Note that the collider AABB taken into account is the one currently stored in the query
    /// pipeline’s BVH. It doesn’t recompute the latest collider AABB.
    #[profiling::function]
    pub fn intersect_aabb_conservative(
        &'a self,
        aabb: Aabb,
    ) -> impl Iterator<Item = (ColliderHandle, &'a Collider)> + 'a {
        // TODO: add to ColliderRef?
        self.bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects(&aabb))
            .filter_map(move |leaf| {
                let (co, co_handle) = self.colliders.get_unknown_gen(leaf)?;
                // NOTE: do **not** recompute and check the latest collider AABB.
                //       Checking only against the one in the BVH is useful, e.g., for conservative
                //       scene queries for CCD.
                if self.filter.test(self.bodies, co_handle, co) {
                    return Some((co_handle, co));
                }

                None
            })
    }

    /// Casts a shape at a constant linear velocity and retrieve the first collider it hits.
    ///
    /// This is similar to ray-casting except that we are casting a whole shape instead of just a
    /// point (the ray origin). In the resulting `TOI`, witness and normal 1 refer to the world
    /// collider, and are in world space.
    ///
    /// # Parameters
    /// * `shape_pos` - The initial position of the shape to cast.
    /// * `shape_vel` - The constant velocity of the shape to cast (i.e. the cast direction).
    /// * `shape` - The shape to cast.
    /// * `options` - Options controlling the shape cast limits and behavior.
    #[profiling::function]
    pub fn cast_shape(
        &self,
        shape_pos: &Isometry<Real>,
        shape_vel: &Vector<Real>,
        shape: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        CompositeShapeRef(self)
            .cast_shape(self.dispatcher, shape_pos, shape_vel, shape, options)
            .and_then(|hit| self.id_to_handle(hit))
    }

    /// Casts a shape with an arbitrary continuous motion and retrieve the first collider it hits.
    ///
    /// In the resulting `TOI`, witness and normal 1 refer to the world collider, and are in world
    /// space.
    ///
    /// # Parameters
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
    #[profiling::function]
    pub fn cast_shape_nonlinear(
        &self,
        shape_motion: &NonlinearRigidMotion,
        shape: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        CompositeShapeRef(self)
            .cast_shape_nonlinear(
                self.dispatcher,
                &NonlinearRigidMotion::identity(),
                shape_motion,
                shape,
                start_time,
                end_time,
                stop_at_penetration,
            )
            .and_then(|hit| self.id_to_handle(hit))
    }

    /// Retrieve all the colliders intersecting the given shape.
    ///
    /// # Parameters
    /// * `shapePos` - The pose of the shape to test.
    /// * `shape` - The shape to test.
    #[profiling::function]
    pub fn intersect_shape(
        &'a self,
        shape_pos: Isometry<Real>,
        shape: &'a dyn Shape,
    ) -> impl Iterator<Item = (ColliderHandle, &'a Collider)> + 'a {
        // TODO: add this to CompositeShapeRef?
        let shape_aabb = shape.compute_aabb(&shape_pos);
        self.bvh
            .leaves(move |node: &BvhNode| node.aabb().intersects(&shape_aabb))
            .filter_map(move |leaf| {
                let (co, co_handle) = self.colliders.get_unknown_gen(leaf)?;
                if self.filter.test(self.bodies, co_handle, co) {
                    let pos12 = shape_pos.inv_mul(co.position());
                    if self.dispatcher.intersection_test(&pos12, shape, co.shape()) == Ok(true) {
                        return Some((co_handle, co));
                    }
                }

                None
            })
    }
}

bitflags::bitflags! {
    #[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
    /// Flags for excluding whole sets of colliders from a scene query.
    pub struct QueryFilterFlags: u32 {
        /// Exclude from the query any collider attached to a fixed rigid-body and colliders with no rigid-body attached.
        const EXCLUDE_FIXED = 1 << 0;
        /// Exclude from the query any collider attached to a kinematic rigid-body.
        const EXCLUDE_KINEMATIC = 1 << 1;
        /// Exclude from the query any collider attached to a dynamic rigid-body.
        const EXCLUDE_DYNAMIC = 1 << 2;
        /// Exclude from the query any collider that is a sensor.
        const EXCLUDE_SENSORS = 1 << 3;
        /// Exclude from the query any collider that is not a sensor.
        const EXCLUDE_SOLIDS = 1 << 4;
        /// Excludes all colliders not attached to a dynamic rigid-body.
        const ONLY_DYNAMIC = Self::EXCLUDE_FIXED.bits() | Self::EXCLUDE_KINEMATIC.bits();
        /// Excludes all colliders not attached to a kinematic rigid-body.
        const ONLY_KINEMATIC = Self::EXCLUDE_DYNAMIC.bits() | Self::EXCLUDE_FIXED.bits();
        /// Exclude all colliders attached to a non-fixed rigid-body
        /// (this will not exclude colliders not attached to any rigid-body).
        const ONLY_FIXED = Self::EXCLUDE_DYNAMIC.bits() | Self::EXCLUDE_KINEMATIC.bits();
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

/// A filter that describes what collider should be included or excluded from a scene query.
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
    #[allow(clippy::type_complexity)] // Type doesn’t look really complex?
    pub predicate: Option<&'a dyn Fn(ColliderHandle, &Collider) -> bool>,
}

impl QueryFilter<'_> {
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

impl From<QueryFilterFlags> for QueryFilter<'_> {
    fn from(flags: QueryFilterFlags) -> Self {
        Self {
            flags,
            ..QueryFilter::default()
        }
    }
}

impl From<InteractionGroups> for QueryFilter<'_> {
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
