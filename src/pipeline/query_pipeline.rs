use crate::cdl::motion::RigidMotion;
use crate::dynamics::RigidBodySet;
use crate::geometry::{
    Collider, ColliderHandle, ColliderSet, InteractionGroups, PointProjection, Ray,
    RayIntersection, SimdQuadTree,
};
use crate::math::{Isometry, Point, Real, Vector};
use cdl::query::details::{
    IntersectionCompositeShapeShapeBestFirstVisitor,
    NonlinearTOICompositeShapeShapeBestFirstVisitor, PointCompositeShapeProjBestFirstVisitor,
    PointCompositeShapeProjWithFeatureBestFirstVisitor,
    RayCompositeShapeToiAndNormalBestFirstVisitor, RayCompositeShapeToiBestFirstVisitor,
    TOICompositeShapeShapeBestFirstVisitor,
};
use cdl::query::{DefaultQueryDispatcher, QueryDispatcher, TOI};
use cdl::shape::{FeatureId, Shape, TypedSimdCompositeShape};
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
    quadtree: SimdQuadTree<ColliderHandle>,
    tree_built: bool,
    dilation_factor: Real,
}

struct QueryPipelineAsCompositeShape<'a> {
    query_pipeline: &'a QueryPipeline,
    colliders: &'a ColliderSet,
    groups: InteractionGroups,
}

impl<'a> TypedSimdCompositeShape for QueryPipelineAsCompositeShape<'a> {
    type PartShape = dyn Shape;
    type PartId = ColliderHandle;

    fn map_typed_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(Option<&Isometry<Real>>, &Self::PartShape),
    ) {
        if let Some(collider) = self.colliders.get(shape_id) {
            if collider.collision_groups.test(self.groups) {
                f(Some(collider.position()), collider.shape())
            }
        }
    }

    fn map_untyped_part_at(
        &self,
        shape_id: Self::PartId,
        mut f: impl FnMut(Option<&Isometry<Real>>, &Self::PartShape),
    ) {
        self.map_typed_part_at(shape_id, f);
    }

    fn typed_quadtree(&self) -> &SimdQuadTree<ColliderHandle> {
        &self.query_pipeline.quadtree
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
        colliders: &'a ColliderSet,
        groups: InteractionGroups,
    ) -> QueryPipelineAsCompositeShape<'a> {
        QueryPipelineAsCompositeShape {
            query_pipeline: self,
            colliders,
            groups,
        }
    }

    pub fn with_query_dispatcher<D>(d: D) -> Self
    where
        D: 'static + QueryDispatcher,
    {
        Self {
            query_dispatcher: Arc::new(d),
            quadtree: SimdQuadTree::new(),
            tree_built: false,
            dilation_factor: 0.01,
        }
    }

    /// Update the acceleration structure on the query pipeline.
    pub fn update(&mut self, bodies: &RigidBodySet, colliders: &ColliderSet) {
        if !self.tree_built {
            let data = colliders.iter().map(|(h, c)| (h, c.compute_aabb()));
            self.quadtree.clear_and_rebuild(data, self.dilation_factor);
            // FIXME: uncomment this once we handle insertion/removals properly.
            // self.tree_built = true;
            return;
        }

        for (_, body) in bodies
            .iter_active_dynamic()
            .chain(bodies.iter_active_kinematic())
        {
            for handle in &body.colliders {
                self.quadtree.pre_update(*handle)
            }
        }

        self.quadtree.update(
            |handle| colliders[*handle].compute_aabb(),
            self.dilation_factor,
        );
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    pub fn cast_ray(
        &self,
        colliders: &ColliderSet,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, Real)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor =
            RayCompositeShapeToiBestFirstVisitor::new(&pipeline_shape, ray, max_toi, solid);

        self.quadtree.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    pub fn cast_ray_and_get_normal(
        &self,
        colliders: &ColliderSet,
        ray: &Ray,
        max_toi: Real,
        solid: bool,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, RayIntersection)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            &pipeline_shape,
            ray,
            max_toi,
            solid,
        );

        self.quadtree.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /// Find the all intersections between a ray and a set of collider and passes them to a callback.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `Real::MAX` for an unbounded ray.
    /// - `callback`: function executed on each collider for which a ray intersection has been found.
    ///   There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///   this method will exit early, ignory any further raycast.
    pub fn intersections_with_ray<'a>(
        &self,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: Real,
        groups: InteractionGroups,
        mut callback: impl FnMut(ColliderHandle, &'a Collider, RayIntersection) -> bool,
    ) {
        // TODO: avoid allocation?
        let mut inter = Vec::new();
        self.quadtree.cast_ray(ray, max_toi, &mut inter);

        for handle in inter {
            let collider = &colliders[handle];

            if collider.collision_groups.test(groups) {
                if let Some(inter) = collider.shape().cast_ray_and_get_normal(
                    collider.position(),
                    ray,
                    max_toi,
                    true,
                ) {
                    if !callback(handle, collider, inter) {
                        return;
                    }
                }
            }
        }
    }

    /// Find up to one collider intersecting the given shape.
    fn intersection_with_shape(
        &self,
        colliders: &ColliderSet,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        groups: InteractionGroups,
    ) -> Option<ColliderHandle> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor = IntersectionCompositeShapeShapeBestFirstVisitor::new(
            &*self.query_dispatcher,
            shape_pos,
            &pipeline_shape,
            shape,
        );

        self.quadtree
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .0))
    }

    // TODO: intersections_with_point (collect all colliders containing the point).

    /// Projects a point on the scene.
    fn project_point(
        &self,
        colliders: &ColliderSet,
        point: &Point<Real>,
        solid: bool,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, PointProjection)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor =
            PointCompositeShapeProjBestFirstVisitor::new(&pipeline_shape, point, solid);

        self.quadtree
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .1, h.1 .0))
    }

    /// Projects a point on the scene and get
    fn project_point_and_get_feature(
        &self,
        colliders: &ColliderSet,
        point: &Point<Real>,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, PointProjection, FeatureId)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor =
            PointCompositeShapeProjWithFeatureBestFirstVisitor::new(&pipeline_shape, point, false);
        self.quadtree
            .traverse_best_first(&mut visitor)
            .map(|h| (h.1 .1 .0, h.1 .0, h.1 .1 .1))
    }

    pub fn cast_shape<'a>(
        &self,
        colliders: &'a ColliderSet,
        shape_pos: &Isometry<Real>,
        shape_vel: &Vector<Real>,
        shape: &dyn Shape,
        max_toi: Real,
        target_distance: Real,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, TOI)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor = TOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_dispatcher,
            shape_pos,
            shape_vel,
            &pipeline_shape,
            shape,
            max_toi,
            target_distance,
        );
        self.quadtree.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    pub fn nonlinear_cast_shape(
        &self,
        colliders: &ColliderSet,
        shape_motion: &dyn RigidMotion,
        shape: &dyn Shape,
        max_toi: Real,
        target_distance: Real,
        groups: InteractionGroups,
    ) -> Option<(ColliderHandle, TOI)> {
        let pipeline_shape = self.as_composite_shape(colliders, groups);
        let mut visitor = NonlinearTOICompositeShapeShapeBestFirstVisitor::new(
            &*self.query_dispatcher,
            shape_motion,
            &pipeline_shape,
            shape,
            max_toi,
            target_distance,
        );
        self.quadtree.traverse_best_first(&mut visitor).map(|h| h.1)
    }

    /*
    /// Gets all the colliders with a shape intersecting the given `shape`.
    pub fn intersections_with_shape<'a>(
        &self,
        colliders: &'a ColliderSet,
        shape_pos: &Isometry<Real>,
        shape: &dyn Shape,
        groups: InteractionGroups,
        mut callback: impl FnMut(ColliderHandle, &'a Collider) -> bool,
    ) {
    }
     */
}
