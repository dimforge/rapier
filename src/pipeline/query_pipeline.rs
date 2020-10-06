use crate::dynamics::RigidBodySet;
use crate::geometry::{Collider, ColliderHandle, ColliderSet, Ray, RayIntersection, WQuadtree};

/// A pipeline for performing queries on all the colliders of a scene.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct QueryPipeline {
    quadtree: WQuadtree<ColliderHandle>,
    tree_built: bool,
    dilation_factor: f32,
}

impl Default for QueryPipeline {
    fn default() -> Self {
        Self::new()
    }
}

impl QueryPipeline {
    /// Initializes an empty query pipeline.
    pub fn new() -> Self {
        Self {
            quadtree: WQuadtree::new(),
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

        self.quadtree.update(colliders, self.dilation_factor);
    }

    /// Find the closest intersection between a ray and a set of collider.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `f32::MAX` for an unbounded ray.
    pub fn cast_ray<'a>(
        &self,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: f32,
    ) -> Option<(ColliderHandle, &'a Collider, RayIntersection)> {
        // TODO: avoid allocation?
        let mut inter = Vec::new();
        self.quadtree.cast_ray(ray, max_toi, &mut inter);

        let mut best = f32::MAX;
        let mut result = None;

        for handle in inter {
            let collider = &colliders[handle];
            if let Some(inter) = collider.shape().cast_ray(collider.position(), ray, max_toi) {
                if inter.toi < best {
                    best = inter.toi;
                    result = Some((handle, collider, inter));
                }
            }
        }

        result
    }

    /// Find the all intersections between a ray and a set of collider and passes them to a callback.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `f32::MAX` for an unbounded ray.
    /// - `callback`: function executed on each collider for which a ray intersection has been found.
    ///   There is no guarantees on the order the results will be yielded. If this callback returns `false`,
    ///   this method will exit early, ignory any further raycast.
    pub fn interferences_with_ray<'a>(
        &self,
        colliders: &'a ColliderSet,
        ray: &Ray,
        max_toi: f32,
        mut callback: impl FnMut(ColliderHandle, &'a Collider, RayIntersection) -> bool,
    ) {
        // TODO: avoid allocation?
        let mut inter = Vec::new();
        self.quadtree.cast_ray(ray, max_toi, &mut inter);

        for handle in inter {
            let collider = &colliders[handle];
            if let Some(inter) = collider.shape().cast_ray(collider.position(), ray, max_toi) {
                if !callback(handle, collider, inter) {
                    return;
                }
            }
        }
    }
}
