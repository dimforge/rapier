use crate::dynamics::RigidBodySet;
use crate::geometry::{
    Collider, ColliderHandle, ColliderSet, Ray, RayIntersection, WQuadtree, AABB, WAABB,
};
use crate::math::{Point, Vector};
use ncollide::bounding_volume::BoundingVolume;

/// A pipeline for performing queries on all the colliders of a scene.
pub struct QueryPipeline {
    // hierarchy: WAABBHierarchy,
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
            // hierarchy: WAABBHierarchy::new(),
        }
    }

    /// Update the acceleration structure on the query pipeline.
    pub fn update(&mut self, _bodies: &mut RigidBodySet, _colliders: &mut ColliderSet) {}

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
        let t0 = instant::now();
        let mut tree = WQuadtree::new();
        tree.clear_and_rebuild(colliders);
        println!("Built quadtree in time: {}", instant::now() - t0);
        let t0 = instant::now();
        let inter = tree.cast_ray(ray, max_toi);
        println!(
            "Found {} interefrences in time {}.",
            inter.len(),
            instant::now() - t0
        );

        let t0 = instant::now();
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
        println!("Cast time: {}", instant::now() - t0);

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
        // FIXME: this is a brute-force approach.
        for (handle, collider) in colliders.iter() {
            if let Some(inter) = collider.shape().cast_ray(collider.position(), ray, max_toi) {
                if !callback(handle, collider, inter) {
                    return;
                }
            }
        }
    }
}
