use crate::geometry::Cylinder;
use crate::math::{Isometry, Point, Vector};
use buckler::query::{
    gjk::VoronoiSimplex, PointProjection, PointQuery, Ray, RayCast, RayIntersection,
};
use buckler::shape::{FeatureId, SupportMap};
use na::Unit;

/// A rounded cylinder.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub struct RoundCylinder {
    /// The cylinder being rounded.
    pub cylinder: Cylinder,
    /// The rounding radius.
    pub border_radius: f32,
}

impl RoundCylinder {
    /// Create sa new cylinder where all its edges and vertices are rounded by a radius of `radius`.
    ///
    /// This is done by applying a dilation of the given radius to the cylinder.
    pub fn new(half_height: f32, radius: f32, border_radius: f32) -> Self {
        Self {
            cylinder: Cylinder::new(half_height, radius),
            border_radius,
        }
    }
}

impl SupportMap for RoundCylinder {
    fn local_support_point(&self, dir: &Vector<f32>) -> Point<f32> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    fn local_support_point_toward(&self, dir: &Unit<Vector<f32>>) -> Point<f32> {
        self.cylinder.local_support_point_toward(dir) + **dir * self.border_radius
    }

    fn support_point(&self, transform: &Isometry<f32>, dir: &Vector<f32>) -> Point<f32> {
        let local_dir = transform.inverse_transform_vector(dir);
        transform * self.local_support_point(&local_dir)
    }

    fn support_point_toward(
        &self,
        transform: &Isometry<f32>,
        dir: &Unit<Vector<f32>>,
    ) -> Point<f32> {
        let local_dir = Unit::new_unchecked(transform.inverse_transform_vector(dir));
        transform * self.local_support_point_toward(&local_dir)
    }
}

impl RayCast for RoundCylinder {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_toi: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        buckler::query::details::local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

// TODO: if PointQuery had a `project_point_with_normal` method, we could just
// call this and adjust the projected point accordingly.
impl PointQuery for RoundCylinder {
    #[inline]
    fn project_local_point(&self, point: &Point<f32>, solid: bool) -> PointProjection {
        buckler::query::details::local_point_projection_on_support_map(
            self,
            &mut VoronoiSimplex::new(),
            point,
            solid,
        )
    }

    #[inline]
    fn project_local_point_and_get_feature(
        &self,
        point: &Point<f32>,
    ) -> (PointProjection, FeatureId) {
        (self.project_local_point(point, false), FeatureId::Unknown)
    }
}
