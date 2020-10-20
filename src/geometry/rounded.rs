#[cfg(feature = "dim3")]
use crate::geometry::Cylinder;
use crate::geometry::ShapeType;
use crate::math::{Isometry, Point, Vector};
use na::Unit;
use ncollide::query::{
    algorithms::VoronoiSimplex, PointProjection, PointQuery, Ray, RayCast, RayIntersection,
};
use ncollide::shape::{FeatureId, SupportMap};

/// A shape which corner can be rounded.
pub trait Roundable {
    /// The ShapeType fo this shape after rounding its corners.
    fn rounded_shape_type() -> ShapeType;
}

#[cfg(feature = "dim3")]
impl Roundable for Cylinder {
    fn rounded_shape_type() -> ShapeType {
        ShapeType::RoundedCylinder
    }
}

/// A rounded shape.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Rounded<S: Roundable> {
    /// The shape being rounded.
    pub shape: S,
    /// The rounding radius.
    pub radius: f32,
}

impl<S: Roundable> Rounded<S> {
    /// Create sa new shape where all its edges and vertices are rounded by a radius of `radius`.
    ///
    /// This is done by applying a dilation of the given radius to the shape.
    pub fn new(shape: S, radius: f32) -> Self {
        Self { shape, radius }
    }
}

impl<S: Roundable + SupportMap<f32>> SupportMap<f32> for Rounded<S> {
    fn local_support_point(&self, dir: &Vector<f32>) -> Point<f32> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    fn local_support_point_toward(&self, dir: &Unit<Vector<f32>>) -> Point<f32> {
        self.shape.local_support_point_toward(dir) + **dir * self.radius
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

impl<S: Roundable + SupportMap<f32>> RayCast<f32> for Rounded<S> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<f32>,
        ray: &Ray<f32>,
        max_toi: f32,
        solid: bool,
    ) -> Option<RayIntersection<f32>> {
        let ls_ray = ray.inverse_transform_by(m);

        ncollide::query::ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

// TODO: if PointQuery had a `project_point_with_normal` method, we could just
// call this and adjust the projected point accordingly.
impl<S: Roundable + SupportMap<f32>> PointQuery<f32> for Rounded<S> {
    #[inline]
    fn project_point(
        &self,
        m: &Isometry<f32>,
        point: &Point<f32>,
        solid: bool,
    ) -> PointProjection<f32> {
        ncollide::query::point_projection_on_support_map(
            m,
            self,
            &mut VoronoiSimplex::new(),
            point,
            solid,
        )
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<f32>,
        point: &Point<f32>,
    ) -> (PointProjection<f32>, FeatureId) {
        (self.project_point(m, point, false), FeatureId::Unknown)
    }
}
