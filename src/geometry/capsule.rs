use crate::geometry::AABB;
use crate::math::{Isometry, Point, Rotation, Vector};
use approx::AbsDiffEq;
use na::Unit;
use ncollide::query::{PointProjection, PointQuery};
use ncollide::shape::{FeatureId, Segment};

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A capsule shape defined as a segment with a radius.
pub struct Capsule {
    /// The first endpoint of the capsule.
    pub a: Point<f32>,
    /// The second enpdoint of the capsule.
    pub b: Point<f32>,
    /// The radius of the capsule.
    pub radius: f32,
}

impl Capsule {
    /// Creates a new capsule aligned with the `x` axis and with the given half-height an radius.
    pub fn new_x(half_height: f32, radius: f32) -> Self {
        let b = Point::from(Vector::x() * half_height);
        Self::new(-b, b, radius)
    }

    /// Creates a new capsule aligned with the `y` axis and with the given half-height an radius.
    pub fn new_y(half_height: f32, radius: f32) -> Self {
        let b = Point::from(Vector::y() * half_height);
        Self::new(-b, b, radius)
    }

    /// Creates a new capsule aligned with the `z` axis and with the given half-height an radius.
    #[cfg(feature = "dim3")]
    pub fn new_z(half_height: f32, radius: f32) -> Self {
        let b = Point::from(Vector::z() * half_height);
        Self::new(-b, b, radius)
    }

    /// Creates a new capsule defined as the segment between `a` and `b` and with the given `radius`.
    pub fn new(a: Point<f32>, b: Point<f32>, radius: f32) -> Self {
        Self { a, b, radius }
    }

    /// The axis-aligned bounding box of this capsule.
    pub fn aabb(&self, pos: &Isometry<f32>) -> AABB {
        let a = pos * self.a;
        let b = pos * self.b;
        let mins = a.coords.inf(&b.coords) - Vector::repeat(self.radius);
        let maxs = a.coords.sup(&b.coords) + Vector::repeat(self.radius);
        AABB::new(mins.into(), maxs.into())
    }

    /// The height of this capsule.
    pub fn height(&self) -> f32 {
        (self.b - self.a).norm()
    }

    /// The half-height of this capsule.
    pub fn half_height(&self) -> f32 {
        self.height() / 2.0
    }

    /// The center of this capsule.
    pub fn center(&self) -> Point<f32> {
        na::center(&self.a, &self.b)
    }

    /// Creates a new capsule equal to `self` with all its endpoints transformed by `pos`.
    pub fn transform_by(&self, pos: &Isometry<f32>) -> Self {
        Self::new(pos * self.a, pos * self.b, self.radius)
    }

    /// The rotation `r` such that `r * Y` is collinear with `b - a`.
    pub fn rotation_wrt_y(&self) -> Rotation<f32> {
        let mut dir = self.b - self.a;
        if dir.y < 0.0 {
            dir = -dir;
        }

        #[cfg(feature = "dim2")]
        {
            Rotation::rotation_between(&Vector::y(), &dir)
        }

        #[cfg(feature = "dim3")]
        {
            Rotation::rotation_between(&Vector::y(), &dir).unwrap_or(Rotation::identity())
        }
    }

    /// The transform `t` such that `t * Y` is collinear with `b - a` and such that `t * origin = (b + a) / 2.0`.
    pub fn transform_wrt_y(&self) -> Isometry<f32> {
        let rot = self.rotation_wrt_y();
        Isometry::from_parts(self.center().coords.into(), rot)
    }
}

// impl SupportMap<f32> for Capsule {
//     fn local_support_point(&self, dir: &Vector) -> Point {
//         let dir = Unit::try_new(dir, 0.0).unwrap_or(Vector::y_axis());
//         self.local_support_point_toward(&dir)
//     }
//
//     fn local_support_point_toward(&self, dir: &Unit<Vector>) -> Point {
//         if dir.dot(&self.a.coords) > dir.dot(&self.b.coords) {
//             self.a + **dir * self.radius
//         } else {
//             self.b + **dir * self.radius
//         }
//     }
// }

// TODO: this code has been extracted from ncollide and added here
// so we can modify it to fit with our new definition of capsule.
// Wa should find a way to avoid this code duplication.
impl PointQuery<f32> for Capsule {
    #[inline]
    fn project_point(
        &self,
        m: &Isometry<f32>,
        pt: &Point<f32>,
        solid: bool,
    ) -> PointProjection<f32> {
        let seg = Segment::new(self.a, self.b);
        let proj = seg.project_point(m, pt, solid);
        let dproj = *pt - proj.point;

        if let Some((dir, dist)) = Unit::try_new_and_get(dproj, f32::default_epsilon()) {
            let inside = dist <= self.radius;
            if solid && inside {
                return PointProjection::new(true, *pt);
            } else {
                return PointProjection::new(inside, proj.point + dir.into_inner() * self.radius);
            }
        } else if solid {
            return PointProjection::new(true, *pt);
        }

        #[cfg(feature = "dim2")]
        if let Some(dir) = seg.normal() {
            let dir = m * *dir;
            PointProjection::new(true, proj.point + dir * self.radius)
        } else {
            // The segment has no normal, likely because it degenerates to a point.
            PointProjection::new(true, proj.point + Vector::ith(1, self.radius))
        }

        #[cfg(feature = "dim3")]
        if let Some(dir) = seg.direction() {
            use crate::utils::WBasis;
            let dir = m * dir.orthonormal_basis()[0];
            PointProjection::new(true, proj.point + dir * self.radius)
        } else {
            // The segment has no normal, likely because it degenerates to a point.
            PointProjection::new(true, proj.point + Vector::ith(1, self.radius))
        }
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<f32>,
        pt: &Point<f32>,
    ) -> (PointProjection<f32>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }
}
