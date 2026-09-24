use crate::math::{Real, Vect};
use rapier::parry::shape::Segment;

/// Read-only access to the properties of a segment.
#[derive(Copy, Clone)]
pub struct SegmentView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Segment,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The segment first point.
            pub fn a(&self) -> Vect {
                self.raw.a.into()
            }

            /// The segment second point.
            pub fn b(&self) -> Vect {
                self.raw.b.into()
            }

            /// The direction of this segment scaled by its length.
            ///
            /// Points from `self.a` toward `self.b`.
            pub fn scaled_direction(&self) -> Vect {
                self.raw.scaled_direction().into()
            }

            /// The length of this segment.
            pub fn length(&self) -> Real {
                self.raw.length()
            }

            /// The unit direction of this segment.
            ///
            /// Points from `self.a()` toward `self.b()`.
            /// Returns `None` is both points are equal.
            pub fn direction(&self) -> Option<Vect> {
                self.raw.direction()
            }

            /// In 2D, the not-normalized counterclockwise normal of this segment.
            #[cfg(feature = "dim2")]
            pub fn scaled_normal(&self) -> Vect {
                self.raw.scaled_normal()
            }

            /// The not-normalized counterclockwise normal of this segment, assuming it lies on the plane
            /// with the normal collinear to the given axis (0 = X, 1 = Y, 2 = Z).
            #[cfg(feature = "dim3")]
            pub fn scaled_planar_normal(&self, plane_axis: u8) -> Vect {
                self.raw.scaled_planar_normal(plane_axis)
            }

            /// In 2D, the normalized counterclockwise normal of this segment.
            #[cfg(feature = "dim2")]
            pub fn normal(&self) -> Option<Vect> {
                self.raw.normal()
            }

            /// Returns `None`. Exists only for API similarity with the 2D parry.
            #[cfg(feature = "dim3")]
            pub fn normal(&self) -> Option<Vect> {
                self.raw.normal()
            }

            /// The normalized counterclockwise normal of this segment, assuming it lies on the plane
            /// with the normal collinear to the given axis (0 = X, 1 = Y, 2 = Z).
            #[cfg(feature = "dim3")]
            pub fn planar_normal(&self, plane_axis: u8) -> Option<Vect> {
                self.raw.planar_normal(plane_axis)
            }
        }
    }
);

impl_ref_methods!(SegmentView);

/// Read-write access to the properties of a segment.
pub struct SegmentViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Segment,
}

impl_ref_methods!(SegmentViewMut);

impl SegmentViewMut<'_> {
    /// Set the first point of the segment.
    pub fn set_a(&mut self, a: Vect) {
        self.raw.a = a;
    }

    /// Set the second point of the segment.
    pub fn set_b(&mut self, b: Vect) {
        self.raw.b = b;
    }
}
