use crate::math::{Real, Vect};
use rapier::parry::shape::Triangle;

/// Read-only access to the properties of a triangle.
#[derive(Copy, Clone)]
pub struct TriangleView<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a Triangle,
}

macro_rules! impl_ref_methods(
    ($View: ident) => {
        impl<'a> $View<'a> {
            /// The triangle first point.
            pub fn a(&self) -> Vect {
                self.raw.a.into()
            }

            /// The triangle second point.
            pub fn b(&self) -> Vect {
                self.raw.b.into()
            }

            /// The triangle third point.
            pub fn c(&self) -> Vect {
                self.raw.c.into()
            }

            /// Reference to an array containing the three vertices of this triangle.
            #[inline]
            pub fn vertices(&self) -> [Vect; 3] {
                let vtx = self.raw.vertices();
                [vtx[0].into(), vtx[1].into(), vtx[2].into()]
            }

            /// The normal of this triangle assuming it is oriented ccw.
            ///
            /// The normal points such that it is collinear to `AB × AC` (where `×` denotes the cross
            /// product).
            #[cfg(feature = "dim3")]
            #[inline]
            pub fn normal(&self) -> Option<Vect> {
                self.raw.normal()
            }

            /// A vector normal of this triangle.
            ///
            /// The vector points such that it is collinear to `AB × AC` (where `×` denotes the cross
            /// product).
            #[cfg(feature = "dim3")]
            #[inline]
            pub fn scaled_normal(&self) -> Vect {
                self.raw.scaled_normal().into()
            }

            /// The area of this triangle.
            #[inline]
            pub fn area(&self) -> Real {
                self.raw.area()
            }

            /// The geometric center of this triangle.
            #[inline]
            pub fn center(&self) -> Vect {
                self.raw.center().into()
            }

            /// The perimeter of this triangle.
            #[inline]
            pub fn perimeter(&self) -> Real {
                self.raw.perimeter()
            }

            /// The circumcircle of this triangle.
            pub fn circumcircle(&self) -> (Vect, Real) {
                let (center, radius) = self.raw.circumcircle();
                (center.into(), radius)
            }
        }
    }
);

impl_ref_methods!(TriangleView);

/// Read-write access to the properties of a triangle.
pub struct TriangleViewMut<'a> {
    /// The raw shape from Rapier.
    pub raw: &'a mut Triangle,
}

impl_ref_methods!(TriangleViewMut);

impl TriangleViewMut<'_> {
    /// Set the first point of the segment.
    pub fn set_a(&mut self, a: Vect) {
        self.raw.a = a;
    }

    /// Set the second point of the segment.
    pub fn set_b(&mut self, b: Vect) {
        self.raw.b = b;
    }

    /// Set the third point of the segment.
    pub fn set_c(&mut self, c: Vect) {
        self.raw.c = c;
    }
}
