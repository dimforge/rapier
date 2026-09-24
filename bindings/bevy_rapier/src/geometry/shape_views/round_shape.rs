use crate::geometry::shape_views::{CuboidView, CuboidViewMut, TriangleView, TriangleViewMut};
use rapier::geometry::{RoundCuboid, RoundTriangle};

#[cfg(feature = "dim2")]
use {
    crate::geometry::shape_views::{ConvexPolygonView, ConvexPolygonViewMut},
    rapier::geometry::RoundConvexPolygon,
};

#[cfg(feature = "dim3")]
use {
    crate::geometry::shape_views::{
        ConeView, ConeViewMut, ConvexPolyhedronView, ConvexPolyhedronViewMut, CylinderView,
        CylinderViewMut,
    },
    rapier::geometry::{RoundCone, RoundConvexPolyhedron, RoundCylinder},
};

macro_rules!  round_shape_view(
    ($RoundShape: ident, $RoundShapeView: ident, $ShapeView: ident, $RoundShapeViewMut: ident, $ShapeViewMut: ident) => {
        /// Read-only access to the properties of a round shape.
        #[derive(Copy, Clone)]
        pub struct $RoundShapeView<'a> {
            /// The raw shape from Rapier.
            pub raw: &'a $RoundShape,
        }

        impl<'a> $RoundShapeView<'a> {
            /// The radius of the round border of this shape.
            pub fn border_radius(&self) -> f32 {
                self.raw.border_radius
            }

            /// The underlying not-rounded shape.
            pub fn inner_shape(&self) -> $ShapeView<'_> {
                $ShapeView {
                    raw: &self.raw.inner_shape,
                }
            }
        }

        /// Read-write access to the properties of a round shape.
        pub struct $RoundShapeViewMut<'a> {
            /// The raw shape from Rapier.
            pub raw: &'a mut $RoundShape,
        }

        impl<'a> $RoundShapeViewMut<'a> {
            /// The radius of the round border of this shape.
            pub fn border_radius(&self) -> f32 {
                self.raw.border_radius
            }

            /// Set the radius of the round border of this shape.
            pub fn set_border_radius(&mut self, new_border_radius: f32) {
                self.raw.border_radius = new_border_radius;
            }

            /// The underlying not-rounded shape.
            pub fn inner_shape(&mut self) -> $ShapeViewMut<'_> {
                $ShapeViewMut {
                    raw: &mut self.raw.inner_shape,
                }
            }
        }
    }
);

round_shape_view!(
    RoundCuboid,
    RoundCuboidView,
    CuboidView,
    RoundCuboidViewMut,
    CuboidViewMut
);

round_shape_view!(
    RoundTriangle,
    RoundTriangleView,
    TriangleView,
    RoundTriangleViewMut,
    TriangleViewMut
);

#[cfg(feature = "dim2")]
round_shape_view!(
    RoundConvexPolygon,
    RoundConvexPolygonView,
    ConvexPolygonView,
    RoundConvexPolygonViewMut,
    ConvexPolygonViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundCone,
    RoundConeView,
    ConeView,
    RoundConeViewMut,
    ConeViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundCylinder,
    RoundCylinderView,
    CylinderView,
    RoundCylinderViewMut,
    CylinderViewMut
);

#[cfg(feature = "dim3")]
round_shape_view!(
    RoundConvexPolyhedron,
    RoundConvexPolyhedronView,
    ConvexPolyhedronView,
    RoundConvexPolyhedronViewMut,
    ConvexPolyhedronViewMut
);
