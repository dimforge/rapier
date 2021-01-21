use crate::cdl::transformation::vhacd::{VHACDParameters, VHACD};
use crate::math::{Isometry, Point, Real, Vector, DIM};
use cdl::shape::{
    Ball, Capsule, Compound, Cuboid, HalfSpace, HeightField, RoundCuboid, RoundShape,
    RoundTriangle, Segment, Shape, ShapeType, TriMesh, Triangle,
};
#[cfg(feature = "dim3")]
use cdl::shape::{
    Cone, ConvexPolyhedron, Cylinder, RoundCone, RoundConvexPolyhedron, RoundCylinder,
};
#[cfg(feature = "dim2")]
use cdl::shape::{ConvexPolygon, RoundConvexPolygon};
use std::ops::Deref;
use std::sync::Arc;

/// The shape of a collider.
#[derive(Clone)]
pub struct ColliderShape(pub Arc<dyn Shape>);

impl Deref for ColliderShape {
    type Target = dyn Shape;
    fn deref(&self) -> &dyn Shape {
        &*self.0
    }
}

impl ColliderShape {
    /// Initialize a compound shape defined by its subshapes.
    pub fn compound(shapes: Vec<(Isometry<Real>, ColliderShape)>) -> Self {
        let raw_shapes = shapes.into_iter().map(|s| (s.0, s.1 .0)).collect();
        let compound = Compound::new(raw_shapes);
        ColliderShape(Arc::new(compound))
    }

    /// Initialize a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        ColliderShape(Arc::new(Ball::new(radius)))
    }

    /// Initialize a cylindrical shape defined by its half-height
    /// (along along the y axis) and its radius.
    #[cfg(feature = "dim3")]
    pub fn cylinder(half_height: Real, radius: Real) -> Self {
        ColliderShape(Arc::new(Cylinder::new(half_height, radius)))
    }

    /// Initialize a rounded cylindrical shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
        ColliderShape(Arc::new(RoundShape {
            base_shape: Cylinder::new(half_height, radius),
            border_radius,
        }))
    }

    /// Initialize a rounded cone shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
        ColliderShape(Arc::new(RoundShape {
            base_shape: Cone::new(half_height, radius),
            border_radius,
        }))
    }

    /// Initialize a cone shape defined by its half-height
    /// (along along the y axis) and its basis radius.
    #[cfg(feature = "dim3")]
    pub fn cone(half_height: Real, radius: Real) -> Self {
        ColliderShape(Arc::new(Cone::new(half_height, radius)))
    }

    /// Initialize a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: Real, hy: Real) -> Self {
        ColliderShape(Arc::new(Cuboid::new(Vector::new(hx, hy))))
    }

    /// Initialize a round cuboid shape defined by its half-extents and border radius.
    #[cfg(feature = "dim2")]
    pub fn round_cuboid(hx: Real, hy: Real, border_radius: Real) -> Self {
        ColliderShape(Arc::new(RoundShape {
            base_shape: Cuboid::new(Vector::new(hx, hy)),
            border_radius,
        }))
    }

    /// Initialize a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
        ColliderShape(Arc::new(Cuboid::new(Vector::new(hx, hy, hz))))
    }

    /// Initialize a round cuboid shape defined by its half-extents and border radius.
    #[cfg(feature = "dim3")]
    pub fn round_cuboid(hx: Real, hy: Real, hz: Real, border_radius: Real) -> Self {
        ColliderShape(Arc::new(RoundShape {
            base_shape: Cuboid::new(Vector::new(hx, hy, hz)),
            border_radius,
        }))
    }

    /// Initialize a capsule shape from its endpoints and radius.
    pub fn capsule(a: Point<Real>, b: Point<Real>, radius: Real) -> Self {
        ColliderShape(Arc::new(Capsule::new(a, b, radius)))
    }

    /// Initialize a segment shape from its endpoints.
    pub fn segment(a: Point<Real>, b: Point<Real>) -> Self {
        ColliderShape(Arc::new(Segment::new(a, b)))
    }

    /// Initializes a triangle shape.
    pub fn triangle(a: Point<Real>, b: Point<Real>, c: Point<Real>) -> Self {
        ColliderShape(Arc::new(Triangle::new(a, b, c)))
    }

    /// Initializes a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Point<Real>>, indices: Vec<[u32; 3]>) -> Self {
        ColliderShape(Arc::new(TriMesh::new(vertices, indices)))
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D) or
    /// polyline (in 2D) into convex parts.
    pub fn convex_decomposition(vertices: &[Point<Real>], indices: &[[u32; DIM]]) -> Self {
        Self::convex_decomposition_with_params(vertices, indices, &VHACDParameters::default())
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D) or
    /// polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        border_radius: Real,
    ) -> Self {
        Self::round_convex_decomposition_with_params(
            vertices,
            indices,
            &VHACDParameters::default(),
            border_radius,
        )
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D) or
    /// polyline (in 2D) into convex parts.
    pub fn convex_decomposition_with_params(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
    ) -> Self {
        let mut parts = vec![];
        let decomp = VHACD::decompose(params, &vertices, &indices, true);

        for (vertices, indices) in decomp.compute_exact_convex_hulls(&vertices, &indices) {
            if let Some(convex) = Self::convex_mesh(vertices, &indices) {
                parts.push((Isometry::identity(), convex));
            }
        }

        Self::compound(parts)
    }

    /// Initializes a compound shape obtained from the decomposition of the given trimesh (in 3D) or
    /// polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition_with_params(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
        border_radius: Real,
    ) -> Self {
        let mut parts = vec![];
        let decomp = VHACD::decompose(params, &vertices, &indices, true);

        for (vertices, indices) in decomp.compute_exact_convex_hulls(&vertices, &indices) {
            if let Some(convex) = Self::round_convex_mesh(vertices, &indices, border_radius) {
                parts.push((Isometry::identity(), convex));
            }
        }

        Self::compound(parts)
    }

    pub fn convex_hull(points: &[Point<Real>]) -> Option<Self> {
        #[cfg(feature = "dim2")]
        return ConvexPolygon::from_convex_hull(points).map(|ch| ColliderShape(Arc::new(ch)));
        #[cfg(feature = "dim3")]
        return ConvexPolyhedron::from_convex_hull(points).map(|ch| ColliderShape(Arc::new(ch)));
    }

    #[cfg(feature = "dim2")]
    pub fn convex_polyline(points: Vec<Point<Real>>) -> Option<Self> {
        ConvexPolygon::from_convex_polyline(points).map(|ch| ColliderShape(Arc::new(ch)))
    }

    #[cfg(feature = "dim3")]
    pub fn convex_mesh(points: Vec<Point<Real>>, indices: &[[u32; 3]]) -> Option<Self> {
        ConvexPolyhedron::from_convex_mesh(points, indices).map(|ch| ColliderShape(Arc::new(ch)))
    }

    pub fn round_convex_hull(points: &[Point<Real>], border_radius: Real) -> Option<Self> {
        #[cfg(feature = "dim2")]
        return ConvexPolygon::from_convex_hull(points).map(|ch| {
            ColliderShape(Arc::new(RoundShape {
                base_shape: ch,
                border_radius,
            }))
        });
        #[cfg(feature = "dim3")]
        return ConvexPolyhedron::from_convex_hull(points).map(|ch| {
            ColliderShape(Arc::new(RoundShape {
                base_shape: ch,
                border_radius,
            }))
        });
    }

    #[cfg(feature = "dim2")]
    pub fn round_convex_polyline(points: Vec<Point<Real>>, border_radius: Real) -> Option<Self> {
        ConvexPolygon::from_convex_polyline(points).map(|ch| {
            ColliderShape(Arc::new(RoundShape {
                base_shape: ch,
                border_radius,
            }))
        })
    }

    #[cfg(feature = "dim3")]
    pub fn round_convex_mesh(
        points: Vec<Point<Real>>,
        indices: &[[u32; 3]],
        border_radius: Real,
    ) -> Option<Self> {
        ConvexPolyhedron::from_convex_mesh(points, indices).map(|ch| {
            ColliderShape(Arc::new(RoundShape {
                base_shape: ch,
                border_radius,
            }))
        })
    }

    /// Initializes an heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: na::DVector<Real>, scale: Vector<Real>) -> Self {
        ColliderShape(Arc::new(HeightField::new(heights, scale)))
    }

    /// Initializes an heightfield shape on the x-z plane defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: na::DMatrix<Real>, scale: Vector<Real>) -> Self {
        ColliderShape(Arc::new(HeightField::new(heights, scale)))
    }
}

#[cfg(feature = "serde-serialize")]
impl serde::Serialize for ColliderShape {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use crate::serde::ser::SerializeStruct;

        if let Some(ser) = self.0.as_serialize() {
            let typ = self.0.shape_type();
            let mut state = serializer.serialize_struct("ColliderShape", 2)?;
            state.serialize_field("tag", &(typ as i32))?;
            state.serialize_field("inner", ser)?;
            state.end()
        } else {
            Err(serde::ser::Error::custom(
                "Found a non-serializable custom shape.",
            ))
        }
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de> serde::Deserialize<'de> for ColliderShape {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor {};
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = ColliderShape;
            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(formatter, "one shape type tag and the inner shape data")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                use num::cast::FromPrimitive;

                let tag: i32 = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;

                fn deser<'de, A, S: Shape + serde::Deserialize<'de>>(
                    seq: &mut A,
                ) -> Result<Arc<dyn Shape>, A::Error>
                where
                    A: serde::de::SeqAccess<'de>,
                {
                    let shape: S = seq.next_element()?.ok_or_else(|| {
                        serde::de::Error::custom("Failed to deserialize builtin shape.")
                    })?;
                    Ok(Arc::new(shape) as Arc<dyn Shape>)
                }

                let shape = match ShapeType::from_i32(tag) {
                    Some(ShapeType::Ball) => deser::<A, Ball>(&mut seq)?,
                    Some(ShapeType::Cuboid) => deser::<A, Cuboid>(&mut seq)?,
                    Some(ShapeType::Capsule) => deser::<A, Capsule>(&mut seq)?,
                    Some(ShapeType::Triangle) => deser::<A, Triangle>(&mut seq)?,
                    Some(ShapeType::Segment) => deser::<A, Segment>(&mut seq)?,
                    Some(ShapeType::TriMesh) => deser::<A, TriMesh>(&mut seq)?,
                    Some(ShapeType::HeightField) => deser::<A, HeightField>(&mut seq)?,
                    Some(ShapeType::HalfSpace) => deser::<A, HalfSpace>(&mut seq)?,
                    Some(ShapeType::RoundCuboid) => deser::<A, RoundCuboid>(&mut seq)?,
                    Some(ShapeType::RoundTriangle) => deser::<A, RoundTriangle>(&mut seq)?,
                    #[cfg(feature = "dim2")]
                    Some(ShapeType::ConvexPolygon) => deser::<A, ConvexPolygon>(&mut seq)?,
                    #[cfg(feature = "dim2")]
                    Some(ShapeType::RoundConvexPolygon) => {
                        deser::<A, RoundConvexPolygon>(&mut seq)?
                    }
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::Cylinder) => deser::<A, Cylinder>(&mut seq)?,
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::ConvexPolyhedron) => deser::<A, ConvexPolyhedron>(&mut seq)?,
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::Cone) => deser::<A, Cone>(&mut seq)?,
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::RoundCylinder) => deser::<A, RoundCylinder>(&mut seq)?,
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::RoundCone) => deser::<A, RoundCone>(&mut seq)?,
                    #[cfg(feature = "dim3")]
                    Some(ShapeType::RoundConvexPolyhedron) => {
                        deser::<A, RoundConvexPolyhedron>(&mut seq)?
                    }
                    Some(ShapeType::Compound) => {
                        return Err(serde::de::Error::custom(
                            "found invalid shape type to deserialize",
                        ))
                    }
                    None => {
                        return Err(serde::de::Error::custom(
                            "found invalid shape type to deserialize",
                        ))
                    }
                };

                Ok(ColliderShape(shape))
            }
        }

        deserializer.deserialize_struct("ColliderShape", &["tag", "inner"], Visitor {})
    }
}
