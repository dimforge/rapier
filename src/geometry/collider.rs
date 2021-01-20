use crate::dynamics::{MassProperties, RigidBodyHandle};
use crate::geometry::InteractionGroups;
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector};
use cdl::bounding_volume::AABB;
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

// TODO: move this to its own file.
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

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
///
/// To build a new collider, use the `ColliderBuilder` structure.
pub struct Collider {
    shape: ColliderShape,
    density: Real,
    is_sensor: bool,
    pub(crate) parent: RigidBodyHandle,
    pub(crate) delta: Isometry<Real>,
    pub(crate) position: Isometry<Real>,
    pub(crate) predicted_position: Isometry<Real>,
    /// The friction coefficient of this collider.
    pub friction: Real,
    /// The restitution coefficient of this collider.
    pub restitution: Real,
    pub(crate) collision_groups: InteractionGroups,
    pub(crate) solver_groups: InteractionGroups,
    pub(crate) proxy_index: usize,
    /// User-defined data associated to this rigid-body.
    pub user_data: u128,
}

impl Collider {
    pub(crate) fn reset_internal_references(&mut self) {
        self.parent = RigidBodyHandle::invalid();
        self.proxy_index = crate::INVALID_USIZE;
    }

    /// The rigid body this collider is attached to.
    pub fn parent(&self) -> RigidBodyHandle {
        self.parent
    }

    /// Is this collider a sensor?
    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    #[doc(hidden)]
    pub fn set_position_debug(&mut self, position: Isometry<Real>) {
        self.position = position;
    }

    /// The position of this collider expressed in the local-space of the rigid-body it is attached to.
    #[deprecated(note = "use `.position_wrt_parent()` instead.")]
    pub fn delta(&self) -> &Isometry<Real> {
        &self.delta
    }

    /// The world-space position of this collider.
    pub fn position(&self) -> &Isometry<Real> {
        &self.position
    }

    /// The position of this collider wrt the body it is attached to.
    pub fn position_wrt_parent(&self) -> &Isometry<Real> {
        &self.delta
    }

    /// The collision groups used by this collider.
    pub fn collision_groups(&self) -> InteractionGroups {
        self.collision_groups
    }

    /// The solver groups used by this collider.
    pub fn solver_groups(&self) -> InteractionGroups {
        self.solver_groups
    }

    /// The density of this collider.
    pub fn density(&self) -> Real {
        self.density
    }

    /// The geometric shape of this collider.
    pub fn shape(&self) -> &dyn Shape {
        &*self.shape.0
    }

    /// Compute the axis-aligned bounding box of this collider.
    pub fn compute_aabb(&self) -> AABB {
        self.shape.compute_aabb(&self.position)
    }

    // pub(crate) fn compute_aabb_with_prediction(&self) -> AABB {
    //     let aabb1 = self.shape.compute_aabb(&self.position);
    //     let aabb2 = self.shape.compute_aabb(&self.predicted_position);
    //     aabb1.merged(&aabb2)
    // }

    /// Compute the local-space mass properties of this collider.
    pub fn mass_properties(&self) -> MassProperties {
        self.shape.mass_properties(self.density)
    }
}

/// A structure responsible for building a new collider.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderBuilder {
    /// The shape of the collider to be built.
    pub shape: ColliderShape,
    /// The density of the collider to be built.
    density: Option<Real>,
    /// The friction coefficient of the collider to be built.
    pub friction: Real,
    /// The restitution coefficient of the collider to be built.
    pub restitution: Real,
    /// The position of this collider relative to the local frame of the rigid-body it is attached to.
    pub delta: Isometry<Real>,
    /// Is this collider a sensor?
    pub is_sensor: bool,
    /// The user-data of the collider being built.
    pub user_data: u128,
    /// The collision groups for the collider being built.
    pub collision_groups: InteractionGroups,
    /// The solver groups for the collider being built.
    pub solver_groups: InteractionGroups,
}

impl ColliderBuilder {
    /// Initialize a new collider builder with the given shape.
    pub fn new(shape: ColliderShape) -> Self {
        Self {
            shape,
            density: None,
            friction: Self::default_friction(),
            restitution: 0.0,
            delta: Isometry::identity(),
            is_sensor: false,
            user_data: 0,
            collision_groups: InteractionGroups::all(),
            solver_groups: InteractionGroups::all(),
        }
    }

    /// The density of the collider being built.
    pub fn get_density(&self) -> Real {
        let default_density = if self.is_sensor { 0.0 } else { 1.0 };
        self.density.unwrap_or(default_density)
    }

    pub fn compound(shapes: Vec<(Isometry<Real>, ColliderShape)>) -> Self {
        Self::new(ColliderShape::compound(shapes))
    }

    /// Initialize a new collider builder with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        Self::new(ColliderShape::ball(radius))
    }

    /// Initialize a new collider builder with a cylindrical shape defined by its half-height
    /// (along along the y axis) and its radius.
    #[cfg(feature = "dim3")]
    pub fn cylinder(half_height: Real, radius: Real) -> Self {
        Self::new(ColliderShape::cylinder(half_height, radius))
    }

    /// Initialize a new collider builder with a rounded cylindrical shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
        Self::new(ColliderShape::round_cylinder(
            half_height,
            radius,
            border_radius,
        ))
    }

    /// Initialize a new collider builder with a cone shape defined by its half-height
    /// (along along the y axis) and its basis radius.
    #[cfg(feature = "dim3")]
    pub fn cone(half_height: Real, radius: Real) -> Self {
        Self::new(ColliderShape::cone(half_height, radius))
    }

    /// Initialize a new collider builder with a rounded cone shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
        Self::new(ColliderShape::round_cone(
            half_height,
            radius,
            border_radius,
        ))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: Real, hy: Real) -> Self {
        Self::new(ColliderShape::cuboid(hx, hy))
    }

    /// Initialize a new collider builder with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim2")]
    pub fn round_cuboid(hx: Real, hy: Real, border_radius: Real) -> Self {
        Self::new(ColliderShape::round_cuboid(hx, hy, border_radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::x() * half_height);
        Self::new(ColliderShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::y() * half_height);
        Self::new(ColliderShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::z() * half_height);
        Self::new(ColliderShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
        Self::new(ColliderShape::cuboid(hx, hy, hz))
    }

    /// Initialize a new collider builder with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim3")]
    pub fn round_cuboid(hx: Real, hy: Real, hz: Real, border_radius: Real) -> Self {
        Self::new(ColliderShape::round_cuboid(hx, hy, hz, border_radius))
    }

    /// Initializes a collider builder with a segment shape.
    pub fn segment(a: Point<Real>, b: Point<Real>) -> Self {
        Self::new(ColliderShape::segment(a, b))
    }

    /// Initializes a collider builder with a triangle shape.
    pub fn triangle(a: Point<Real>, b: Point<Real>, c: Point<Real>) -> Self {
        Self::new(ColliderShape::triangle(a, b, c))
    }

    /// Initializes a collider builder with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Point<Real>>, indices: Vec<[u32; 3]>) -> Self {
        Self::new(ColliderShape::trimesh(vertices, indices))
    }

    pub fn convex_hull(points: &[Point<Real>]) -> Option<Self> {
        ColliderShape::convex_hull(points).map(|cp| Self::new(cp))
    }

    pub fn round_convex_hull(points: &[Point<Real>], border_radius: Real) -> Option<Self> {
        ColliderShape::round_convex_hull(points, border_radius).map(|cp| Self::new(cp))
    }

    #[cfg(feature = "dim2")]
    pub fn convex_polyline(points: Vec<Point<Real>>) -> Option<Self> {
        ColliderShape::convex_polyline(points).map(|cp| Self::new(cp))
    }

    #[cfg(feature = "dim2")]
    pub fn round_convex_polyline(points: Vec<Point<Real>>, border_radius: Real) -> Option<Self> {
        ColliderShape::round_convex_polyline(points, border_radius).map(|cp| Self::new(cp))
    }

    #[cfg(feature = "dim3")]
    pub fn convex_mesh(points: Vec<Point<Real>>, indices: &[[u32; 3]]) -> Option<Self> {
        ColliderShape::convex_mesh(points, indices).map(|cp| Self::new(cp))
    }

    #[cfg(feature = "dim3")]
    pub fn round_convex_mesh(
        points: Vec<Point<Real>>,
        indices: &[[u32; 3]],
        border_radius: Real,
    ) -> Option<Self> {
        ColliderShape::round_convex_mesh(points, indices, border_radius).map(|cp| Self::new(cp))
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: na::DVector<Real>, scale: Vector<Real>) -> Self {
        Self::new(ColliderShape::heightfield(heights, scale))
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: na::DMatrix<Real>, scale: Vector<Real>) -> Self {
        Self::new(ColliderShape::heightfield(heights, scale))
    }

    /// The default friction coefficient used by the collider builder.
    pub fn default_friction() -> Real {
        0.5
    }

    /// Sets an arbitrary user-defined 128-bit integer associated to the colliders built by this builder.
    pub fn user_data(mut self, data: u128) -> Self {
        self.user_data = data;
        self
    }

    /// Sets the collision groups used by this collider.
    ///
    /// Two colliders will interact iff. their collision groups are compatible.
    /// See [InteractionGroups::test] for details.
    pub fn collision_groups(mut self, groups: InteractionGroups) -> Self {
        self.collision_groups = groups;
        self
    }

    /// Sets the solver groups used by this collider.
    ///
    /// Forces between two colliders in contact will be computed iff their solver groups are
    /// compatible. See [InteractionGroups::test] for details.
    pub fn solver_groups(mut self, groups: InteractionGroups) -> Self {
        self.solver_groups = groups;
        self
    }

    /// Sets whether or not the collider built by this builder is a sensor.
    pub fn sensor(mut self, is_sensor: bool) -> Self {
        self.is_sensor = is_sensor;
        self
    }

    /// Sets the friction coefficient of the collider this builder will build.
    pub fn friction(mut self, friction: Real) -> Self {
        self.friction = friction;
        self
    }

    /// Sets the restitution coefficient of the collider this builder will build.
    pub fn restitution(mut self, restitution: Real) -> Self {
        self.restitution = restitution;
        self
    }

    /// Sets the density of the collider this builder will build.
    pub fn density(mut self, density: Real) -> Self {
        self.density = Some(density);
        self
    }

    /// Sets the initial translation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    #[cfg(feature = "dim2")]
    pub fn translation(mut self, x: Real, y: Real) -> Self {
        self.delta.translation.x = x;
        self.delta.translation.y = y;
        self
    }

    /// Sets the initial translation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    #[cfg(feature = "dim3")]
    pub fn translation(mut self, x: Real, y: Real, z: Real) -> Self {
        self.delta.translation.x = x;
        self.delta.translation.y = y;
        self.delta.translation.z = z;
        self
    }

    /// Sets the initial orientation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    pub fn rotation(mut self, angle: AngVector<Real>) -> Self {
        self.delta.rotation = Rotation::new(angle);
        self
    }

    /// Sets the initial position (translation and orientation) of the collider to be created,
    /// relative to the rigid-body it is attached to.
    pub fn position(mut self, pos: Isometry<Real>) -> Self {
        self.delta = pos;
        self
    }

    /// Set the position of this collider in the local-space of the rigid-body it is attached to.
    #[deprecated(note = "Use `.position` instead.")]
    pub fn delta(mut self, delta: Isometry<Real>) -> Self {
        self.delta = delta;
        self
    }

    /// Builds a new collider attached to the given rigid-body.
    pub fn build(&self) -> Collider {
        let density = self.get_density();

        Collider {
            shape: self.shape.clone(),
            density,
            friction: self.friction,
            restitution: self.restitution,
            delta: self.delta,
            is_sensor: self.is_sensor,
            parent: RigidBodyHandle::invalid(),
            position: Isometry::identity(),
            predicted_position: Isometry::identity(),
            proxy_index: crate::INVALID_USIZE,
            collision_groups: self.collision_groups,
            solver_groups: self.solver_groups,
            user_data: self.user_data,
        }
    }
}
