use crate::dynamics::{MassProperties, RigidBodyHandle, RigidBodySet};
use crate::geometry::{
    Ball, Capsule, ColliderGraphIndex, Contact, Cuboid, HeightField, InteractionGraph, Polygon,
    Proximity, Ray, RayIntersection, Triangle, Trimesh,
};
use crate::math::{AngVector, Isometry, Point, Rotation, Vector};
use na::Point3;
use ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ncollide::query::RayCast;
use num::Zero;

#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// An enum grouping all the possible shape of a collider.
pub enum Shape {
    /// A ball shape.
    Ball(Ball),
    /// A convex polygon shape.
    Polygon(Polygon),
    /// A cuboid shape.
    Cuboid(Cuboid),
    /// A capsule shape.
    Capsule(Capsule),
    /// A triangle shape.
    Triangle(Triangle),
    /// A triangle mesh shape.
    Trimesh(Trimesh),
    /// A heightfield shape.
    HeightField(HeightField),
}

impl Shape {
    /// Gets a reference to the underlying ball shape, if `self` is one.
    pub fn as_ball(&self) -> Option<&Ball> {
        match self {
            Shape::Ball(b) => Some(b),
            _ => None,
        }
    }

    /// Gets a reference to the underlying polygon shape, if `self` is one.
    pub fn as_polygon(&self) -> Option<&Polygon> {
        match self {
            Shape::Polygon(p) => Some(p),
            _ => None,
        }
    }

    /// Gets a reference to the underlying cuboid shape, if `self` is one.
    pub fn as_cuboid(&self) -> Option<&Cuboid> {
        match self {
            Shape::Cuboid(c) => Some(c),
            _ => None,
        }
    }

    /// Gets a reference to the underlying capsule shape, if `self` is one.
    pub fn as_capsule(&self) -> Option<&Capsule> {
        match self {
            Shape::Capsule(c) => Some(c),
            _ => None,
        }
    }

    /// Gets a reference to the underlying triangle mesh shape, if `self` is one.
    pub fn as_trimesh(&self) -> Option<&Trimesh> {
        match self {
            Shape::Trimesh(c) => Some(c),
            _ => None,
        }
    }

    /// Gets a reference to the underlying heightfield shape, if `self` is one.
    pub fn as_heightfield(&self) -> Option<&HeightField> {
        match self {
            Shape::HeightField(h) => Some(h),
            _ => None,
        }
    }

    /// Gets a reference to the underlying triangle shape, if `self` is one.
    pub fn as_triangle(&self) -> Option<&Triangle> {
        match self {
            Shape::Triangle(c) => Some(c),
            _ => None,
        }
    }

    /// Computes the axis-aligned bounding box of this shape.
    pub fn compute_aabb(&self, position: &Isometry<f32>) -> AABB<f32> {
        match self {
            Shape::Ball(ball) => ball.bounding_volume(position),
            Shape::Polygon(poly) => poly.aabb(position),
            Shape::Capsule(caps) => caps.aabb(position),
            Shape::Cuboid(cuboid) => cuboid.bounding_volume(position),
            Shape::Triangle(triangle) => triangle.bounding_volume(position),
            Shape::Trimesh(trimesh) => trimesh.aabb(position),
            Shape::HeightField(heightfield) => heightfield.bounding_volume(position),
        }
    }

    /// Computes the first intersection point between a ray in this collider.
    ///
    /// Some shapes are not supported yet and will always return `None`.
    ///
    /// # Parameters
    /// - `position`: the position of this shape.
    /// - `ray`: the ray to cast.
    /// - `max_toi`: the maximum time-of-impact that can be reported by this cast. This effectively
    ///   limits the length of the ray to `ray.dir.norm() * max_toi`. Use `f32::MAX` for an unbounded ray.
    pub fn cast_ray(
        &self,
        position: &Isometry<f32>,
        ray: &Ray,
        max_toi: f32,
    ) -> Option<RayIntersection> {
        match self {
            Shape::Ball(ball) => ball.toi_and_normal_with_ray(position, ray, max_toi, true),
            Shape::Polygon(_poly) => None,
            Shape::Capsule(caps) => {
                let pos = position * caps.transform_wrt_y();
                let caps = ncollide::shape::Capsule::new(caps.half_height(), caps.radius);
                caps.toi_and_normal_with_ray(&pos, ray, max_toi, true)
            }
            Shape::Cuboid(cuboid) => cuboid.toi_and_normal_with_ray(position, ray, max_toi, true),
            #[cfg(feature = "dim2")]
            Shape::Triangle(triangle) => {
                // This is not implemented yet in 2D.
                None
            }
            #[cfg(feature = "dim3")]
            Shape::Triangle(triangle) => {
                triangle.toi_and_normal_with_ray(position, ray, max_toi, true)
            }
            Shape::Trimesh(_trimesh) => None,
            Shape::HeightField(heightfield) => {
                heightfield.toi_and_normal_with_ray(position, ray, max_toi, true)
            }
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
///
/// To build a new collider, use the `ColliderBuilder` structure.
pub struct Collider {
    shape: Shape,
    density: f32,
    is_sensor: bool,
    pub(crate) parent: RigidBodyHandle,
    pub(crate) delta: Isometry<f32>,
    pub(crate) position: Isometry<f32>,
    pub(crate) predicted_position: Isometry<f32>,
    /// The friction coefficient of this collider.
    pub friction: f32,
    /// The restitution coefficient of this collider.
    pub restitution: f32,
    pub(crate) contact_graph_index: ColliderGraphIndex,
    pub(crate) proximity_graph_index: ColliderGraphIndex,
    pub(crate) proxy_index: usize,
}

impl Clone for Collider {
    fn clone(&self) -> Self {
        Self {
            shape: self.shape.clone(),
            parent: RigidBodySet::invalid_handle(),
            contact_graph_index: ColliderGraphIndex::new(crate::INVALID_U32),
            proximity_graph_index: ColliderGraphIndex::new(crate::INVALID_U32),
            proxy_index: crate::INVALID_USIZE,
            ..*self
        }
    }
}

impl Collider {
    /// The rigid body this collider is attached to.
    pub fn parent(&self) -> RigidBodyHandle {
        self.parent
    }

    /// Is this collider a sensor?
    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    #[doc(hidden)]
    pub fn set_position_debug(&mut self, position: Isometry<f32>) {
        self.position = position;
    }

    /// The position of this collider expressed in the local-space of the rigid-body it is attached to.
    #[deprecated(note = "use `.position_wrt_parent()` instead.")]
    pub fn delta(&self) -> &Isometry<f32> {
        &self.delta
    }

    /// The world-space position of this collider.
    pub fn position(&self) -> &Isometry<f32> {
        &self.position
    }

    /// The position of this collider wrt the body it is attached to.
    pub fn position_wrt_parent(&self) -> &Isometry<f32> {
        &self.delta
    }

    /// The density of this collider.
    pub fn density(&self) -> f32 {
        self.density
    }

    /// The geometric shape of this collider.
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    /// Compute the axis-aligned bounding box of this collider.
    pub fn compute_aabb(&self) -> AABB<f32> {
        self.shape.compute_aabb(&self.position)
    }

    // pub(crate) fn compute_aabb_with_prediction(&self) -> AABB<f32> {
    //     let aabb1 = self.shape.compute_aabb(&self.position);
    //     let aabb2 = self.shape.compute_aabb(&self.predicted_position);
    //     aabb1.merged(&aabb2)
    // }

    /// Compute the local-space mass properties of this collider.
    pub fn mass_properties(&self) -> MassProperties {
        match &self.shape {
            Shape::Ball(ball) => MassProperties::from_ball(self.density, ball.radius),
            #[cfg(feature = "dim2")]
            Shape::Polygon(p) => MassProperties::from_polygon(self.density, p.vertices()),
            #[cfg(feature = "dim3")]
            Shape::Polygon(_p) => unimplemented!(),
            Shape::Cuboid(c) => MassProperties::from_cuboid(self.density, c.half_extents),
            Shape::Capsule(caps) => {
                MassProperties::from_capsule(self.density, caps.a, caps.b, caps.radius)
            }
            Shape::Triangle(_) => MassProperties::zero(),
            Shape::Trimesh(_) => MassProperties::zero(),
            Shape::HeightField(_) => MassProperties::zero(),
        }
    }
}

/// A structure responsible for building a new collider.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderBuilder {
    /// The shape of the collider to be built.
    pub shape: Shape,
    /// The density of the collider to be built.
    density: Option<f32>,
    /// The friction coefficient of the collider to be built.
    pub friction: f32,
    /// The restitution coefficient of the collider to be built.
    pub restitution: f32,
    /// The position of this collider relative to the local frame of the rigid-body it is attached to.
    pub delta: Isometry<f32>,
    /// Is this collider a sensor?
    pub is_sensor: bool,
}

impl ColliderBuilder {
    /// Initialize a new collider builder with the given shape.
    pub fn new(shape: Shape) -> Self {
        Self {
            shape,
            density: None,
            friction: Self::default_friction(),
            restitution: 0.0,
            delta: Isometry::identity(),
            is_sensor: false,
        }
    }

    /// The density of the collider being built.
    pub fn get_density(&self) -> f32 {
        let default_density = if self.is_sensor { 0.0 } else { 1.0 };
        self.density.unwrap_or(default_density)
    }

    /// Initialize a new collider builder with a ball shape defined by its radius.
    pub fn ball(radius: f32) -> Self {
        Self::new(Shape::Ball(Ball::new(radius)))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: f32, hy: f32) -> Self {
        let cuboid = Cuboid {
            half_extents: Vector::new(hx, hy),
        };

        Self::new(Shape::Cuboid(cuboid))

        /*
        use crate::math::Point;
        let vertices = vec![
            Point::new(hx, -hy),
            Point::new(hx, hy),
            Point::new(-hx, hy),
            Point::new(-hx, -hy),
        ];
        let normals = vec![Vector::x(), Vector::y(), -Vector::x(), -Vector::y()];
        let polygon = Polygon::new(vertices, normals);

        Self::new(Shape::Polygon(polygon))
        */
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: f32, radius: f32) -> Self {
        let capsule = Capsule::new_x(half_height, radius);
        Self::new(Shape::Capsule(capsule))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: f32, radius: f32) -> Self {
        let capsule = Capsule::new_y(half_height, radius);
        Self::new(Shape::Capsule(capsule))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: f32, radius: f32) -> Self {
        let capsule = Capsule::new_z(half_height, radius);
        Self::new(Shape::Capsule(capsule))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: f32, hy: f32, hz: f32) -> Self {
        let cuboid = Cuboid {
            half_extents: Vector::new(hx, hy, hz),
        };

        Self::new(Shape::Cuboid(cuboid))
    }

    /// Initializes a collider builder with a segment shape.
    ///
    /// A segment shape is modeled by a capsule with a 0 radius.
    pub fn segment(a: Point<f32>, b: Point<f32>) -> Self {
        let capsule = Capsule::new(a, b, 0.0);
        Self::new(Shape::Capsule(capsule))
    }

    /// Initializes a collider builder with a triangle shape.
    pub fn triangle(a: Point<f32>, b: Point<f32>, c: Point<f32>) -> Self {
        let triangle = Triangle::new(a, b, c);
        Self::new(Shape::Triangle(triangle))
    }

    /// Initializes a collider builder with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Point<f32>>, indices: Vec<Point3<u32>>) -> Self {
        let trimesh = Trimesh::new(vertices, indices);
        Self::new(Shape::Trimesh(trimesh))
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: na::DVector<f32>, scale: Vector<f32>) -> Self {
        let heightfield = HeightField::new(heights, scale);
        Self::new(Shape::HeightField(heightfield))
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: na::DMatrix<f32>, scale: Vector<f32>) -> Self {
        let heightfield = HeightField::new(heights, scale);
        Self::new(Shape::HeightField(heightfield))
    }

    /// The default friction coefficient used by the collider builder.
    pub fn default_friction() -> f32 {
        0.5
    }

    /// Sets whether or not the collider built by this builder is a sensor.
    pub fn sensor(mut self, is_sensor: bool) -> Self {
        self.is_sensor = is_sensor;
        self
    }

    /// Sets the friction coefficient of the collider this builder will build.
    pub fn friction(mut self, friction: f32) -> Self {
        self.friction = friction;
        self
    }

    /// Sets the restitution coefficient of the collider this builder will build.
    pub fn restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution;
        self
    }

    /// Sets the density of the collider this builder will build.
    pub fn density(mut self, density: f32) -> Self {
        self.density = Some(density);
        self
    }

    /// Sets the initial translation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    #[cfg(feature = "dim2")]
    pub fn translation(mut self, x: f32, y: f32) -> Self {
        self.delta.translation.x = x;
        self.delta.translation.y = y;
        self
    }

    /// Sets the initial translation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    #[cfg(feature = "dim3")]
    pub fn translation(mut self, x: f32, y: f32, z: f32) -> Self {
        self.delta.translation.x = x;
        self.delta.translation.y = y;
        self.delta.translation.z = z;
        self
    }

    /// Sets the initial orientation of the collider to be created,
    /// relative to the rigid-body it is attached to.
    pub fn rotation(mut self, angle: AngVector<f32>) -> Self {
        self.delta.rotation = Rotation::new(angle);
        self
    }

    /// Sets the initial position (translation and orientation) of the collider to be created,
    /// relative to the rigid-body it is attached to.
    pub fn position(mut self, pos: Isometry<f32>) -> Self {
        self.delta = pos;
        self
    }

    /// Set the position of this collider in the local-space of the rigid-body it is attached to.
    #[deprecated(note = "Use `.position` instead.")]
    pub fn delta(mut self, delta: Isometry<f32>) -> Self {
        self.delta = delta;
        self
    }

    /// Buildes a new collider attached to the given rigid-body.
    pub fn build(&self) -> Collider {
        let density = self.get_density();

        Collider {
            shape: self.shape.clone(),
            density,
            friction: self.friction,
            restitution: self.restitution,
            delta: self.delta,
            is_sensor: self.is_sensor,
            parent: RigidBodySet::invalid_handle(),
            position: Isometry::identity(),
            predicted_position: Isometry::identity(),
            contact_graph_index: InteractionGraph::<Contact>::invalid_graph_index(),
            proximity_graph_index: InteractionGraph::<Proximity>::invalid_graph_index(),
            proxy_index: crate::INVALID_USIZE,
        }
    }
}
