use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle};
use crate::geometry::{InteractionGroups, SharedShape, SolverFlags};
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector, DIM};
use crate::parry::transformation::vhacd::VHACDParameters;
use parry::bounding_volume::AABB;
use parry::shape::Shape;

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub(crate) struct ColliderFlags: u8 {
        const SENSOR = 1 << 0;
        const FRICTION_COMBINE_RULE_01 = 1 << 1;
        const FRICTION_COMBINE_RULE_10 = 1 << 2;
        const RESTITUTION_COMBINE_RULE_01 = 1 << 3;
        const RESTITUTION_COMBINE_RULE_10 = 1 << 4;
    }
}

impl ColliderFlags {
    pub fn is_sensor(self) -> bool {
        self.contains(ColliderFlags::SENSOR)
    }

    pub fn friction_combine_rule_value(self) -> u8 {
        (self.bits & 0b0000_0110) >> 1
    }

    pub fn restitution_combine_rule_value(self) -> u8 {
        (self.bits & 0b0001_1000) >> 3
    }

    pub fn with_friction_combine_rule(mut self, rule: CoefficientCombineRule) -> Self {
        self.bits = (self.bits & !0b0000_0110) | ((rule as u8) << 1);
        self
    }

    pub fn with_restitution_combine_rule(mut self, rule: CoefficientCombineRule) -> Self {
        self.bits = (self.bits & !0b0001_1000) | ((rule as u8) << 3);
        self
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
///
/// To build a new collider, use the `ColliderBuilder` structure.
pub struct Collider {
    shape: SharedShape,
    density: Real,
    mass_properties: MassProperties,
    pub(crate) flags: ColliderFlags,
    pub(crate) solver_flags: SolverFlags,
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
        self.flags.is_sensor()
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

    /// Read the local-space mass properties of this collider.
    pub fn mass_properties(&self) -> &MassProperties {
        &self.mass_properties
    }
}

/// A structure responsible for building a new collider.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderBuilder {
    /// The shape of the collider to be built.
    pub shape: SharedShape,
    /// The uniform density of the collider to be built.
    density: Option<Real>,
    /// Overrides automatic computation of `MassProperties`.
    /// If None, it will be computed based on shape and desnity.
    mass_properties: Option<MassProperties>,
    /// The friction coefficient of the collider to be built.
    pub friction: Real,
    /// The rule used to combine two friction coefficients.
    pub friction_combine_rule: CoefficientCombineRule,
    /// The restitution coefficient of the collider to be built.
    pub restitution: Real,
    /// The rule used to combine two restitution coefficients.
    pub restitution_combine_rule: CoefficientCombineRule,
    /// The position of this collider relative to the local frame of the rigid-body it is attached to.
    pub delta: Isometry<Real>,
    /// Is this collider a sensor?
    pub is_sensor: bool,
    /// Do we have to always call the contact modifier
    /// on this collider?
    pub modify_solver_contacts: bool,
    /// The user-data of the collider being built.
    pub user_data: u128,
    /// The collision groups for the collider being built.
    pub collision_groups: InteractionGroups,
    /// The solver groups for the collider being built.
    pub solver_groups: InteractionGroups,
}

impl ColliderBuilder {
    /// Initialize a new collider builder with the given shape.
    pub fn new(shape: SharedShape) -> Self {
        Self {
            shape,
            density: None,
            mass_properties: None,
            friction: Self::default_friction(),
            restitution: 0.0,
            delta: Isometry::identity(),
            is_sensor: false,
            user_data: 0,
            collision_groups: InteractionGroups::all(),
            solver_groups: InteractionGroups::all(),
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution_combine_rule: CoefficientCombineRule::Average,
            modify_solver_contacts: false,
        }
    }

    /// The density of the collider being built.
    pub fn get_density(&self) -> Real {
        let default_density = if self.is_sensor { 0.0 } else { 1.0 };
        self.density.unwrap_or(default_density)
    }

    /// Initialize a new collider builder with a compound shape.
    pub fn compound(shapes: Vec<(Isometry<Real>, SharedShape)>) -> Self {
        Self::new(SharedShape::compound(shapes))
    }

    /// Initialize a new collider builder with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        Self::new(SharedShape::ball(radius))
    }

    /// Initialize a new collider builder with a cylindrical shape defined by its half-height
    /// (along along the y axis) and its radius.
    #[cfg(feature = "dim3")]
    pub fn cylinder(half_height: Real, radius: Real) -> Self {
        Self::new(SharedShape::cylinder(half_height, radius))
    }

    /// Initialize a new collider builder with a rounded cylindrical shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cylinder(half_height: Real, radius: Real, border_radius: Real) -> Self {
        Self::new(SharedShape::round_cylinder(
            half_height,
            radius,
            border_radius,
        ))
    }

    /// Initialize a new collider builder with a cone shape defined by its half-height
    /// (along along the y axis) and its basis radius.
    #[cfg(feature = "dim3")]
    pub fn cone(half_height: Real, radius: Real) -> Self {
        Self::new(SharedShape::cone(half_height, radius))
    }

    /// Initialize a new collider builder with a rounded cone shape defined by its half-height
    /// (along along the y axis), its radius, and its roundedness (the
    /// radius of the sphere used for dilating the cylinder).
    #[cfg(feature = "dim3")]
    pub fn round_cone(half_height: Real, radius: Real, border_radius: Real) -> Self {
        Self::new(SharedShape::round_cone(half_height, radius, border_radius))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim2")]
    pub fn cuboid(hx: Real, hy: Real) -> Self {
        Self::new(SharedShape::cuboid(hx, hy))
    }

    /// Initialize a new collider builder with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim2")]
    pub fn round_cuboid(hx: Real, hy: Real, border_radius: Real) -> Self {
        Self::new(SharedShape::round_cuboid(hx, hy, border_radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `x` axis.
    pub fn capsule_x(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::x() * half_height);
        Self::new(SharedShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::y() * half_height);
        Self::new(SharedShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        let p = Point::from(Vector::z() * half_height);
        Self::new(SharedShape::capsule(-p, p, radius))
    }

    /// Initialize a new collider builder with a cuboid shape defined by its half-extents.
    #[cfg(feature = "dim3")]
    pub fn cuboid(hx: Real, hy: Real, hz: Real) -> Self {
        Self::new(SharedShape::cuboid(hx, hy, hz))
    }

    /// Initialize a new collider builder with a round cuboid shape defined by its half-extents
    /// and border radius.
    #[cfg(feature = "dim3")]
    pub fn round_cuboid(hx: Real, hy: Real, hz: Real, border_radius: Real) -> Self {
        Self::new(SharedShape::round_cuboid(hx, hy, hz, border_radius))
    }

    /// Initializes a collider builder with a segment shape.
    pub fn segment(a: Point<Real>, b: Point<Real>) -> Self {
        Self::new(SharedShape::segment(a, b))
    }

    /// Initializes a collider builder with a triangle shape.
    pub fn triangle(a: Point<Real>, b: Point<Real>, c: Point<Real>) -> Self {
        Self::new(SharedShape::triangle(a, b, c))
    }

    /// Initializes a collider builder with a triangle shape with round corners.
    pub fn round_triangle(
        a: Point<Real>,
        b: Point<Real>,
        c: Point<Real>,
        border_radius: Real,
    ) -> Self {
        Self::new(SharedShape::round_triangle(a, b, c, border_radius))
    }

    /// Initializes a collider builder with a polyline shape defined by its vertex and index buffers.
    pub fn polyline(vertices: Vec<Point<Real>>, indices: Option<Vec<[u32; 2]>>) -> Self {
        Self::new(SharedShape::polyline(vertices, indices))
    }

    /// Initializes a collider builder with a triangle mesh shape defined by its vertex and index buffers.
    pub fn trimesh(vertices: Vec<Point<Real>>, indices: Vec<[u32; 3]>) -> Self {
        Self::new(SharedShape::trimesh(vertices, indices))
    }

    /// Initializes a collider builder with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition(vertices: &[Point<Real>], indices: &[[u32; DIM]]) -> Self {
        Self::new(SharedShape::convex_decomposition(vertices, indices))
    }

    /// Initializes a collider builder with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        border_radius: Real,
    ) -> Self {
        Self::new(SharedShape::round_convex_decomposition(
            vertices,
            indices,
            border_radius,
        ))
    }

    /// Initializes a collider builder with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts.
    pub fn convex_decomposition_with_params(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
    ) -> Self {
        Self::new(SharedShape::convex_decomposition_with_params(
            vertices, indices, params,
        ))
    }

    /// Initializes a collider builder with a compound shape obtained from the decomposition of
    /// the given trimesh (in 3D) or polyline (in 2D) into convex parts dilated with round corners.
    pub fn round_convex_decomposition_with_params(
        vertices: &[Point<Real>],
        indices: &[[u32; DIM]],
        params: &VHACDParameters,
        border_radius: Real,
    ) -> Self {
        Self::new(SharedShape::round_convex_decomposition_with_params(
            vertices,
            indices,
            params,
            border_radius,
        ))
    }

    /// Initializes a new collider builder with a 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points.
    pub fn convex_hull(points: &[Point<Real>]) -> Option<Self> {
        SharedShape::convex_hull(points).map(Self::new)
    }

    /// Initializes a new collider builder with a round 2D convex polygon or 3D convex polyhedron
    /// obtained after computing the convex-hull of the given points. The shape is dilated
    /// by a sphere of radius `border_radius`.
    pub fn round_convex_hull(points: &[Point<Real>], border_radius: Real) -> Option<Self> {
        SharedShape::round_convex_hull(points, border_radius).map(Self::new)
    }

    /// Creates a new collider builder that is a convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim2")]
    pub fn convex_polyline(points: Vec<Point<Real>>) -> Option<Self> {
        SharedShape::convex_polyline(points).map(Self::new)
    }

    /// Creates a new collider builder that is a round convex polygon formed by the
    /// given polyline assumed to be convex (no convex-hull will be automatically
    /// computed). The polygon shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim2")]
    pub fn round_convex_polyline(points: Vec<Point<Real>>, border_radius: Real) -> Option<Self> {
        SharedShape::round_convex_polyline(points, border_radius).map(Self::new)
    }

    /// Creates a new collider builder that is a convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed).
    #[cfg(feature = "dim3")]
    pub fn convex_mesh(points: Vec<Point<Real>>, indices: &[[u32; 3]]) -> Option<Self> {
        SharedShape::convex_mesh(points, indices).map(Self::new)
    }

    /// Creates a new collider builder that is a round convex polyhedron formed by the
    /// given triangle-mesh assumed to be convex (no convex-hull will be automatically
    /// computed). The triangle mesh shape is dilated by a sphere of radius `border_radius`.
    #[cfg(feature = "dim3")]
    pub fn round_convex_mesh(
        points: Vec<Point<Real>>,
        indices: &[[u32; 3]],
        border_radius: Real,
    ) -> Option<Self> {
        SharedShape::round_convex_mesh(points, indices, border_radius).map(Self::new)
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim2")]
    pub fn heightfield(heights: na::DVector<Real>, scale: Vector<Real>) -> Self {
        Self::new(SharedShape::heightfield(heights, scale))
    }

    /// Initializes a collider builder with a heightfield shape defined by its set of height and a scale
    /// factor along each coordinate axis.
    #[cfg(feature = "dim3")]
    pub fn heightfield(heights: na::DMatrix<Real>, scale: Vector<Real>) -> Self {
        Self::new(SharedShape::heightfield(heights, scale))
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
    /// Sensors will have a default density of zero,
    /// but if you call [`Self::mass_properties`] you can assigna a mass to a sensor.
    pub fn sensor(mut self, is_sensor: bool) -> Self {
        self.is_sensor = is_sensor;
        self
    }

    /// If set to `true` then the physics hooks will always run to modify
    /// contacts involving this collider.
    pub fn modify_solver_contacts(mut self, modify_solver_contacts: bool) -> Self {
        self.modify_solver_contacts = modify_solver_contacts;
        self
    }

    /// Sets the friction coefficient of the collider this builder will build.
    pub fn friction(mut self, friction: Real) -> Self {
        self.friction = friction;
        self
    }

    /// Sets the rule to be used to combine two friction coefficients in a contact.
    pub fn friction_combine_rule(mut self, rule: CoefficientCombineRule) -> Self {
        self.friction_combine_rule = rule;
        self
    }

    /// Sets the restitution coefficient of the collider this builder will build.
    pub fn restitution(mut self, restitution: Real) -> Self {
        self.restitution = restitution;
        self
    }

    /// Sets the rule to be used to combine two restitution coefficients in a contact.
    pub fn restitution_combine_rule(mut self, rule: CoefficientCombineRule) -> Self {
        self.restitution_combine_rule = rule;
        self
    }

    /// Sets the uniform density of the collider this builder will build.
    /// This will be overridden by a call to [`Self::mass_properties`] so it only makes sense to call
    /// either [`Self::density`] or [`Self::mass_properties`].
    pub fn density(mut self, density: Real) -> Self {
        self.density = Some(density);
        self
    }

    /// Sets the mass properties of the collider this builder will build.
    /// If this is set, [`Self::density`] will be ignored, so it only makes sense to call
    /// either [`Self::density`] or [`Self::mass_properties`].
    pub fn mass_properties(mut self, mass_properties: MassProperties) -> Self {
        self.mass_properties = Some(mass_properties);
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
        let (density, mass_properties);
        if let Some(mp) = self.mass_properties {
            mass_properties = mp;

            let volume = volume(&self.shape);
            density = if volume == 0.0 || mp.inv_mass == 0.0 {
                Real::INFINITY
            } else {
                mass(&mp) / volume
            };
        } else {
            density = self.get_density();
            mass_properties = self.shape.mass_properties(density);
        }

        let mut flags = ColliderFlags::empty();
        flags.set(ColliderFlags::SENSOR, self.is_sensor);
        flags = flags
            .with_friction_combine_rule(self.friction_combine_rule)
            .with_restitution_combine_rule(self.restitution_combine_rule);
        let mut solver_flags = SolverFlags::default();
        solver_flags.set(
            SolverFlags::MODIFY_SOLVER_CONTACTS,
            self.modify_solver_contacts,
        );

        Collider {
            shape: self.shape.clone(),
            density,
            mass_properties,
            friction: self.friction,
            restitution: self.restitution,
            delta: self.delta,
            flags,
            solver_flags,
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

fn volume(shape: &SharedShape) -> Real {
    mass(&shape.mass_properties(1.0)) // TODO: add SharedShape::volume to parry
}

fn mass(mp: &MassProperties) -> Real {
    crate::utils::inv(mp.inv_mass) // TODO: add MassProperties::mass() to parry
}
