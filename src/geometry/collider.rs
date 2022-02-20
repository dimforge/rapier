use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle};
use crate::geometry::{
    ActiveCollisionTypes, ColliderBroadPhaseData, ColliderChanges, ColliderFlags,
    ColliderMassProps, ColliderMaterial, ColliderParent, ColliderPosition, ColliderShape,
    ColliderType, InteractionGroups, SharedShape,
};
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector, DIM};
use crate::parry::transformation::vhacd::VHACDParameters;
use crate::pipeline::{ActiveEvents, ActiveHooks};
use na::Unit;
use parry::bounding_volume::AABB;
use parry::shape::Shape;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
///
/// To build a new collider, use the `ColliderBuilder` structure.
pub struct Collider {
    pub(crate) co_type: ColliderType,
    pub(crate) co_shape: ColliderShape,
    pub(crate) co_mprops: ColliderMassProps,
    pub(crate) co_changes: ColliderChanges,
    pub(crate) co_parent: Option<ColliderParent>,
    pub(crate) co_pos: ColliderPosition,
    pub(crate) co_material: ColliderMaterial,
    pub(crate) co_flags: ColliderFlags,
    pub(crate) co_bf_data: ColliderBroadPhaseData,
    /// User-defined data associated to this collider.
    pub user_data: u128,
}

impl Collider {
    pub(crate) fn reset_internal_references(&mut self) {
        self.co_bf_data.proxy_index = crate::INVALID_U32;
        self.co_changes = ColliderChanges::all();
    }

    /// The rigid body this collider is attached to.
    pub fn parent(&self) -> Option<RigidBodyHandle> {
        self.co_parent.map(|parent| parent.handle)
    }

    /// Is this collider a sensor?
    pub fn is_sensor(&self) -> bool {
        self.co_type.is_sensor()
    }

    /// The physics hooks enabled for this collider.
    pub fn active_hooks(&self) -> ActiveHooks {
        self.co_flags.active_hooks
    }

    /// Sets the physics hooks enabled for this collider.
    pub fn set_active_hooks(&mut self, active_hooks: ActiveHooks) {
        self.co_flags.active_hooks = active_hooks;
    }

    /// The events enabled for this collider.
    pub fn active_events(&self) -> ActiveEvents {
        self.co_flags.active_events
    }

    /// Sets the events enabled for this collider.
    pub fn set_active_events(&mut self, active_events: ActiveEvents) {
        self.co_flags.active_events = active_events;
    }

    /// The collision types enabled for this collider.
    pub fn active_collision_types(&self) -> ActiveCollisionTypes {
        self.co_flags.active_collision_types
    }

    /// Sets the collision types enabled for this collider.
    pub fn set_active_collision_types(&mut self, active_collision_types: ActiveCollisionTypes) {
        self.co_flags.active_collision_types = active_collision_types;
    }

    /// The friction coefficient of this collider.
    pub fn friction(&self) -> Real {
        self.co_material.friction
    }

    /// Sets the friction coefficient of this collider.
    pub fn set_friction(&mut self, coefficient: Real) {
        self.co_material.friction = coefficient
    }

    /// The combine rule used by this collider to combine its friction
    /// coefficient with the friction coefficient of the other collider it
    /// is in contact with.
    pub fn friction_combine_rule(&self) -> CoefficientCombineRule {
        self.co_material.friction_combine_rule
    }

    /// Sets the combine rule used by this collider to combine its friction
    /// coefficient with the friction coefficient of the other collider it
    /// is in contact with.
    pub fn set_friction_combine_rule(&mut self, rule: CoefficientCombineRule) {
        self.co_material.friction_combine_rule = rule;
    }

    /// The restitution coefficient of this collider.
    pub fn restitution(&self) -> Real {
        self.co_material.restitution
    }

    /// Sets the restitution coefficient of this collider.
    pub fn set_restitution(&mut self, coefficient: Real) {
        self.co_material.restitution = coefficient
    }

    /// The combine rule used by this collider to combine its restitution
    /// coefficient with the restitution coefficient of the other collider it
    /// is in contact with.
    pub fn restitution_combine_rule(&self) -> CoefficientCombineRule {
        self.co_material.restitution_combine_rule
    }

    /// Sets the combine rule used by this collider to combine its restitution
    /// coefficient with the restitution coefficient of the other collider it
    /// is in contact with.
    pub fn set_restitution_combine_rule(&mut self, rule: CoefficientCombineRule) {
        self.co_material.restitution_combine_rule = rule;
    }

    /// Sets whether or not this is a sensor collider.
    pub fn set_sensor(&mut self, is_sensor: bool) {
        if is_sensor != self.is_sensor() {
            self.co_changes.insert(ColliderChanges::TYPE);
            self.co_type = if is_sensor {
                ColliderType::Sensor
            } else {
                ColliderType::Solid
            };
        }
    }

    /// Sets the translational part of this collider's position.
    pub fn set_translation(&mut self, translation: Vector<Real>) {
        self.co_changes.insert(ColliderChanges::POSITION);
        self.co_pos.0.translation.vector = translation;
    }

    /// Sets the rotational part of this collider's position.
    pub fn set_rotation(&mut self, rotation: AngVector<Real>) {
        self.co_changes.insert(ColliderChanges::POSITION);
        self.co_pos.0.rotation = Rotation::new(rotation);
    }

    /// Sets the position of this collider.
    pub fn set_position(&mut self, position: Isometry<Real>) {
        self.co_changes.insert(ColliderChanges::POSITION);
        self.co_pos.0 = position;
    }

    /// The world-space position of this collider.
    pub fn position(&self) -> &Isometry<Real> {
        &self.co_pos
    }

    /// The translational part of this collider's position.
    pub fn translation(&self) -> &Vector<Real> {
        &self.co_pos.0.translation.vector
    }

    /// The rotational part of this collider's position.
    pub fn rotation(&self) -> &Rotation<Real> {
        &self.co_pos.0.rotation
    }

    /// The position of this collider wrt the body it is attached to.
    pub fn position_wrt_parent(&self) -> Option<&Isometry<Real>> {
        self.co_parent.as_ref().map(|p| &p.pos_wrt_parent)
    }

    /// Sets the translational part of this collider's translation relative to its parent rigid-body.
    pub fn set_translation_wrt_parent(&mut self, translation: Vector<Real>) {
        if let Some(co_parent) = self.co_parent.as_mut() {
            self.co_changes.insert(ColliderChanges::PARENT);
            co_parent.pos_wrt_parent.translation.vector = translation;
        }
    }

    /// Sets the rotational part of this collider's rotaiton relative to its parent rigid-body.
    pub fn set_rotation_wrt_parent(&mut self, rotation: AngVector<Real>) {
        if let Some(co_parent) = self.co_parent.as_mut() {
            self.co_changes.insert(ColliderChanges::PARENT);
            co_parent.pos_wrt_parent.rotation = Rotation::new(rotation);
        }
    }

    /// Sets the position of this collider wrt. its parent rigid-body.
    ///
    /// Does nothing if the collider is not attached to a rigid-body.
    pub fn set_position_wrt_parent(&mut self, pos_wrt_parent: Isometry<Real>) {
        if let Some(co_parent) = self.co_parent.as_mut() {
            self.co_changes.insert(ColliderChanges::PARENT);
            co_parent.pos_wrt_parent = pos_wrt_parent;
        }
    }

    /// The collision groups used by this collider.
    pub fn collision_groups(&self) -> InteractionGroups {
        self.co_flags.collision_groups
    }

    /// Sets the collision groups of this collider.
    pub fn set_collision_groups(&mut self, groups: InteractionGroups) {
        if self.co_flags.collision_groups != groups {
            self.co_changes.insert(ColliderChanges::GROUPS);
            self.co_flags.collision_groups = groups;
        }
    }

    /// The solver groups used by this collider.
    pub fn solver_groups(&self) -> InteractionGroups {
        self.co_flags.solver_groups
    }

    /// Sets the solver groups of this collider.
    pub fn set_solver_groups(&mut self, groups: InteractionGroups) {
        if self.co_flags.solver_groups != groups {
            self.co_changes.insert(ColliderChanges::GROUPS);
            self.co_flags.solver_groups = groups;
        }
    }

    /// The material (friction and restitution properties) of this collider.
    pub fn material(&self) -> &ColliderMaterial {
        &self.co_material
    }

    /// The density of this collider, if set.
    pub fn density(&self) -> Option<Real> {
        match &self.co_mprops {
            ColliderMassProps::Density(density) => Some(*density),
            ColliderMassProps::MassProperties(_) => None,
        }
    }

    /// The geometric shape of this collider.
    pub fn shape(&self) -> &dyn Shape {
        self.co_shape.as_ref()
    }

    /// A mutable reference to the geometric shape of this collider.
    ///
    /// If that shape is shared by multiple colliders, it will be
    /// cloned first so that `self` contains a unique copy of that
    /// shape that you can modify.
    pub fn shape_mut(&mut self) -> &mut dyn Shape {
        self.co_changes.insert(ColliderChanges::SHAPE);
        self.co_shape.make_mut()
    }

    /// Sets the shape of this collider.
    pub fn set_shape(&mut self, shape: SharedShape) {
        self.co_changes.insert(ColliderChanges::SHAPE);
        self.co_shape = shape;
    }

    /// Retrieve the SharedShape. Also see the `shape()` function
    pub fn shared_shape(&self) -> &SharedShape {
        &self.co_shape
    }

    /// Compute the axis-aligned bounding box of this collider.
    pub fn compute_aabb(&self) -> AABB {
        self.co_shape.compute_aabb(&self.co_pos)
    }

    /// Compute the axis-aligned bounding box of this collider moving from its current position
    /// to the given `next_position`
    pub fn compute_swept_aabb(&self, next_position: &Isometry<Real>) -> AABB {
        self.co_shape
            .compute_swept_aabb(&self.co_pos, next_position)
    }

    /// Compute the local-space mass properties of this collider.
    pub fn mass_properties(&self) -> MassProperties {
        match &self.co_mprops {
            ColliderMassProps::Density(density) => self.co_shape.mass_properties(*density),
            ColliderMassProps::MassProperties(mass_properties) => **mass_properties,
        }
    }
}

/// A structure responsible for building a new collider.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[must_use = "Builder functions return the updated builder"]
pub struct ColliderBuilder {
    /// The shape of the collider to be built.
    pub shape: SharedShape,
    /// The uniform density of the collider to be built.
    pub density: Option<Real>,
    /// Overrides automatic computation of `MassProperties`.
    /// If None, it will be computed based on shape and density.
    pub mass_properties: Option<MassProperties>,
    /// The friction coefficient of the collider to be built.
    pub friction: Real,
    /// The rule used to combine two friction coefficients.
    pub friction_combine_rule: CoefficientCombineRule,
    /// The restitution coefficient of the collider to be built.
    pub restitution: Real,
    /// The rule used to combine two restitution coefficients.
    pub restitution_combine_rule: CoefficientCombineRule,
    /// The position of this collider.
    pub position: Isometry<Real>,
    /// Is this collider a sensor?
    pub is_sensor: bool,
    /// Contact pairs enabled for this collider.
    pub active_collision_types: ActiveCollisionTypes,
    /// Physics hooks enabled for this collider.
    pub active_hooks: ActiveHooks,
    /// Events enabled for this collider.
    pub active_events: ActiveEvents,
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
            position: Isometry::identity(),
            is_sensor: false,
            user_data: 0,
            collision_groups: InteractionGroups::all(),
            solver_groups: InteractionGroups::all(),
            friction_combine_rule: CoefficientCombineRule::Average,
            restitution_combine_rule: CoefficientCombineRule::Average,
            active_collision_types: ActiveCollisionTypes::default(),
            active_hooks: ActiveHooks::empty(),
            active_events: ActiveEvents::empty(),
        }
    }

    /// Initialize a new collider builder with a compound shape.
    pub fn compound(shapes: Vec<(Isometry<Real>, SharedShape)>) -> Self {
        Self::new(SharedShape::compound(shapes))
    }

    /// Initialize a new collider builder with a ball shape defined by its radius.
    pub fn ball(radius: Real) -> Self {
        Self::new(SharedShape::ball(radius))
    }

    /// Initialize a new collider build with a half-space shape defined by the outward normal
    /// of its planar boundary.
    pub fn halfspace(outward_normal: Unit<Vector<Real>>) -> Self {
        Self::new(SharedShape::halfspace(outward_normal))
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

    /// The default density used by the collider builder.
    pub fn default_density() -> Real {
        1.0
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
    ///
    /// Sensors will have a default density of zero,
    /// but if you call [`Self::mass_properties`] you can assign a mass to a sensor.
    pub fn sensor(mut self, is_sensor: bool) -> Self {
        self.is_sensor = is_sensor;
        self
    }

    /// The set of physics hooks enabled for this collider.
    pub fn active_hooks(mut self, active_hooks: ActiveHooks) -> Self {
        self.active_hooks = active_hooks;
        self
    }

    /// The set of events enabled for this collider.
    pub fn active_events(mut self, active_events: ActiveEvents) -> Self {
        self.active_events = active_events;
        self
    }

    /// The set of active collision types for this collider.
    pub fn active_collision_types(mut self, active_collision_types: ActiveCollisionTypes) -> Self {
        self.active_collision_types = active_collision_types;
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
    ///
    /// This will be overridden by a call to [`Self::mass_properties`] so it only makes sense to call
    /// either [`Self::density`] or [`Self::mass_properties`].
    pub fn density(mut self, density: Real) -> Self {
        self.density = Some(density);
        self
    }

    /// Sets the mass properties of the collider this builder will build.
    ///
    /// If this is set, [`Self::density`] will be ignored, so it only makes sense to call
    /// either [`Self::density`] or [`Self::mass_properties`].
    pub fn mass_properties(mut self, mass_properties: MassProperties) -> Self {
        self.mass_properties = Some(mass_properties);
        self
    }

    /// Sets the initial translation of the collider to be created.
    ///
    /// If the collider will be attached to a rigid-body, this sets the translation relative to the
    /// rigid-body it will be attached to.
    pub fn translation(mut self, translation: Vector<Real>) -> Self {
        self.position.translation.vector = translation;
        self
    }

    /// Sets the initial orientation of the collider to be created.
    ///
    /// If the collider will be attached to a rigid-body, this sets the orientation relative to the
    /// rigid-body it will be attached to.
    pub fn rotation(mut self, angle: AngVector<Real>) -> Self {
        self.position.rotation = Rotation::new(angle);
        self
    }

    /// Sets the initial position (translation and orientation) of the collider to be created.
    ///
    /// If the collider will be attached to a rigid-body, this sets the position relative
    /// to the rigid-body it will be attached to.
    pub fn position(mut self, pos: Isometry<Real>) -> Self {
        self.position = pos;
        self
    }

    /// Sets the initial position (translation and orientation) of the collider to be created,
    /// relative to the rigid-body it is attached to.
    #[deprecated(note = "Use `.position` instead.")]
    pub fn position_wrt_parent(mut self, pos: Isometry<Real>) -> Self {
        self.position = pos;
        self
    }

    /// Set the position of this collider in the local-space of the rigid-body it is attached to.
    #[deprecated(note = "Use `.position` instead.")]
    pub fn delta(mut self, delta: Isometry<Real>) -> Self {
        self.position = delta;
        self
    }

    /// Builds a new collider attached to the given rigid-body.
    pub fn build(&self) -> Collider {
        let (co_changes, co_pos, co_bf_data, co_shape, co_type, co_material, co_flags, co_mprops) =
            self.components();
        Collider {
            co_shape,
            co_mprops,
            co_material,
            co_parent: None,
            co_changes,
            co_pos,
            co_bf_data,
            co_flags,
            co_type,
            user_data: self.user_data,
        }
    }

    /// Builds all the components required by a collider.
    pub fn components(
        &self,
    ) -> (
        ColliderChanges,
        ColliderPosition,
        ColliderBroadPhaseData,
        ColliderShape,
        ColliderType,
        ColliderMaterial,
        ColliderFlags,
        ColliderMassProps,
    ) {
        let mass_info = if let Some(mp) = self.mass_properties {
            ColliderMassProps::MassProperties(Box::new(mp))
        } else {
            let default_density = Self::default_density();
            let density = self.density.unwrap_or(default_density);
            ColliderMassProps::Density(density)
        };

        let co_shape = self.shape.clone();
        let co_mprops = mass_info;
        let co_material = ColliderMaterial {
            friction: self.friction,
            restitution: self.restitution,
            friction_combine_rule: self.friction_combine_rule,
            restitution_combine_rule: self.restitution_combine_rule,
        };
        let co_flags = ColliderFlags {
            collision_groups: self.collision_groups,
            solver_groups: self.solver_groups,
            active_collision_types: self.active_collision_types,
            active_hooks: self.active_hooks,
            active_events: self.active_events,
        };
        let co_changes = ColliderChanges::all();
        let co_pos = ColliderPosition(self.position);
        let co_bf_data = ColliderBroadPhaseData::default();
        let co_type = if self.is_sensor {
            ColliderType::Sensor
        } else {
            ColliderType::Solid
        };

        (
            co_changes,
            co_pos,
            co_bf_data,
            co_shape,
            co_type,
            co_material,
            co_flags,
            co_mprops,
        )
    }
}

impl Into<Collider> for ColliderBuilder {
    fn into(self) -> Collider {
        self.build()
    }
}
