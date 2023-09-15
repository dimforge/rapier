use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle};
use crate::geometry::{
    ActiveCollisionTypes, ColliderBroadPhaseData, ColliderChanges, ColliderFlags,
    ColliderMassProps, ColliderMaterial, ColliderParent, ColliderPosition, ColliderShape,
    ColliderType, InteractionGroups, SharedShape,
};
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector, DIM};
use crate::parry::transformation::vhacd::VHACDParameters;
use crate::pipeline::{ActiveEvents, ActiveHooks};
use crate::prelude::ColliderEnabled;
use na::Unit;
use parry::bounding_volume::Aabb;
use parry::shape::{Shape, TriMeshFlags};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A geometric entity that can be attached to a body so it can be affected by contacts and proximity queries.
///
/// To build a new collider, use the [`ColliderBuilder`] structure.
pub struct Collider {
    pub(crate) coll_type: ColliderType,
    pub(crate) shape: ColliderShape,
    pub(crate) mprops: ColliderMassProps,
    pub(crate) changes: ColliderChanges,
    pub(crate) parent: Option<ColliderParent>,
    pub(crate) pos: ColliderPosition,
    pub(crate) material: ColliderMaterial,
    pub(crate) flags: ColliderFlags,
    pub(crate) bf_data: ColliderBroadPhaseData,
    contact_force_event_threshold: Real,
    /// User-defined data associated to this collider.
    pub user_data: u128,
}

impl Collider {
    pub(crate) fn reset_internal_references(&mut self) {
        self.bf_data.proxy_index = crate::INVALID_U32;
        self.changes = ColliderChanges::all();
    }

    pub(crate) fn effective_contact_force_event_threshold(&self) -> Real {
        if self
            .flags
            .active_events
            .contains(ActiveEvents::CONTACT_FORCE_EVENTS)
        {
            self.contact_force_event_threshold
        } else {
            Real::MAX
        }
    }

    /// The rigid body this collider is attached to.
    pub fn parent(&self) -> Option<RigidBodyHandle> {
        self.parent.map(|parent| parent.handle)
    }

    /// Is this collider a sensor?
    pub fn is_sensor(&self) -> bool {
        self.coll_type.is_sensor()
    }

    /// The physics hooks enabled for this collider.
    pub fn active_hooks(&self) -> ActiveHooks {
        self.flags.active_hooks
    }

    /// Sets the physics hooks enabled for this collider.
    pub fn set_active_hooks(&mut self, active_hooks: ActiveHooks) {
        self.flags.active_hooks = active_hooks;
    }

    /// The events enabled for this collider.
    pub fn active_events(&self) -> ActiveEvents {
        self.flags.active_events
    }

    /// Sets the events enabled for this collider.
    pub fn set_active_events(&mut self, active_events: ActiveEvents) {
        self.flags.active_events = active_events;
    }

    /// The collision types enabled for this collider.
    pub fn active_collision_types(&self) -> ActiveCollisionTypes {
        self.flags.active_collision_types
    }

    /// Sets the collision types enabled for this collider.
    pub fn set_active_collision_types(&mut self, active_collision_types: ActiveCollisionTypes) {
        self.flags.active_collision_types = active_collision_types;
    }

    /// The friction coefficient of this collider.
    pub fn friction(&self) -> Real {
        self.material.friction
    }

    /// Sets the friction coefficient of this collider.
    pub fn set_friction(&mut self, coefficient: Real) {
        self.material.friction = coefficient
    }

    /// The combine rule used by this collider to combine its friction
    /// coefficient with the friction coefficient of the other collider it
    /// is in contact with.
    pub fn friction_combine_rule(&self) -> CoefficientCombineRule {
        self.material.friction_combine_rule
    }

    /// Sets the combine rule used by this collider to combine its friction
    /// coefficient with the friction coefficient of the other collider it
    /// is in contact with.
    pub fn set_friction_combine_rule(&mut self, rule: CoefficientCombineRule) {
        self.material.friction_combine_rule = rule;
    }

    /// The restitution coefficient of this collider.
    pub fn restitution(&self) -> Real {
        self.material.restitution
    }

    /// Sets the restitution coefficient of this collider.
    pub fn set_restitution(&mut self, coefficient: Real) {
        self.material.restitution = coefficient
    }

    /// The combine rule used by this collider to combine its restitution
    /// coefficient with the restitution coefficient of the other collider it
    /// is in contact with.
    pub fn restitution_combine_rule(&self) -> CoefficientCombineRule {
        self.material.restitution_combine_rule
    }

    /// Sets the combine rule used by this collider to combine its restitution
    /// coefficient with the restitution coefficient of the other collider it
    /// is in contact with.
    pub fn set_restitution_combine_rule(&mut self, rule: CoefficientCombineRule) {
        self.material.restitution_combine_rule = rule;
    }

    /// Sets the total force magnitude beyond which a contact force event can be emitted.
    pub fn set_contact_force_event_threshold(&mut self, threshold: Real) {
        self.contact_force_event_threshold = threshold;
    }

    /// Sets whether or not this is a sensor collider.
    pub fn set_sensor(&mut self, is_sensor: bool) {
        if is_sensor != self.is_sensor() {
            self.changes.insert(ColliderChanges::TYPE);
            self.coll_type = if is_sensor {
                ColliderType::Sensor
            } else {
                ColliderType::Solid
            };
        }
    }

    /// Is this collider enabled?
    pub fn is_enabled(&self) -> bool {
        match self.flags.enabled {
            ColliderEnabled::Enabled => true,
            _ => false,
        }
    }

    /// Sets whether or not this collider is enabled.
    pub fn set_enabled(&mut self, enabled: bool) {
        match self.flags.enabled {
            ColliderEnabled::Enabled | ColliderEnabled::DisabledByParent => {
                if !enabled {
                    self.changes.insert(ColliderChanges::ENABLED_OR_DISABLED);
                    self.flags.enabled = ColliderEnabled::Disabled;
                }
            }
            ColliderEnabled::Disabled => {
                if enabled {
                    self.changes.insert(ColliderChanges::ENABLED_OR_DISABLED);
                    self.flags.enabled = ColliderEnabled::Enabled;
                }
            }
        }
    }

    /// Sets the translational part of this collider's position.
    pub fn set_translation(&mut self, translation: Vector<Real>) {
        self.changes.insert(ColliderChanges::POSITION);
        self.pos.0.translation.vector = translation;
    }

    /// Sets the rotational part of this collider's position.
    pub fn set_rotation(&mut self, rotation: Rotation<Real>) {
        self.changes.insert(ColliderChanges::POSITION);
        self.pos.0.rotation = rotation;
    }

    /// Sets the position of this collider.
    pub fn set_position(&mut self, position: Isometry<Real>) {
        self.changes.insert(ColliderChanges::POSITION);
        self.pos.0 = position;
    }

    /// The world-space position of this collider.
    pub fn position(&self) -> &Isometry<Real> {
        &self.pos
    }

    /// The translational part of this collider's position.
    pub fn translation(&self) -> &Vector<Real> {
        &self.pos.0.translation.vector
    }

    /// The rotational part of this collider's position.
    pub fn rotation(&self) -> &Rotation<Real> {
        &self.pos.0.rotation
    }

    /// The position of this collider with respect to the body it is attached to.
    pub fn position_wrt_parent(&self) -> Option<&Isometry<Real>> {
        self.parent.as_ref().map(|p| &p.pos_wrt_parent)
    }

    /// Sets the translational part of this collider's translation relative to its parent rigid-body.
    pub fn set_translation_wrt_parent(&mut self, translation: Vector<Real>) {
        if let Some(parent) = self.parent.as_mut() {
            self.changes.insert(ColliderChanges::PARENT);
            parent.pos_wrt_parent.translation.vector = translation;
        }
    }

    /// Sets the rotational part of this collider's rotaiton relative to its parent rigid-body.
    pub fn set_rotation_wrt_parent(&mut self, rotation: AngVector<Real>) {
        if let Some(parent) = self.parent.as_mut() {
            self.changes.insert(ColliderChanges::PARENT);
            parent.pos_wrt_parent.rotation = Rotation::new(rotation);
        }
    }

    /// Sets the position of this collider with respect to its parent rigid-body.
    ///
    /// Does nothing if the collider is not attached to a rigid-body.
    pub fn set_position_wrt_parent(&mut self, pos_wrt_parent: Isometry<Real>) {
        if let Some(parent) = self.parent.as_mut() {
            self.changes.insert(ColliderChanges::PARENT);
            parent.pos_wrt_parent = pos_wrt_parent;
        }
    }

    /// The collision groups used by this collider.
    pub fn collision_groups(&self) -> InteractionGroups {
        self.flags.collision_groups
    }

    /// Sets the collision groups of this collider.
    pub fn set_collision_groups(&mut self, groups: InteractionGroups) {
        if self.flags.collision_groups != groups {
            self.changes.insert(ColliderChanges::GROUPS);
            self.flags.collision_groups = groups;
        }
    }

    /// The solver groups used by this collider.
    pub fn solver_groups(&self) -> InteractionGroups {
        self.flags.solver_groups
    }

    /// Sets the solver groups of this collider.
    pub fn set_solver_groups(&mut self, groups: InteractionGroups) {
        if self.flags.solver_groups != groups {
            self.changes.insert(ColliderChanges::GROUPS);
            self.flags.solver_groups = groups;
        }
    }

    /// The material (friction and restitution properties) of this collider.
    pub fn material(&self) -> &ColliderMaterial {
        &self.material
    }

    /// The volume (or surface in 2D) of this collider.
    pub fn volume(&self) -> Real {
        self.shape.mass_properties(1.0).mass()
    }

    /// The density of this collider.
    pub fn density(&self) -> Real {
        match &self.mprops {
            ColliderMassProps::Density(density) => *density,
            ColliderMassProps::Mass(mass) => {
                let inv_volume = self.shape.mass_properties(1.0).inv_mass;
                mass * inv_volume
            }
            ColliderMassProps::MassProperties(mprops) => {
                let inv_volume = self.shape.mass_properties(1.0).inv_mass;
                mprops.mass() * inv_volume
            }
        }
    }

    /// The mass of this collider.
    pub fn mass(&self) -> Real {
        match &self.mprops {
            ColliderMassProps::Density(density) => self.shape.mass_properties(*density).mass(),
            ColliderMassProps::Mass(mass) => *mass,
            ColliderMassProps::MassProperties(mprops) => mprops.mass(),
        }
    }

    /// Sets the uniform density of this collider.
    ///
    /// This will override any previous mass-properties set by [`Self::set_density`],
    /// [`Self::set_mass`], [`Self::set_mass_properties`], [`ColliderBuilder::density`],
    /// [`ColliderBuilder::mass`], or [`ColliderBuilder::mass_properties`]
    /// for this collider.
    ///
    /// The mass and angular inertia of this collider will be computed automatically based on its
    /// shape.
    pub fn set_density(&mut self, density: Real) {
        self.do_set_mass_properties(ColliderMassProps::Density(density));
    }

    /// Sets the mass of this collider.
    ///
    /// This will override any previous mass-properties set by [`Self::set_density`],
    /// [`Self::set_mass`], [`Self::set_mass_properties`], [`ColliderBuilder::density`],
    /// [`ColliderBuilder::mass`], or [`ColliderBuilder::mass_properties`]
    /// for this collider.
    ///
    /// The angular inertia of this collider will be computed automatically based on its shape
    /// and this mass value.
    pub fn set_mass(&mut self, mass: Real) {
        self.do_set_mass_properties(ColliderMassProps::Mass(mass));
    }

    /// Sets the mass properties of this collider.
    ///
    /// This will override any previous mass-properties set by [`Self::set_density`],
    /// [`Self::set_mass`], [`Self::set_mass_properties`], [`ColliderBuilder::density`],
    /// [`ColliderBuilder::mass`], or [`ColliderBuilder::mass_properties`]
    /// for this collider.
    pub fn set_mass_properties(&mut self, mass_properties: MassProperties) {
        self.do_set_mass_properties(ColliderMassProps::MassProperties(Box::new(mass_properties)))
    }

    fn do_set_mass_properties(&mut self, mprops: ColliderMassProps) {
        if mprops != self.mprops {
            self.changes |= ColliderChanges::LOCAL_MASS_PROPERTIES;
            self.mprops = mprops;
        }
    }

    /// The geometric shape of this collider.
    pub fn shape(&self) -> &dyn Shape {
        self.shape.as_ref()
    }

    /// A mutable reference to the geometric shape of this collider.
    ///
    /// If that shape is shared by multiple colliders, it will be
    /// cloned first so that `self` contains a unique copy of that
    /// shape that you can modify.
    pub fn shape_mut(&mut self) -> &mut dyn Shape {
        self.changes.insert(ColliderChanges::SHAPE);
        self.shape.make_mut()
    }

    /// Sets the shape of this collider.
    pub fn set_shape(&mut self, shape: SharedShape) {
        self.changes.insert(ColliderChanges::SHAPE);
        self.shape = shape;
    }

    /// Retrieve the SharedShape. Also see the `shape()` function
    pub fn shared_shape(&self) -> &SharedShape {
        &self.shape
    }

    /// Compute the axis-aligned bounding box of this collider.
    pub fn compute_aabb(&self) -> Aabb {
        self.shape.compute_aabb(&self.pos)
    }

    /// Compute the axis-aligned bounding box of this collider moving from its current position
    /// to the given `next_position`
    pub fn compute_swept_aabb(&self, next_position: &Isometry<Real>) -> Aabb {
        self.shape.compute_swept_aabb(&self.pos, next_position)
    }

    /// Compute the local-space mass properties of this collider.
    pub fn mass_properties(&self) -> MassProperties {
        self.mprops.mass_properties(&*self.shape)
    }

    /// The total force magnitude beyond which a contact force event can be emitted.
    pub fn contact_force_event_threshold(&self) -> Real {
        self.contact_force_event_threshold
    }
}

/// A structure responsible for building a new collider.
#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[must_use = "Builder functions return the updated builder"]
pub struct ColliderBuilder {
    /// The shape of the collider to be built.
    pub shape: SharedShape,
    /// Controls the way the collider’s mass-properties are computed.
    pub mass_properties: ColliderMassProps,
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
    /// Will the collider being built be enabled?
    pub enabled: bool,
    /// The total force magnitude beyond which a contact force event can be emitted.
    pub contact_force_event_threshold: Real,
}

impl ColliderBuilder {
    /// Initialize a new collider builder with the given shape.
    pub fn new(shape: SharedShape) -> Self {
        Self {
            shape,
            mass_properties: ColliderMassProps::default(),
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
            enabled: true,
            contact_force_event_threshold: 0.0,
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
        Self::new(SharedShape::capsule_x(half_height, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `y` axis.
    pub fn capsule_y(half_height: Real, radius: Real) -> Self {
        Self::new(SharedShape::capsule_y(half_height, radius))
    }

    /// Initialize a new collider builder with a capsule shape aligned with the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn capsule_z(half_height: Real, radius: Real) -> Self {
        Self::new(SharedShape::capsule_z(half_height, radius))
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

    /// Initializes a collider builder with a triangle mesh shape defined by its vertex and index buffers and
    /// flags controlling its pre-processing.
    pub fn trimesh_with_flags(
        vertices: Vec<Point<Real>>,
        indices: Vec<[u32; 3]>,
        flags: TriMeshFlags,
    ) -> Self {
        Self::new(SharedShape::trimesh_with_flags(vertices, indices, flags))
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
    /// This will be overridden by a call to [`Self::mass`] or [`Self::mass_properties`] so it only
    /// makes sense to call either [`Self::density`] or [`Self::mass`] or [`Self::mass_properties`].
    ///
    /// The mass and angular inertia of this collider will be computed automatically based on its
    /// shape.
    pub fn density(mut self, density: Real) -> Self {
        self.mass_properties = ColliderMassProps::Density(density);
        self
    }

    /// Sets the mass of the collider this builder will build.
    ///
    /// This will be overridden by a call to [`Self::density`] or [`Self::mass_properties`] so it only
    /// makes sense to call either [`Self::density`] or [`Self::mass`] or [`Self::mass_properties`].
    ///
    /// The angular inertia of this collider will be computed automatically based on its shape
    /// and this mass value.
    pub fn mass(mut self, mass: Real) -> Self {
        self.mass_properties = ColliderMassProps::Mass(mass);
        self
    }

    /// Sets the mass properties of the collider this builder will build.
    ///
    /// This will be overridden by a call to [`Self::density`] or [`Self::mass`] so it only
    /// makes sense to call either [`Self::density`] or [`Self::mass`] or [`Self::mass_properties`].
    pub fn mass_properties(mut self, mass_properties: MassProperties) -> Self {
        self.mass_properties = ColliderMassProps::MassProperties(Box::new(mass_properties));
        self
    }

    /// Sets the total force magnitude beyond which a contact force event can be emitted.
    pub fn contact_force_event_threshold(mut self, threshold: Real) -> Self {
        self.contact_force_event_threshold = threshold;
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

    /// Enable or disable the collider after its creation.
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Builds a new collider attached to the given rigid-body.
    pub fn build(&self) -> Collider {
        let shape = self.shape.clone();
        let material = ColliderMaterial {
            friction: self.friction,
            restitution: self.restitution,
            friction_combine_rule: self.friction_combine_rule,
            restitution_combine_rule: self.restitution_combine_rule,
        };
        let flags = ColliderFlags {
            collision_groups: self.collision_groups,
            solver_groups: self.solver_groups,
            active_collision_types: self.active_collision_types,
            active_hooks: self.active_hooks,
            active_events: self.active_events,
            enabled: if self.enabled {
                ColliderEnabled::Enabled
            } else {
                ColliderEnabled::Disabled
            },
        };
        let changes = ColliderChanges::all();
        let pos = ColliderPosition(self.position);
        let bf_data = ColliderBroadPhaseData::default();
        let coll_type = if self.is_sensor {
            ColliderType::Sensor
        } else {
            ColliderType::Solid
        };

        Collider {
            shape,
            mprops: self.mass_properties.clone(),
            material,
            parent: None,
            changes,
            pos,
            bf_data,
            flags,
            coll_type,
            contact_force_event_threshold: self.contact_force_event_threshold,
            user_data: self.user_data,
        }
    }
}

impl Into<Collider> for ColliderBuilder {
    fn into(self) -> Collider {
        self.build()
    }
}
