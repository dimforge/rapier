use std::fmt;

#[cfg(all(feature = "dim3", feature = "async-collider"))]
use {
    crate::geometry::{FillMode, MeshConverter, TriMeshFlags, VHACDParameters},
    crate::math::Real,
    bevy::platform::collections::HashMap,
};

use bevy::prelude::*;

use bevy::platform::collections::HashSet;
use rapier::geometry::Shape;
use rapier::prelude::{
    ColliderHandle, InteractionGroups, InteractionTestMode as RapierInteractionTestMode,
    SharedShape,
};

use crate::dynamics::{CoefficientCombineRule, MassProperties};
use crate::math::Vect;

#[cfg(doc)]
use rapier::{dynamics::RigidBody, geometry::ContactForceEvent};

/// The Rapier handle of a collider that was inserted to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierColliderHandle(pub ColliderHandle);

/// A component which will be replaced by the specified collider type after the referenced mesh become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Component, Debug, Clone, Default)]
pub struct AsyncCollider(pub ComputedColliderShape);

/// A component which will be replaced the specified collider types on children with meshes after the referenced scene become available.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Component, Debug, Clone)]
pub struct AsyncSceneCollider {
    /// Collider type for each scene mesh not included in [`Self::named_shapes`]. If [`None`], then all
    /// shapes will be skipped for processing except [`Self::named_shapes`].
    pub shape: Option<ComputedColliderShape>,
    /// Shape types for meshes by name. If shape is [`None`], then it will be skipped for
    /// processing.
    pub named_shapes: HashMap<String, Option<ComputedColliderShape>>,
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
impl Default for AsyncSceneCollider {
    fn default() -> Self {
        Self {
            shape: Some(Default::default()),
            named_shapes: Default::default(),
        }
    }
}

/// Shape type based on a Bevy mesh asset.
#[cfg(all(feature = "dim3", feature = "async-collider"))]
#[derive(Debug, Clone)]
pub enum ComputedColliderShape {
    /// Triangle-mesh.
    TriMesh(TriMeshFlags),
    /// Convex hull.
    ConvexHull,
    /// Convex decomposition.
    ConvexDecomposition(VHACDParameters),
    /// Voxelization of the mesh (see [`Collider::voxelized_mesh`]).
    Voxels {
        /// The size of each voxel along every axis.
        voxel_size: Real,
        /// Controls whether only the voxels intersecting the mesh boundary are kept, or if its
        /// interior is filled too.
        fill_mode: FillMode,
    },
    /// Shape obtained by converting the mesh with the given [`MeshConverter`] (see
    /// [`Collider::converted_trimesh`]).
    Converted(MeshConverter),
}

#[cfg(all(feature = "dim3", feature = "async-collider"))]
impl Default for ComputedColliderShape {
    fn default() -> Self {
        Self::TriMesh(TriMeshFlags::MERGE_DUPLICATE_VERTICES)
    }
}

/// A geometric entity that can be attached to a [`RigidBody`] so it can be affected by contacts
/// and intersection queries.
///
/// Related components:
/// - [`ColliderMassProperties`]
/// - [`ReadColliderMassProperties`]
/// - [`Friction`]
/// - [`Restitution`]
/// - [`Sensor`]
/// - [`CollisionGroups`]
/// - [`SolverGroups`]
/// - [`ActiveCollisionTypes`]
/// - [`ActiveEvents`]
/// - [`ContactForceEventThreshold`]
/// - [`CollidingEntities`]
/// - [`ColliderScale`]
/// - [`ColliderDisabled`]
#[derive(Component, Clone)] // TODO: Reflect
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[require(ReadColliderMassProperties)]
pub struct Collider {
    /// The raw shape from Rapier.
    pub raw: SharedShape,
    pub(crate) unscaled: SharedShape,
    pub(crate) scale: Vect,
}

impl From<SharedShape> for Collider {
    fn from(shared_shape: SharedShape) -> Collider {
        Collider {
            raw: shared_shape.clone(),
            unscaled: shared_shape,
            scale: Vect::ONE,
        }
    }
}

impl<'a> From<&'a Collider> for &'a dyn Shape {
    fn from(collider: &'a Collider) -> &'a dyn Shape {
        &*collider.raw
    }
}

/// A shape accepted by the scene queries (e.g. [`RapierContext::cast_shape`]) and by
/// [`RapierContextMut::move_shape`]: a [`Collider`] (its scaled shape), a Rapier shape trait object
/// (`&dyn Shape`, e.g. `&*shared_shape`), or any concrete Rapier shape (e.g. `Ball`).
///
/// [`RapierContext::cast_shape`]: crate::plugin::context::systemparams::RapierContext::cast_shape
/// [`RapierContextMut::move_shape`]: crate::plugin::context::systemparams::RapierContextMut::move_shape
pub trait AsShape {
    /// The shape used by the query.
    fn as_shape(&self) -> &dyn Shape;
}

impl AsShape for Collider {
    fn as_shape(&self) -> &dyn Shape {
        &*self.raw
    }
}

impl AsShape for dyn Shape {
    fn as_shape(&self) -> &dyn Shape {
        self
    }
}

impl<S: Shape> AsShape for S {
    fn as_shape(&self) -> &dyn Shape {
        self
    }
}

impl fmt::Debug for Collider {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.as_typed_shape().fmt(f)
    }
}

/// Overwrites the default application of [`GlobalTransform`] scale to a [`Collider`]'s shapes.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
pub enum ColliderScale {
    /// This scale will be multiplied with the scale in the [`GlobalTransform`] component
    /// before being applied to the collider.
    Relative(Vect),
    /// This scale will replace the one specified in the [`GlobalTransform`] component.
    Absolute(Vect),
}

/// Indicates whether or not the [`Collider`] is a sensor.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Sensor;

/// Custom mass-properties of a [`Collider`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub enum ColliderMassProperties {
    /// The mass-properties are computed automatically from the collider’s shape and this density.
    Density(f32),
    /// The mass-properties are computed automatically from the collider’s shape and this mass.
    Mass(f32),
    /// The mass-properties of the collider are replaced by the ones specified here.
    MassProperties(MassProperties),
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        Self::Density(1.0)
    }
}

/// The mass-related properties of a [`Collider`], as computed by Rapier.
///
/// This component is inserted automatically with [`Collider`], and updated whenever the collider
/// is created or its shape or [`ColliderMassProperties`] change. Modifying this component has no
/// effect on the simulation; modify [`ColliderMassProperties`] instead.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ReadColliderMassProperties {
    /// The volume (in 3D) or area (in 2D) of the collider's shape.
    pub volume: f32,
    /// The density of the collider.
    ///
    /// If the collider's mass was set explicitly, this is the mass divided by the volume.
    pub density: f32,
    /// The mass of the collider.
    pub mass: f32,
    /// The mass properties of the collider, expressed in the collider's local-space.
    pub local_mass_properties: MassProperties,
}

impl ReadColliderMassProperties {
    /// Extracts the mass-related properties of a Rapier collider.
    pub fn from_rapier(collider: &rapier::geometry::Collider) -> Self {
        Self {
            volume: collider.volume(),
            density: collider.density(),
            mass: collider.mass(),
            local_mass_properties: MassProperties::from_rapier(collider.mass_properties()),
        }
    }
}

/// The friction affecting a [`Collider`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Friction {
    /// The friction coefficient of a collider.
    ///
    /// The greater the value, the stronger the friction forces will be.
    /// Should be `>= 0`.
    pub coefficient: f32,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub combine_rule: CoefficientCombineRule,
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            coefficient: 0.5,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

impl Friction {
    /// Creates a [`Friction`] component from the given friction coefficient, and using the default
    /// [`CoefficientCombineRule::Average`] coefficient combine rule.
    pub const fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }

    /// Creates a [`Friction`] component from the given friction coefficient, and using the default
    /// [`CoefficientCombineRule::Average`] coefficient combine rule.
    pub const fn coefficient(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

/// The restitution affecting a [`Collider`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Restitution {
    /// The restitution coefficient of a collider.
    ///
    /// The greater the value, the stronger the restitution forces will be.
    /// Should be `>= 0`.
    pub coefficient: f32,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub combine_rule: CoefficientCombineRule,
}

impl Restitution {
    /// Creates a [`Restitution`] component from the given restitution coefficient, and using the default
    /// [`CoefficientCombineRule::Average`] coefficient combine rule.
    pub const fn new(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }

    /// Creates a [`Restitution`] component from the given restitution coefficient, and using the default
    /// [`CoefficientCombineRule::Average`] coefficient combine rule.
    pub const fn coefficient(coefficient: f32) -> Self {
        Self {
            coefficient,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

impl Default for Restitution {
    fn default() -> Self {
        Self {
            coefficient: 0.0,
            combine_rule: CoefficientCombineRule::Average,
        }
    }
}

#[derive(Component, Reflect, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
#[reflect(Component, Default, Hash, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Flags affecting whether or not collision-detection happens between two colliders
/// depending on the type of rigid-bodies they are attached to.
pub struct ActiveCollisionTypes(u16);

bitflags::bitflags! {
    impl ActiveCollisionTypes: u16 {
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a dynamic body.
        const DYNAMIC_DYNAMIC = 0b0000_0000_0000_0001;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a kinematic body.
        const DYNAMIC_KINEMATIC = 0b0000_0000_0000_1100;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const DYNAMIC_STATIC  = 0b0000_0000_0000_0010;
        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a kinematic body.
        const KINEMATIC_KINEMATIC = 0b1100_1100_0000_0000;

        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const KINEMATIC_STATIC = 0b0010_0010_0000_0000;

        /// Enable collision-detection between a collider attached to a fixed body (or
        /// not attached to any body) and another collider attached to a fixed body (or
        /// not attached to any body).
        const STATIC_STATIC = 0b0000_0000_0010_0000;
    }
}

impl Default for ActiveCollisionTypes {
    fn default() -> Self {
        Self::DYNAMIC_DYNAMIC | Self::DYNAMIC_KINEMATIC | Self::DYNAMIC_STATIC
    }
}

impl From<ActiveCollisionTypes> for rapier::geometry::ActiveCollisionTypes {
    fn from(collision_types: ActiveCollisionTypes) -> rapier::geometry::ActiveCollisionTypes {
        rapier::geometry::ActiveCollisionTypes::from_bits(collision_types.bits())
            .expect("Internal error: invalid active events conversion.")
    }
}

/// A bit mask identifying groups for interaction.
#[derive(Component, Reflect, Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[reflect(Component, Default, Hash, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct Group(u32);

bitflags::bitflags! {
    impl Group: u32 {
        /// The group n°1.
        const GROUP_1 = 1 << 0;
        /// The group n°2.
        const GROUP_2 = 1 << 1;
        /// The group n°3.
        const GROUP_3 = 1 << 2;
        /// The group n°4.
        const GROUP_4 = 1 << 3;
        /// The group n°5.
        const GROUP_5 = 1 << 4;
        /// The group n°6.
        const GROUP_6 = 1 << 5;
        /// The group n°7.
        const GROUP_7 = 1 << 6;
        /// The group n°8.
        const GROUP_8 = 1 << 7;
        /// The group n°9.
        const GROUP_9 = 1 << 8;
        /// The group n°10.
        const GROUP_10 = 1 << 9;
        /// The group n°11.
        const GROUP_11 = 1 << 10;
        /// The group n°12.
        const GROUP_12 = 1 << 11;
        /// The group n°13.
        const GROUP_13 = 1 << 12;
        /// The group n°14.
        const GROUP_14 = 1 << 13;
        /// The group n°15.
        const GROUP_15 = 1 << 14;
        /// The group n°16.
        const GROUP_16 = 1 << 15;
        /// The group n°17.
        const GROUP_17 = 1 << 16;
        /// The group n°18.
        const GROUP_18 = 1 << 17;
        /// The group n°19.
        const GROUP_19 = 1 << 18;
        /// The group n°20.
        const GROUP_20 = 1 << 19;
        /// The group n°21.
        const GROUP_21 = 1 << 20;
        /// The group n°22.
        const GROUP_22 = 1 << 21;
        /// The group n°23.
        const GROUP_23 = 1 << 22;
        /// The group n°24.
        const GROUP_24 = 1 << 23;
        /// The group n°25.
        const GROUP_25 = 1 << 24;
        /// The group n°26.
        const GROUP_26 = 1 << 25;
        /// The group n°27.
        const GROUP_27 = 1 << 26;
        /// The group n°28.
        const GROUP_28 = 1 << 27;
        /// The group n°29.
        const GROUP_29 = 1 << 28;
        /// The group n°30.
        const GROUP_30 = 1 << 29;
        /// The group n°31.
        const GROUP_31 = 1 << 30;
        /// The group n°32.
        const GROUP_32 = 1 << 31;

        /// All of the groups.
        const ALL = u32::MAX;
        /// None of the groups.
        const NONE = 0;
    }
}

impl Default for Group {
    fn default() -> Self {
        Group::ALL
    }
}

/// Specifies how the memberships and filters of two [`CollisionGroups`] (or two
/// [`SolverGroups`]) are tested against each other.
///
/// If the two groups being tested have different test modes, [`InteractionTestMode::And`]
/// takes priority.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[reflect(Default, Hash, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum InteractionTestMode {
    /// An interaction is allowed iff. the memberships of each group have at least one bit in
    /// common with the filters of the other group.
    #[default]
    And,
    /// An interaction is allowed iff. the memberships of either group have at least one bit in
    /// common with the filters of the other group.
    ///
    /// This only applies if both groups use [`InteractionTestMode::Or`]; otherwise
    /// [`InteractionTestMode::And`] is used.
    Or,
}

impl From<InteractionTestMode> for RapierInteractionTestMode {
    fn from(mode: InteractionTestMode) -> RapierInteractionTestMode {
        match mode {
            InteractionTestMode::And => RapierInteractionTestMode::And,
            InteractionTestMode::Or => RapierInteractionTestMode::Or,
        }
    }
}

impl From<RapierInteractionTestMode> for InteractionTestMode {
    fn from(mode: RapierInteractionTestMode) -> InteractionTestMode {
        match mode {
            RapierInteractionTestMode::And => InteractionTestMode::And,
            RapierInteractionTestMode::Or => InteractionTestMode::Or,
        }
    }
}

/// Pairwise collision filtering using bit masks.
///
/// This filtering method is based on two 32-bit values:
/// - The interaction groups memberships.
/// - The interaction groups filter.
///
/// With the default [`InteractionTestMode::And`], an interaction is allowed between two filters
/// `a` and `b` when two conditions are met simultaneously:
/// - The groups membership of `a` has at least one bit set to `1` in common with the groups filter of `b`.
/// - The groups membership of `b` has at least one bit set to `1` in common with the groups filter of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met:
/// ```ignore
/// (self.memberships & rhs.filter) != 0 && (rhs.memberships & self.filter) != 0
/// ```
///
/// If both filters use [`InteractionTestMode::Or`], meeting either condition is sufficient.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Component, Reflect)]
#[reflect(Component, Default, Hash, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct CollisionGroups {
    /// Groups memberships.
    pub memberships: Group,
    /// Groups filter.
    pub filters: Group,
    /// How the memberships and filters are tested against another [`CollisionGroups`].
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub test_mode: InteractionTestMode,
}

impl CollisionGroups {
    /// Creates a new collision-groups with the given membership masks and filter masks.
    ///
    /// The test mode is set to [`InteractionTestMode::And`].
    pub const fn new(memberships: Group, filters: Group) -> Self {
        Self {
            memberships,
            filters,
            test_mode: InteractionTestMode::And,
        }
    }

    /// Sets the test mode used to check interactions with another [`CollisionGroups`].
    pub const fn with_test_mode(mut self, test_mode: InteractionTestMode) -> Self {
        self.test_mode = test_mode;
        self
    }
}

impl From<CollisionGroups> for InteractionGroups {
    fn from(collision_groups: CollisionGroups) -> InteractionGroups {
        InteractionGroups {
            memberships: rapier::geometry::Group::from_bits(collision_groups.memberships.bits())
                .unwrap(),
            filter: rapier::geometry::Group::from_bits(collision_groups.filters.bits()).unwrap(),
            test_mode: collision_groups.test_mode.into(),
        }
    }
}

impl From<InteractionGroups> for CollisionGroups {
    fn from(groups: InteractionGroups) -> CollisionGroups {
        CollisionGroups {
            memberships: Group::from_bits_retain(groups.memberships.bits()),
            filters: Group::from_bits_retain(groups.filter.bits()),
            test_mode: groups.test_mode.into(),
        }
    }
}

/// Pairwise constraints resolution filtering using bit masks.
///
/// This follows the same rules as the `CollisionGroups`.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Hash, Component, Reflect)]
#[reflect(Component, Default, Hash, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SolverGroups {
    /// Groups memberships.
    pub memberships: Group,
    /// Groups filter.
    pub filters: Group,
    /// How the memberships and filters are tested against another [`SolverGroups`].
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    pub test_mode: InteractionTestMode,
}

impl SolverGroups {
    /// Creates a new solver-groups with the given membership masks and filter masks.
    ///
    /// The test mode is set to [`InteractionTestMode::And`].
    pub const fn new(memberships: Group, filters: Group) -> Self {
        Self {
            memberships,
            filters,
            test_mode: InteractionTestMode::And,
        }
    }

    /// Sets the test mode used to check interactions with another [`SolverGroups`].
    pub const fn with_test_mode(mut self, test_mode: InteractionTestMode) -> Self {
        self.test_mode = test_mode;
        self
    }
}

impl From<SolverGroups> for InteractionGroups {
    fn from(solver_groups: SolverGroups) -> InteractionGroups {
        InteractionGroups {
            memberships: rapier::geometry::Group::from_bits(solver_groups.memberships.bits())
                .unwrap(),
            filter: rapier::geometry::Group::from_bits(solver_groups.filters.bits()).unwrap(),
            test_mode: solver_groups.test_mode.into(),
        }
    }
}

impl From<InteractionGroups> for SolverGroups {
    fn from(groups: InteractionGroups) -> SolverGroups {
        SolverGroups {
            memberships: Group::from_bits_retain(groups.memberships.bits()),
            filters: Group::from_bits_retain(groups.filter.bits()),
            test_mode: groups.test_mode.into(),
        }
    }
}

#[derive(Default, Component, Reflect, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Flags affecting the behavior of the constraints solver for a given contact manifold.
pub struct ActiveHooks(u32);

bitflags::bitflags! {
    impl ActiveHooks: u32 {
        /// If set, Rapier will call `PhysicsHooks::filter_contact_pair` whenever relevant.
        const FILTER_CONTACT_PAIRS = 0b0001;
        /// If set, Rapier will call `PhysicsHooks::filter_intersection_pair` whenever relevant.
        const FILTER_INTERSECTION_PAIR = 0b0010;
        /// If set, Rapier will call `PhysicsHooks::modify_solver_contact` whenever relevant.
        const MODIFY_SOLVER_CONTACTS = 0b0100;
    }
}

impl From<ActiveHooks> for rapier::pipeline::ActiveHooks {
    fn from(active_hooks: ActiveHooks) -> rapier::pipeline::ActiveHooks {
        rapier::pipeline::ActiveHooks::from_bits(active_hooks.bits())
            .expect("Internal error: invalid active events conversion.")
    }
}

#[derive(Default, Component, Reflect, Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Flags affecting the events generated for this [`Collider`].
pub struct ActiveEvents(u32);

bitflags::bitflags! {
    impl ActiveEvents: u32 {
        /// If set, Rapier will call `EventHandler::handle_collision_event`
        /// whenever relevant for this [`Collider`].
        const COLLISION_EVENTS = 0b0001;
        /// If set, Rapier will call `EventHandler::handle_contact_force_event`
        /// whenever relevant for this [`Collider`].
        const CONTACT_FORCE_EVENTS = 0b0010;
    }
}

impl From<ActiveEvents> for rapier::pipeline::ActiveEvents {
    fn from(active_events: ActiveEvents) -> rapier::pipeline::ActiveEvents {
        rapier::pipeline::ActiveEvents::from_bits(active_events.bits())
            .expect("Internal error: invalid active events conversion.")
    }
}

/// The total force magnitude beyond which a [`ContactForceEvent`] can be emitted.
///
/// This requires that the [`ActiveEvents::CONTACT_FORCE_EVENTS`] flag is set on the
/// entity.
#[derive(Copy, Clone, PartialEq, Component, Reflect)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ContactForceEventThreshold(pub f32);

impl Default for ContactForceEventThreshold {
    fn default() -> Self {
        Self(f32::MAX)
    }
}

/// Sets the contact skin of the collider.
///
/// The contact skin acts as if the collider was enlarged with a skin of width `skin_thickness`
/// around it, keeping objects further apart when colliding.
///
/// A non-zero contact skin can increase performance, and in some cases, stability. However
/// it creates a small gap between colliding object (equal to the sum of their skin). If the
/// skin is sufficiently small, this might not be visually significant or can be hidden by the
/// rendering assets.
#[derive(Copy, Clone, PartialEq, Default, Component, Reflect)]
#[reflect(Component, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ContactSkin(pub f32);

/// Component which will be filled (if present) with a list of entities with which the current
/// entity is currently in contact.
///
/// This currently only updates when on an entity with a `Collider`, and if the
/// [`ActiveEvents::COLLISION_EVENTS`] is set on this entity or the entity it
/// collided with.
#[derive(Component, Default, Reflect)]
#[reflect(Component, Default)]
pub struct CollidingEntities(pub(crate) HashSet<Entity>);

impl CollidingEntities {
    /// Returns the number of colliding entities.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns `true` if there is no colliding entities.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns `true` if the collisions contains the specified entity.
    #[must_use]
    pub fn contains(&self, entity: Entity) -> bool {
        self.0.contains(&entity)
    }

    /// An iterator visiting all colliding entities in arbitrary order.
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }
}

/// Indicates whether or not the collider is disabled explicitly by the user.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ColliderDisabled;

/// We restrict the scaling increment to 1.0e-4, to avoid numerical jitter
/// due to the extraction of scaling factor from the GlobalTransform matrix.
pub fn get_snapped_scale(scale: Vect) -> Vect {
    fn snap_value(new: f32) -> f32 {
        const PRECISION: f32 = 1.0e4;
        (new * PRECISION).round() / PRECISION
    }

    Vect {
        x: snap_value(scale.x),
        y: snap_value(scale.y),
        #[cfg(feature = "dim3")]
        z: snap_value(scale.z),
    }
}
