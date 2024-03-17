use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle, RigidBodyType};
use crate::geometry::{InteractionGroups, SAPProxyIndex, Shape, SharedShape};
use crate::math::*;
use crate::parry::partitioning::IndexedData;
use crate::pipeline::{ActiveEvents, ActiveHooks};
use std::ops::{Deref, DerefMut};

use crate::data::Index;
#[cfg(feature = "bevy")]
use bevy::prelude::{Component, Reflect, ReflectComponent};

/// The unique identifier of a collider added to a collider set.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
#[cfg(not(feature = "bevy"))]
pub struct ColliderHandle(pub crate::data::arena::Index);

#[cfg(feature = "bevy")]
pub type ColliderHandle = bevy::prelude::Entity;

#[cfg(not(feature = "bevy"))]
impl ColliderHandle {
    pub const PLACEHOLDER: Self = Self(Index::from_raw_parts(
        crate::INVALID_U32,
        crate::INVALID_U32,
    ));

    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    pub fn index(&self) -> u32 {
        self.0.into_raw_parts().0
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid collider handle.
    pub fn invalid() -> Self {
        Self::PLACEHOLDER
    }
}

#[cfg(not(feature = "bevy"))]
impl From<crate::data::arena::Index> for ColliderHandle {
    fn from(value: Index) -> Self {
        Self(value)
    }
}

#[cfg(not(feature = "bevy"))]
impl From<ColliderHandle> for crate::data::arena::Index {
    fn from(value: ColliderHandle) -> Self {
        value.0
    }
}

#[cfg(not(feature = "bevy"))]
impl IndexedData for ColliderHandle {
    fn default() -> Self {
        Self(IndexedData::default())
    }

    fn index(&self) -> usize {
        self.0.index()
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags describing how the collider has been modified by the user.
    pub struct ColliderChanges: u32 {
        /// Flag indicating that any component of the collider has been modified.
        const MODIFIED = 1 << 0;
        /// Flag indicating that the density or mass-properties of this collider was changed.
        const LOCAL_MASS_PROPERTIES = 1 << 1; // => RigidBody local mass-properties update.
        /// Flag indicating that the `ColliderParent` component of the collider has been modified.
        const PARENT   = 1 << 2; // => BF & NF updates.
        /// Flag indicating that the `ColliderPosition` component of the collider has been modified.
        const POSITION = 1 << 3; // => BF & NF updates.
        /// Flag indicating that the collision groups of the collider have been modified.
        const GROUPS   = 1 << 4; // => NF update.
        /// Flag indicating that the `ColliderShape` component of the collider has been modified.
        const SHAPE    = 1 << 5; // => BF & NF update. NF pair workspace invalidation.
        /// Flag indicating that the `ColliderType` component of the collider has been modified.
        const TYPE     = 1 << 6; // => NF update. NF pair invalidation.
        /// Flag indicating that the dominance groups of the parent of this collider have been modified.
        ///
        /// This flags is automatically set by the `PhysicsPipeline` when the `RigidBodyChanges::DOMINANCE`
        /// or `RigidBodyChanges::TYPE` of the parent rigid-body of this collider is detected.
        const PARENT_EFFECTIVE_DOMINANCE = 1 << 7; // NF update.
        /// Flag indicating that whether or not the collider is enabled was changed.
        const ENABLED_OR_DISABLED = 1 << 8; // BF & NF updates.
    }
}

impl Default for ColliderChanges {
    fn default() -> Self {
        ColliderChanges::empty()
    }
}

impl ColliderChanges {
    /// Do these changes justify a broad-phase update?
    pub fn needs_broad_phase_update(self) -> bool {
        self.intersects(
            ColliderChanges::PARENT
                | ColliderChanges::POSITION
                | ColliderChanges::SHAPE
                | ColliderChanges::ENABLED_OR_DISABLED,
        )
    }

    /// Do these changes justify a narrow-phase update?
    pub fn needs_narrow_phase_update(self) -> bool {
        // NOTE: for simplicity of implementation, we return `true` even if
        //       we only need a dominance update. If this does become a
        //       bottleneck at some point in the future (which is very unlikely)
        //       we could do a special-case for dominance-only change (so that
        //       we only update the relative_dominance of the pre-existing contact.
        self.bits() > 2
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The type of collider.
pub enum ColliderType {
    /// A collider that can generate contacts and contact events.
    Solid,
    /// A collider that can generate intersection and intersection events.
    Sensor,
}

impl ColliderType {
    /// Is this collider a sensor?
    pub fn is_sensor(self) -> bool {
        self == ColliderType::Sensor
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Data associated to a collider that takes part to a broad-phase algorithm.
pub struct ColliderBroadPhaseData {
    pub(crate) proxy_index: SAPProxyIndex,
}

impl Default for ColliderBroadPhaseData {
    fn default() -> Self {
        ColliderBroadPhaseData {
            proxy_index: crate::INVALID_U32,
        }
    }
}

/// The shape of a collider.
pub type ColliderShape = SharedShape;

#[derive(Clone, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component),
    // TODO: Reflect doesn’t like Box?
    // derive(Component, Reflect),
    // reflect(Component, PartialEq)
)]
/// The mass-properties of a collider.
pub enum ColliderMassProperties {
    /// The collider is given a density.
    ///
    /// Its actual `MassProperties` are computed automatically with
    /// the help of [`Shape::mass_properties`].
    Density(Real),
    /// The collider is given a mass.
    ///
    /// Its angular inertia will be computed automatically based on this mass.
    Mass(Real),
    /// The collider is given explicit mass-properties.
    MassProperties(Box<MassProperties>),
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        ColliderMassProperties::Density(1.0)
    }
}

impl From<MassProperties> for ColliderMassProperties {
    fn from(mprops: MassProperties) -> Self {
        ColliderMassProperties::MassProperties(Box::new(mprops))
    }
}

impl ColliderMassProperties {
    /// The mass-properties of this collider.
    ///
    /// If `self` is the `Density` variant, then this computes the mass-properties based
    /// on the given shape.
    ///
    /// If `self` is the `MassProperties` variant, then this returns the stored mass-properties.
    pub fn mass_properties(&self, shape: &dyn Shape) -> MassProperties {
        match self {
            ColliderMassProperties::Density(density) => {
                if *density != 0.0 {
                    shape.mass_properties(*density)
                } else {
                    MassProperties::default()
                }
            }
            ColliderMassProperties::Mass(mass) => {
                if *mass != 0.0 {
                    let mut mprops = shape.mass_properties(1.0);
                    mprops.set_mass(*mass, true);
                    mprops
                } else {
                    MassProperties::default()
                }
            }
            ColliderMassProperties::MassProperties(mass_properties) => **mass_properties,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Information about the rigid-body this collider is attached to.
pub struct ColliderParent {
    /// Handle of the rigid-body this collider is attached to.
    pub handle: RigidBodyHandle,
    /// Const position of this collider relative to its parent rigid-body.
    pub pos_wrt_parent: Isometry,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The position of a collider.
pub struct ColliderPosition(pub Isometry);

impl AsRef<Isometry> for ColliderPosition {
    #[inline]
    fn as_ref(&self) -> &Isometry {
        &self.0
    }
}

impl AsMut<Isometry> for ColliderPosition {
    fn as_mut(&mut self) -> &mut Isometry {
        &mut self.0
    }
}

impl Deref for ColliderPosition {
    type Target = Isometry;
    #[inline]
    fn deref(&self) -> &Isometry {
        &self.0
    }
}

impl DerefMut for ColliderPosition {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Default for ColliderPosition {
    fn default() -> Self {
        Self::identity()
    }
}

impl ColliderPosition {
    /// The identity position.
    #[must_use]
    pub fn identity() -> Self {
        ColliderPosition(Isometry::identity())
    }
}

impl<T> From<T> for ColliderPosition
where
    Isometry: From<T>,
{
    fn from(position: T) -> Self {
        Self(position.into())
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
/// The collider’s friction properties.
pub struct Friction {
    /// The friction coefficient of this collider.
    ///
    /// The greater the value, the stronger the friction forces will be.
    /// Should be `>= 0`.
    pub coefficient: Real,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub combine_rule: CoefficientCombineRule,
}

impl Default for Friction {
    fn default() -> Self {
        Self {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::default(),
        }
    }
}

impl Friction {
    /// Inits the Friction component with the specified friction coefficient
    /// and the default friction [`CoefficientCombineRule`].
    pub fn coefficient(coefficient: Real) -> Self {
        Self {
            coefficient,
            ..Default::default()
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
/// The collider’s restitution properties.
pub struct Restitution {
    /// The restitution coefficient of this collider.
    ///
    /// Increase this value to make contacts with this collider more "bouncy".
    /// Should be `>= 0` and should generally not be greater than `1` (perfectly elastic
    /// collision).
    pub coefficient: Real,
    /// The rule applied to combine the restitution coefficients of two colliders.
    pub combine_rule: CoefficientCombineRule,
}

impl Default for Restitution {
    fn default() -> Self {
        Self {
            coefficient: 1.0,
            combine_rule: CoefficientCombineRule::default(),
        }
    }
}

impl Restitution {
    /// Inits the Restitution component with the specified friction coefficient
    /// and the default friction [`CoefficientCombineRule`].
    pub fn coefficient(coefficient: Real) -> Self {
        Self {
            coefficient,
            ..Default::default()
        }
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    #[cfg_attr(
        feature = "bevy",
        derive(Component, Reflect),
        reflect(Component, Hash, PartialEq)
    )]
    /// Flags affecting whether or not collision-detection happens between two colliders
    /// depending on the type of rigid-bodies they are attached to.
    pub struct ActiveCollisionTypes: u16 {
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a dynamic body.
        const DYNAMIC_DYNAMIC = 0b0000_0000_0000_0001;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a kinematic body.
        const DYNAMIC_KINEMATIC = 0b0000_0000_0000_1100;
        /// Enable collision-detection between a collider attached to a dynamic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const DYNAMIC_FIXED  = 0b0000_0000_0000_0010;
        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a kinematic body.
        const KINEMATIC_KINEMATIC = 0b1100_1100_0000_0000;

        /// Enable collision-detection between a collider attached to a kinematic body
        /// and another collider attached to a fixed body (or not attached to any body).
        const KINEMATIC_FIXED = 0b0010_0010_0000_0000;

        /// Enable collision-detection between a collider attached to a fixed body (or
        /// not attached to any body) and another collider attached to a fixed body (or
        /// not attached to any body).
        const FIXED_FIXED = 0b0000_0000_0010_0000;
    }
}

impl ActiveCollisionTypes {
    /// Test whether contact should be computed between two rigid-bodies with the given types.
    pub fn test(self, rb_type1: RigidBodyType, rb_type2: RigidBodyType) -> bool {
        // NOTE: This test is quite complicated so here is an explanation.
        //       First, we associate the following bit masks:
        //           - DYNAMIC = 0001
        //           - FIXED = 0010
        //           - KINEMATIC = 1100
        //       These are equal to the bits indexed by `RigidBodyType as u32`.
        //       The bit masks defined by ActiveCollisionTypes are defined is such a way
        //       that the first part of the variant name (e.g. DYNAMIC_*) indicates which
        //       groups of four bits should be considered:
        //           - DYNAMIC_* = the first group of four bits.
        //           - FIXED_* = the second group of four bits.
        //           - KINEMATIC_* = the third and fourth groups of four bits.
        //       The second part of the variant name (e.g. *_DYNAMIC) indicates the value
        //       of the aforementioned groups of four bits.
        //       For example, DYNAMIC_FIXED means that the first group of four bits (because
        //       of DYNAMIC_*) must have the value 0010 (because of *_FIXED). That gives
        //       us 0b0000_0000_0000_0010 for the DYNAMIC_FIXED_VARIANT.
        //
        //       The KINEMATIC_* is special because it occupies two groups of four bits. This is
        //       because it combines both KinematicPositionBased and KinematicVelocityBased.
        //
        //       Now that we have a way of building these bit masks, let's see how we use them.
        //       Given a pair of rigid-body types, the first rigid-body type is used to select
        //       the group of four bits we want to test (the selection is done by to the
        //       `>> (rb_type1 as u32 * 4) & 0b0000_1111`) and the second rigid-body type is
        //       used to form the bit mask we test this group of four bits against.
        //       In other word, the selection of the group of four bits tells us "for this type
        //       of rigid-body I can have collision with rigid-body types with these bit representation".
        //       Then the `(1 << rb_type2)` gives us the bit-representation of the rigid-body type,
        //       which needs to be checked.
        //
        //       Because that test must be symmetric, we perform two similar tests by swapping
        //       rb_type1 and rb_type2.
        ((self.bits >> (rb_type1 as u32 * 4)) & 0b0000_1111) & (1 << rb_type2 as u32) != 0
            || ((self.bits >> (rb_type2 as u32 * 4)) & 0b0000_1111) & (1 << rb_type1 as u32) != 0
    }
}

impl Default for ActiveCollisionTypes {
    fn default() -> Self {
        ActiveCollisionTypes::DYNAMIC_DYNAMIC
            | ActiveCollisionTypes::DYNAMIC_KINEMATIC
            | ActiveCollisionTypes::DYNAMIC_FIXED
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Enum indicating whether or not a collider is enabled.
pub enum ColliderEnabled {
    /// The collider is enabled.
    Enabled,
    /// The collider wasn’t disabled by the user explicitly but it is attached to
    /// a disabled rigid-body.
    DisabledByParent,
    /// The collider is disabled by the user explicitly.
    Disabled,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A set of flags for controlling collision/intersection filtering, modification, and events.
pub struct ColliderFlags {
    /// Controls whether collision-detection happens between two colliders depending on
    /// the type of the rigid-bodies they are attached to.
    pub active_collision_types: ActiveCollisionTypes,
    /// The groups controlling the pairs of colliders that can interact (generate
    /// interaction events or contacts).
    pub collision_groups: InteractionGroups,
    /// The groups controlling the pairs of collider that have their contact
    /// points taken into account for force computation.
    pub solver_groups: InteractionGroups,
    /// The physics hooks enabled for contact pairs and intersection pairs involving this collider.
    pub active_hooks: ActiveHooks,
    /// The events enabled for this collider.
    pub active_events: ActiveEvents,
    /// Whether or not the collider is enabled.
    pub enabled: ColliderEnabled,
}

impl Default for ColliderFlags {
    fn default() -> Self {
        Self {
            active_collision_types: ActiveCollisionTypes::default(),
            collision_groups: InteractionGroups::all(),
            solver_groups: InteractionGroups::all(),
            active_hooks: ActiveHooks::empty(),
            active_events: ActiveEvents::empty(),
            enabled: ColliderEnabled::Enabled,
        }
    }
}

impl From<ActiveHooks> for ColliderFlags {
    fn from(active_hooks: ActiveHooks) -> Self {
        Self {
            active_hooks,
            ..Default::default()
        }
    }
}

impl From<ActiveEvents> for ColliderFlags {
    fn from(active_events: ActiveEvents) -> Self {
        Self {
            active_events,
            ..Default::default()
        }
    }
}
