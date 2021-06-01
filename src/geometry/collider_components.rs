use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle};
use crate::geometry::{InteractionGroups, SAPProxyIndex, Shape, SharedShape};
use crate::math::{Isometry, Real};
use crate::parry::partitioning::IndexedData;
use crate::pipeline::{ActiveEvents, ActiveHooks};
use std::ops::Deref;

/// The unique identifier of a collider added to a collider set.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
pub struct ColliderHandle(pub crate::data::arena::Index);

impl ColliderHandle {
    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid collider handle.
    pub fn invalid() -> Self {
        Self(crate::data::arena::Index::from_raw_parts(
            crate::INVALID_U32,
            crate::INVALID_U32,
        ))
    }
}

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
        /// Flag indicating that the `RigidBodyParent` component of the collider has been modified.
        const PARENT   = 1 << 1; // => BF & NF updates.
        /// Flag indicating that the `RigidBodyPosition` component of the collider has been modified.
        const POSITION = 1 << 2; // => BF & NF updates.
        /// Flag indicating that the `RigidBodyGroups` component of the collider has been modified.
        const GROUPS   = 1 << 3; // => NF update.
        /// Flag indicating that the `RigidBodyShape` component of the collider has been modified.
        const SHAPE    = 1 << 4; // => BF & NF update. NF pair workspace invalidation.
        /// Flag indicating that the `RigidBodyType` component of the collider has been modified.
        const TYPE     = 1 << 5; // => NF update. NF pair invalidation.
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
            ColliderChanges::PARENT | ColliderChanges::POSITION | ColliderChanges::SHAPE,
        )
    }

    /// Do these changes justify a narrow-phase update?
    pub fn needs_narrow_phase_update(self) -> bool {
        self.bits() > 1
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

#[derive(Copy, Clone, Debug)]
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

#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The mass-properties of a collider.
pub enum ColliderMassProps {
    /// The collider is given a density.
    ///
    /// Its actual `MassProperties` are computed automatically with
    /// the help of [`SharedShape::mass_properties`].
    Density(Real),
    /// The collider is given explicit mass-properties.
    MassProperties(Box<MassProperties>),
}

impl Default for ColliderMassProps {
    fn default() -> Self {
        ColliderMassProps::Density(1.0)
    }
}

impl From<MassProperties> for ColliderMassProps {
    fn from(mprops: MassProperties) -> Self {
        ColliderMassProps::MassProperties(Box::new(mprops))
    }
}

impl ColliderMassProps {
    /// The mass-properties of this collider.
    ///
    /// If `self` is the `Density` variant, then this computes the mass-properties based
    /// on the given shape.
    ///
    /// If `self` is the `MassProperties` variant, then this returns the stored mass-properties.
    pub fn mass_properties(&self, shape: &dyn Shape) -> MassProperties {
        match self {
            Self::Density(density) => shape.mass_properties(*density),
            Self::MassProperties(mprops) => **mprops,
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Information about the rigid-body this collider is attached to.
pub struct ColliderParent {
    /// Handle of the rigid-body this collider is attached to.
    pub handle: RigidBodyHandle,
    /// Const position of this collider relative to its parent rigid-body.
    pub pos_wrt_parent: Isometry<Real>,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The position of a collider.
pub struct ColliderPosition(pub Isometry<Real>);

impl AsRef<Isometry<Real>> for ColliderPosition {
    #[inline]
    fn as_ref(&self) -> &Isometry<Real> {
        &self.0
    }
}

impl Deref for ColliderPosition {
    type Target = Isometry<Real>;
    #[inline]
    fn deref(&self) -> &Isometry<Real> {
        &self.0
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
    Isometry<Real>: From<T>,
{
    fn from(position: T) -> Self {
        Self(position.into())
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The groups of this collider, for filtering contact and solver pairs.
pub struct ColliderGroups {
    /// The groups controlling the pairs of colliders that can interact (generate
    /// interaction events or contacts).
    pub collision_groups: InteractionGroups,
    /// The groups controlling the pairs of collider that have their contact
    /// points taken into account for force computation.
    pub solver_groups: InteractionGroups,
}

impl Default for ColliderGroups {
    fn default() -> Self {
        Self {
            collision_groups: InteractionGroups::default(),
            solver_groups: InteractionGroups::default(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The constraints solver-related properties of this collider (friction, restitution, etc.)
pub struct ColliderMaterial {
    /// The friction coefficient of this collider.
    ///
    /// The greater the value, the stronger the friction forces will be.
    /// Should be `>= 0`.
    pub friction: Real,
    /// The restitution coefficient of this collider.
    ///
    /// Increase this value to make contacts with this collider more "bouncy".
    /// Should be `>= 0` and should generally not be greater than `1` (perfectly elastic
    /// collision).
    pub restitution: Real,
    /// The rule applied to combine the friction coefficients of two colliders in contact.
    pub friction_combine_rule: CoefficientCombineRule,
    /// The rule applied to combine the restitution coefficients of two colliders.
    pub restitution_combine_rule: CoefficientCombineRule,
}

impl ColliderMaterial {
    /// Creates a new collider material with the given friction and restitution coefficients.
    pub fn new(friction: Real, restitution: Real) -> Self {
        Self {
            friction,
            restitution,
            ..Default::default()
        }
    }
}

impl Default for ColliderMaterial {
    fn default() -> Self {
        Self {
            friction: 1.0,
            restitution: 0.0,
            friction_combine_rule: CoefficientCombineRule::default(),
            restitution_combine_rule: CoefficientCombineRule::default(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A set of flags controlling the active hooks and events for this colliders.
pub struct ColliderFlags {
    /// The physics hooks enabled for contact pairs and intersection pairs involving this collider.
    pub active_hooks: ActiveHooks,
    /// The events enabled for this collider.
    pub active_events: ActiveEvents,
}

impl Default for ColliderFlags {
    fn default() -> Self {
        Self {
            active_hooks: ActiveHooks::empty(),
            active_events: ActiveEvents::empty(),
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
