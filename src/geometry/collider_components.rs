use crate::dynamics::{CoefficientCombineRule, MassProperties, RigidBodyHandle};
use crate::geometry::{InteractionGroups, SAPProxyIndex, Shape, SharedShape, SolverFlags};
use crate::math::{Isometry, Real};
use crate::parry::partitioning::IndexedData;
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
        const MODIFIED             = 1 << 0;
        const PARENT               = 1 << 1; // => BF & NF updates.
        const POSITION             = 1 << 2; // => BF & NF updates.
        const GROUPS               = 1 << 3; // => NF update.
        const SHAPE                = 1 << 4; // => BF & NF update. NF pair workspace invalidation.
        const TYPE                 = 1 << 5; // => NF update. NF pair invalidation.
    }
}

impl Default for ColliderChanges {
    fn default() -> Self {
        ColliderChanges::empty()
    }
}

impl ColliderChanges {
    pub fn needs_broad_phase_update(self) -> bool {
        self.intersects(
            ColliderChanges::PARENT | ColliderChanges::POSITION | ColliderChanges::SHAPE,
        )
    }

    pub fn needs_narrow_phase_update(self) -> bool {
        self.bits() > 1
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum ColliderType {
    Solid,
    Sensor,
}

impl ColliderType {
    pub fn is_sensor(self) -> bool {
        self == ColliderType::Sensor
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
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

pub type ColliderShape = SharedShape;

#[derive(Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum ColliderMassProperties {
    /// `MassProperties` are computed with the help of [`SharedShape::mass_properties`].
    Density(Real),
    MassProperties(Box<MassProperties>),
}

impl Default for ColliderMassProperties {
    fn default() -> Self {
        ColliderMassProperties::Density(1.0)
    }
}

impl ColliderMassProperties {
    pub fn mass_properties(&self, shape: &dyn Shape) -> MassProperties {
        match self {
            Self::Density(density) => shape.mass_properties(*density),
            Self::MassProperties(mprops) => **mprops,
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ColliderParent {
    pub handle: RigidBodyHandle,
    pub pos_wrt_parent: Isometry<Real>,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
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
    #[must_use]
    fn identity() -> Self {
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
pub struct ColliderGroups {
    pub collision_groups: InteractionGroups,
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
pub struct ColliderMaterial {
    pub friction: Real,
    pub restitution: Real,
    pub friction_combine_rule: CoefficientCombineRule,
    pub restitution_combine_rule: CoefficientCombineRule,
    pub solver_flags: SolverFlags,
}

impl ColliderMaterial {
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
            solver_flags: SolverFlags::default(),
        }
    }
}
