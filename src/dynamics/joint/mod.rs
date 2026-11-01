pub use self::fixed_joint::*;
pub use self::generic_joint::*;
pub use self::impulse_joint::*;
pub use self::motor_model::MotorModel;
pub use self::multibody_joint::*;
pub use self::pin_slot_joint::*;
pub use self::prismatic_joint::*;
pub use self::revolute_joint::*;
pub use self::rope_joint::*;
pub use self::spring_joint::*;

#[cfg(feature = "dim3")]
pub use self::spherical_joint::*;

mod fixed_joint;
mod generic_joint;
mod impulse_joint;
mod motor_model;
mod multibody_joint;
mod pin_slot_joint;
mod prismatic_joint;
mod revolute_joint;
mod rope_joint;

#[cfg(feature = "dim3")]
mod spherical_joint;
mod spring_joint;

#[cfg(feature = "alloc")]
pub(crate) use self::pending::{sorted_joins, sorted_wake_ups};

/// The joint sets' pending wake-ups and island joins, in a canonical order.
#[cfg(feature = "alloc")]
mod pending {
    use crate::alloc_prelude::*;
    use crate::dynamics::RigidBodyHandle;
    use parry::utils::hashset::HashSet;

    /// The bodies a joint set wakes up at the next step, in handle order (the order snapshots
    /// store and steps process them in).
    pub(crate) fn sorted_wake_ups(set: &HashSet<RigidBodyHandle>) -> Vec<RigidBodyHandle> {
        let mut handles: Vec<_> = set.iter().copied().collect();
        handles.sort_unstable_by_key(|h| h.into_raw_parts());
        handles
    }

    /// The body pairs whose islands a joint set joins at the next step, in handle order.
    pub(crate) fn sorted_joins(
        set: &HashSet<(RigidBodyHandle, RigidBodyHandle)>,
    ) -> Vec<(RigidBodyHandle, RigidBodyHandle)> {
        let mut pairs: Vec<_> = set.iter().copied().collect();
        pairs.sort_unstable_by_key(|(a, b)| (a.into_raw_parts(), b.into_raw_parts()));
        pairs
    }

    #[cfg(feature = "serde-serialize")]
    pub(crate) fn serialize_sorted_wake_ups<S: serde::Serializer>(
        set: &HashSet<RigidBodyHandle>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        serde::Serialize::serialize(&sorted_wake_ups(set), s)
    }

    #[cfg(feature = "serde-serialize")]
    pub(crate) fn serialize_sorted_joins<S: serde::Serializer>(
        set: &HashSet<(RigidBodyHandle, RigidBodyHandle)>,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        serde::Serialize::serialize(&sorted_joins(set), s)
    }
}
