//! Structures related to geometry: colliders, shapes, etc.

pub use self::broad_phase::*;
pub use self::collider_set::*;
pub use self::contact::*;
pub use self::feature::*;
pub use self::narrow_phase::*;
pub use self::point::*;
pub use self::ray::*;
pub use self::shape::*;
pub use self::toi::*;

mod broad_phase;
mod collider;
mod collider_set;
mod contact;
mod feature;
mod narrow_phase;
mod point;
mod ray;
mod shape;
mod toi;

use rapier::dynamics::CoefficientCombineRule;
use rapier::geometry::InteractionGroups;
use rapier::prelude::Group;

pub const fn unpack_interaction_groups(memberships_filter: u32) -> InteractionGroups {
    InteractionGroups::new(
        Group::from_bits_retain((memberships_filter >> 16) as u32),
        Group::from_bits_retain((memberships_filter & 0x0000_ffff) as u32),
    )
}

pub const fn pack_interaction_groups(groups: InteractionGroups) -> u32 {
    (groups.memberships.bits() << 16) | groups.filter.bits()
}

pub const fn combine_rule_from_u32(rule: u32) -> CoefficientCombineRule {
    if rule == CoefficientCombineRule::Average as u32 {
        CoefficientCombineRule::Average
    } else if rule == CoefficientCombineRule::Min as u32 {
        CoefficientCombineRule::Min
    } else if rule == CoefficientCombineRule::Multiply as u32 {
        CoefficientCombineRule::Multiply
    } else {
        CoefficientCombineRule::Max
    }
}
