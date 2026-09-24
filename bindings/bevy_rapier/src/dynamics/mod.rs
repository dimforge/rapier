pub use self::generic_joint::*;
pub use self::joint::*;
pub use self::multibody::*;
pub use self::rigid_body::*;
pub use self::soft_body::*;

pub use self::fixed_joint::*;
pub use self::prismatic_joint::*;
pub use self::revolute_joint::*;
pub use self::rope_joint::*;
pub use self::spring_joint::*;

use bevy::reflect::Reflect;
use rapier::dynamics::CoefficientCombineRule as RapierCoefficientCombineRule;

#[cfg(feature = "dim2")]
pub use self::pin_slot_joint::*;
#[cfg(feature = "dim3")]
pub use self::spherical_joint::*;

mod generic_joint;
mod joint;
mod multibody;
mod rigid_body;
pub mod soft_body;

mod fixed_joint;
mod prismatic_joint;
mod revolute_joint;
mod rope_joint;

#[cfg(feature = "dim2")]
mod pin_slot_joint;
#[cfg(feature = "dim3")]
mod spherical_joint;
mod spring_joint;

/// Rules used to combine two coefficients.
///
/// This is used to determine the effective restitution and
/// friction coefficients for a contact between two colliders.
/// Each collider has its combination rule of type
/// `CoefficientCombineRule`. And the rule
/// actually used is given by `max(first_combine_rule as usize, second_combine_rule as usize)`,
/// i.e., `GeometricMean > ClampedSum > Max > Multiply > Min > Average`.
///
/// It is set by the [`Friction`] and [`Restitution`] components of colliders, and applies to
/// every contact between colliders, whether they are attached to a rigid-body or not.
///
/// [`Friction`]: crate::geometry::Friction
/// [`Restitution`]: crate::geometry::Restitution
#[derive(Copy, Clone, Debug, PartialEq, Eq, Reflect, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    #[default]
    /// The two coefficients are averaged.
    Average = 0,
    /// The smallest coefficient is chosen.
    Min,
    /// The two coefficients are multiplied.
    Multiply,
    /// The greatest coefficient is chosen.
    Max,
    /// The sum of the two coefficients, clamped to the range `[0, 1]`.
    ClampedSum,
    /// The square root of the product of the two coefficients.
    ///
    /// This is zero if either coefficient is zero.
    GeometricMean,
}

impl CoefficientCombineRule {
    /// Combines two coefficients according to their respective combine rules.
    ///
    /// If the two rules differ, the one with the highest priority is used
    /// (`GeometricMean > ClampedSum > Max > Multiply > Min > Average`).
    pub fn combine(coeff1: f32, coeff2: f32, rule1: Self, rule2: Self) -> f32 {
        RapierCoefficientCombineRule::combine(coeff1, coeff2, rule1.into(), rule2.into())
    }
}

impl From<CoefficientCombineRule> for RapierCoefficientCombineRule {
    fn from(combine_rule: CoefficientCombineRule) -> RapierCoefficientCombineRule {
        match combine_rule {
            CoefficientCombineRule::Average => RapierCoefficientCombineRule::Average,
            CoefficientCombineRule::Min => RapierCoefficientCombineRule::Min,
            CoefficientCombineRule::Multiply => RapierCoefficientCombineRule::Multiply,
            CoefficientCombineRule::Max => RapierCoefficientCombineRule::Max,
            CoefficientCombineRule::ClampedSum => RapierCoefficientCombineRule::ClampedSum,
            CoefficientCombineRule::GeometricMean => RapierCoefficientCombineRule::GeometricMean,
        }
    }
}

impl From<RapierCoefficientCombineRule> for CoefficientCombineRule {
    fn from(combine_rule: RapierCoefficientCombineRule) -> CoefficientCombineRule {
        match combine_rule {
            RapierCoefficientCombineRule::Average => CoefficientCombineRule::Average,
            RapierCoefficientCombineRule::Min => CoefficientCombineRule::Min,
            RapierCoefficientCombineRule::Multiply => CoefficientCombineRule::Multiply,
            RapierCoefficientCombineRule::Max => CoefficientCombineRule::Max,
            RapierCoefficientCombineRule::ClampedSum => CoefficientCombineRule::ClampedSum,
            RapierCoefficientCombineRule::GeometricMean => CoefficientCombineRule::GeometricMean,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::CoefficientCombineRule as Rule;

    #[test]
    fn combine_rules() {
        assert_eq!(Rule::combine(0.2, 0.6, Rule::Average, Rule::Average), 0.4);
        assert_eq!(Rule::combine(0.2, 0.6, Rule::Min, Rule::Average), 0.2);
        assert_eq!(Rule::combine(0.5, 0.6, Rule::Multiply, Rule::Min), 0.3);
        assert_eq!(Rule::combine(0.2, 0.6, Rule::Max, Rule::Multiply), 0.6);
        assert_eq!(Rule::combine(0.7, 0.6, Rule::ClampedSum, Rule::Max), 1.0);
        assert_eq!(
            Rule::combine(0.2, 0.3, Rule::Average, Rule::ClampedSum),
            0.5
        );
        assert_eq!(
            Rule::combine(0.25, 1.0, Rule::ClampedSum, Rule::GeometricMean),
            0.5
        );
        assert_eq!(
            Rule::combine(0.0, 0.8, Rule::GeometricMean, Rule::Average),
            0.0
        );
    }

    #[test]
    fn combine_rule_conversions_round_trip() {
        for rule in [
            Rule::Average,
            Rule::Min,
            Rule::Multiply,
            Rule::Max,
            Rule::ClampedSum,
            Rule::GeometricMean,
        ] {
            let raw: rapier::dynamics::CoefficientCombineRule = rule.into();
            assert_eq!(raw as u32, rule as u32);
            assert_eq!(Rule::from(raw), rule);
        }
    }
}
