use crate::math::Real;

/// Rules used to combine two coefficients.
///
/// This is used to determine the effective restitution and
/// friction coefficients for a contact between two colliders.
/// Each collider has its combination rule of type
/// `CoefficientCombineRule`. And the rule
/// actually used is given by `max(first_combine_rule as usize, second_combine_rule as usize)`.
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    /// The two coefficients are averaged.
    #[default]
    Average = 0,
    /// The smallest coefficient is chosen.
    Min = 1,
    /// The two coefficients are multiplied.
    Multiply = 2,
    /// The greatest coefficient is chosen.
    Max = 3,
}

impl CoefficientCombineRule {
    pub(crate) fn combine(
        coeff1: Real,
        coeff2: Real,
        rule_value1: CoefficientCombineRule,
        rule_value2: CoefficientCombineRule,
    ) -> Real {
        let effective_rule = rule_value1.max(rule_value2);

        match effective_rule {
            CoefficientCombineRule::Average => (coeff1 + coeff2) / 2.0,
            CoefficientCombineRule::Min => coeff1.min(coeff2),
            CoefficientCombineRule::Multiply => coeff1 * coeff2,
            CoefficientCombineRule::Max => coeff1.max(coeff2),
        }
    }
}
