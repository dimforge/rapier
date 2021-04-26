use crate::math::Real;

/// Rules used to combine two coefficients.
///
/// This is used to determine the effective restitution and
/// friction coefficients for a contact between two colliders.
/// Each collider has its combination rule of type
/// `CoefficientCombineRule`. And the rule
/// actually used is given by `max(first_combine_rule as usize, second_combine_rule as usize)`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    /// The two coefficients are averaged.
    Average = 0,
    /// The smallest coefficient is chosen.
    Min,
    /// The two coefficients are multiplied.
    Multiply,
    /// The greatest coefficient is chosen.
    Max,
}

impl Default for CoefficientCombineRule {
    fn default() -> Self {
        CoefficientCombineRule::Average
    }
}

impl CoefficientCombineRule {
    pub(crate) fn combine(coeff1: Real, coeff2: Real, rule_value1: u8, rule_value2: u8) -> Real {
        let effective_rule = rule_value1.max(rule_value2);

        match effective_rule {
            0 => (coeff1 + coeff2) / 2.0,
            1 => coeff1.min(coeff2),
            2 => coeff1 * coeff2,
            _ => coeff1.max(coeff2),
        }
    }
}
