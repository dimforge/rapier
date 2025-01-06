use crate::math::Real;

/// Rules used to combine two coefficients.
///
/// This is used to determine the effective restitution and
/// friction coefficients for a contact between two colliders.
/// Each collider has its combination rule of type
/// `CoefficientCombineRule`. And the rule
/// actually used is given by `max(first_combine_rule as usize, second_combine_rule as usize)`.
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    /// The two coefficients are averaged.
    #[default]
    Average = 0,
    /// The smallest coefficient is chosen.
    Min,
    /// The two coefficients are multiplied.
    Multiply,
    /// The greatest coefficient is chosen.
    Max,
    /// The sum of the two coefficients.
    Sum,
}

impl CoefficientCombineRule {
    pub(crate) fn combine(coeff1: Real, coeff2: Real, rule_value1: u8, rule_value2: u8) -> Real {
        let effective_rule = rule_value1.max(rule_value2);

        match effective_rule {
            0 => (coeff1 + coeff2) / 2.0,
            1 => {
                // Even though coeffs are meant to be positive, godot use-case has negative values.
                // We're following their logic here.
                // Context: https://github.com/dimforge/rapier/pull/741#discussion_r1862402948
                coeff1.min(coeff2).abs()
            }
            2 => coeff1 * coeff2,
            4 => (coeff1 + coeff2).clamp(0.0, 1.0),
            // 3 is missing as Max is the default one in case of mismatch.
            _ => coeff1.max(coeff2),
        }
    }
}
