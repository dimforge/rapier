use crate::math::Real;

/// How to combine friction/restitution values when two colliders touch.
///
/// When two colliders with different friction (or restitution) values collide, Rapier
/// needs to decide what the effective friction/restitution should be. Each collider has
/// a combine rule, and the "stronger" rule wins (Max > Multiply > Min > Average).
///
/// ## Combine Rules
///
/// **Most games use Average (the default)** and never change this.
///
/// - **Average** (default): `(friction1 + friction2) / 2` - Balanced, intuitive
/// - **Min**: `min(friction1, friction2)` - "Slippery wins" (ice on any surface = ice)
/// - **Multiply**: `friction1 × friction2` - Both must be high for high friction
/// - **Max**: `max(friction1, friction2)` - "Sticky wins" (rubber on any surface = rubber)
///
/// ## Example
/// ```ignore
/// // Ice collider that makes everything slippery
/// let ice = ColliderBuilder::cuboid(10.0, 0.1, 10.0)
///     .friction(0.0)
///     .friction_combine_rule(CoefficientCombineRule::Min)  // Ice wins!
///     .build();
/// ```
///
/// ## Priority System
/// If colliders disagree on rules, the "higher" one wins: Max > Multiply > Min > Average
#[derive(Default, Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub enum CoefficientCombineRule {
    /// Average the two values (default, most common).
    #[default]
    Average = 0,
    /// Use the smaller value ("slippery/soft wins").
    Min = 1,
    /// Multiply the two values (both must be high).
    Multiply = 2,
    /// Use the larger value ("sticky/bouncy wins").
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
