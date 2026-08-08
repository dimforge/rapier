use crate::math::Real;
// Provides `sqrt` in no-std builds (same pattern as island_manager/local_split.rs).
#[allow(unused_imports)]
use simba::scalar::ComplexField as _;

/// How to combine friction/restitution values when two colliders touch.
///
/// When two colliders with different friction (or restitution) values collide, Rapier
/// needs to decide what the effective friction/restitution should be. Each collider has
/// a combine rule, and the "stronger" rule wins
/// (GeometricMean > ClampedSum > Max > Multiply > Min > Average).
///
/// ## Combine Rules
///
/// **Most games use Average (the default)** and never change this.
///
/// - **Average** (default): `(friction1 + friction2) / 2` - Balanced, intuitive
/// - **Min**: `min(friction1, friction2).abs()` - "Slippery wins" (ice on any surface = ice)
/// - **Multiply**: `friction1 × friction2` - Both must be high for high friction
/// - **Max**: `max(friction1, friction2)` - "Sticky wins" (rubber on any surface = rubber)
/// - **ClampedSum**: `sum(friction1, friction2).clamp(0, 1)` - Sum of both frictions, clamped to range 0, 1.
/// - **GeometricMean**: `sqrt(friction1 × friction2)` - Between Multiply and Average; zero if either is zero.
///
/// ## Example
/// ```
/// # use rapier3d::prelude::*;
/// // Ice collider that makes everything slippery
/// let ice = ColliderBuilder::cuboid(10.0, 0.1, 10.0)
///     .friction(0.0)
///     .friction_combine_rule(CoefficientCombineRule::Min)  // Ice wins!
///     .build();
/// ```
///
/// ## Priority System
/// If colliders disagree on rules, the "higher" one wins:
/// GeometricMean > ClampedSum > Max > Multiply > Min > Average
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
    /// The clamped sum of the two coefficients.
    ClampedSum = 4,
    /// The square root of the product of the two values.
    ///
    /// A common convention in other engines (e.g. Bullet, PhysX): stricter than Average
    /// (either value being zero results in zero) but less aggressive than Multiply for
    /// values below 1.
    GeometricMean = 5,
}

impl CoefficientCombineRule {
    #[allow(dead_code)]
    pub(crate) fn combine(
        coeff1: Real,
        coeff2: Real,
        rule_value1: CoefficientCombineRule,
        rule_value2: CoefficientCombineRule,
    ) -> Real {
        let effective_rule = rule_value1.max(rule_value2);

        match effective_rule {
            CoefficientCombineRule::Average => (coeff1 + coeff2) / 2.0,
            CoefficientCombineRule::Min => {
                // Even though coeffs are meant to be positive, godot use-case has negative values.
                // We're following their logic here.
                // Context: https://github.com/dimforge/rapier/pull/741#discussion_r1862402948
                coeff1.min(coeff2).abs()
            }
            CoefficientCombineRule::Multiply => coeff1 * coeff2,
            CoefficientCombineRule::Max => coeff1.max(coeff2),
            CoefficientCombineRule::ClampedSum => (coeff1 + coeff2).clamp(0.0, 1.0),
            // Negative coefficients are tolerated (see the Min comment above), so clamp
            // before taking the square root to avoid NaN on a negative product.
            CoefficientCombineRule::GeometricMean => (coeff1.max(0.0) * coeff2.max(0.0)).sqrt(),
        }
    }
}

#[cfg(test)]
mod test {
    use super::CoefficientCombineRule;
    use crate::math::Real;

    fn combine(c1: Real, c2: Real, rule: CoefficientCombineRule) -> Real {
        CoefficientCombineRule::combine(c1, c2, rule, rule)
    }

    #[test]
    fn geometric_mean_combine() {
        assert_eq!(
            combine(0.25, 1.0, CoefficientCombineRule::GeometricMean),
            0.5
        );
        assert_eq!(
            combine(0.7, 0.0, CoefficientCombineRule::GeometricMean),
            0.0
        );
        // Negative coefficients (tolerated for the godot use-case) must not produce NaN.
        assert_eq!(
            combine(-0.5, 0.5, CoefficientCombineRule::GeometricMean),
            0.0
        );
        assert_eq!(
            combine(-0.5, -0.5, CoefficientCombineRule::GeometricMean),
            0.0
        );
    }

    #[test]
    fn geometric_mean_wins_rule_priority() {
        assert_eq!(
            CoefficientCombineRule::combine(
                0.25,
                1.0,
                CoefficientCombineRule::GeometricMean,
                CoefficientCombineRule::Average,
            ),
            0.5
        );
    }
}
