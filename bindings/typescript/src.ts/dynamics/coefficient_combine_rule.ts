/**
 * A rule applied to combine coefficients.
 *
 * Use this when configuring the `ColliderDesc` to specify
 * how friction and restitution coefficient should be combined
 * in a contact.
 */
export enum CoefficientCombineRule {
    Average = 0,
    Min = 1,
    Multiply = 2,
    Max = 3,
}
