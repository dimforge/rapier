//! `<equality>` element AST.

/// One `<equality>` entry.
#[derive(Clone, Debug)]
pub enum Equality {
    /// `<equality><connect>` — point-to-point constraint between two bodies.
    Connect(EqualityConnect),
    /// `<equality><weld>` — rigid attachment between two bodies.
    Weld(EqualityWeld),
    /// `<equality><joint>` — polynomial coupling between two joints' positions.
    Joint(EqualityJoint),
}

/// Common metadata shared by all equality variants.
#[derive(Clone, Debug, Default)]
pub struct EqualityCommon {
    /// `name` attribute.
    pub name: Option<String>,
    /// Class.
    pub class: Option<String>,
    /// `active` attribute (default `true`).
    pub active: bool,
}

/// `<equality><connect>` element.
#[derive(Clone, Debug, Default)]
pub struct EqualityConnect {
    /// Common metadata.
    pub common: EqualityCommon,
    /// First body.
    pub body1: String,
    /// Second body. May be omitted (constrains to world).
    pub body2: Option<String>,
    /// Anchor in body1's frame.
    pub anchor: [f64; 3],
}

/// `<equality><weld>` element.
#[derive(Clone, Debug, Default)]
pub struct EqualityWeld {
    /// Common metadata.
    pub common: EqualityCommon,
    /// First body.
    pub body1: String,
    /// Second body. May be omitted (constrains to world).
    pub body2: Option<String>,
    /// Optional `relpose` attribute `(pos quat)`.
    pub relpose: Option<crate::Pose>,
    /// Optional anchor (`anchor="x y z"`).
    pub anchor: Option<[f64; 3]>,
    /// `torquescale` attribute.
    pub torque_scale: f64,
}

/// `<equality><joint>` element: couples the position of `joint2` to that of
/// `joint1` through a quartic polynomial of the offset from their reference
/// positions:
/// `q2 − ref2 = p0 + p1·(q1 − ref1) + p2·(q1 − ref1)² + p3·(q1 − ref1)³ + p4·(q1 − ref1)⁴`.
/// `joint2` may be omitted, in which case `joint1` is constrained to the
/// constant `p0`.
#[derive(Clone, Debug, Default)]
pub struct EqualityJoint {
    /// Common metadata.
    pub common: EqualityCommon,
    /// First (independent) joint.
    pub joint1: String,
    /// Second (dependent) joint. `None` constrains `joint1` to `polycoef[0]`.
    pub joint2: Option<String>,
    /// Polynomial coefficients `[p0, p1, p2, p3, p4]` (default `[0,1,0,0,0]`).
    pub polycoef: [f64; 5],
}
