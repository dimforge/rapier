//! `<equality>` element AST.

/// One `<equality>` entry.
#[derive(Clone, Debug)]
pub enum Equality {
    /// `<equality><connect>` — point-to-point constraint between two bodies.
    Connect(EqualityConnect),
    /// `<equality><weld>` — rigid attachment between two bodies.
    Weld(EqualityWeld),
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
