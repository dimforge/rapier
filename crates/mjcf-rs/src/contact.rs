//! `<contact>` element AST.

/// All `<contact>` data for a model.
#[derive(Default, Clone, Debug)]
pub struct Contact {
    /// `<contact><pair>` overrides.
    pub pairs: Vec<ContactPair>,
    /// `<contact><exclude>` exclusions.
    pub excludes: Vec<ContactExclude>,
}

/// `<contact><pair>` element.
#[derive(Clone, Debug, Default)]
pub struct ContactPair {
    /// Pair name (optional).
    pub name: Option<String>,
    /// Default class.
    pub class: Option<String>,
    /// Geom name 1.
    pub geom1: String,
    /// Geom name 2.
    pub geom2: String,
    /// `condim` override (1, 3, 4, or 6).
    pub condim: Option<u32>,
    /// Friction `(slide, spin, roll)` override.
    pub friction: Option<[f64; 3]>,
    /// Margin override.
    pub margin: Option<f64>,
    /// Gap override.
    pub gap: Option<f64>,
}

/// `<contact><exclude>` element.
#[derive(Clone, Debug, Default)]
pub struct ContactExclude {
    /// Exclude name (optional).
    pub name: Option<String>,
    /// Body name 1.
    pub body1: String,
    /// Body name 2.
    pub body2: String,
}
