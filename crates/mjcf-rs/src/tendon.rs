//! `<tendon>` element AST. Only **fixed** tendons are represented — a fixed
//! tendon is a linear combination of joint coordinates, `L = Σ coefᵢ·qᵢ`, used
//! for actuator transmission and (via `<equality><tendon>`) length coupling.
//! Spatial tendons (site-routed cables with wrapping) are out of scope.

/// One `(joint, coefficient)` term of a [`FixedTendon`].
#[derive(Clone, Debug, Default)]
pub struct TendonJoint {
    /// Referenced joint name.
    pub joint: String,
    /// Coefficient of this joint in the tendon length (`coef`, default 1).
    pub coef: f64,
}

/// A `<tendon><fixed>` element: the scalar length `Σ coefᵢ·qᵢ` over its joints.
#[derive(Clone, Debug, Default)]
pub struct FixedTendon {
    /// `name` attribute.
    pub name: Option<String>,
    /// `<default>` class.
    pub class: Option<String>,
    /// The `(joint, coef)` terms, in document order. The first term's joint is
    /// treated as the tendon's "primary" coordinate by the loader.
    pub joints: Vec<TendonJoint>,
}
