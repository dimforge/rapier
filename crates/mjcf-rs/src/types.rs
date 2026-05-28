//! Common AST types shared across modules.

/// Tristate field used by attributes such as `<joint limited>`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum Tristate {
    /// Explicit `"true"` / `"1"`.
    True,
    /// Explicit `"false"` / `"0"`.
    False,
    /// `"auto"` — defer to `<compiler autolimits>`. This is the default.
    #[default]
    Auto,
}

impl Tristate {
    /// Resolves the tristate against the compiler's autolimits flag.
    pub fn resolve(self, autolimits: bool, has_range: bool) -> bool {
        match self {
            Tristate::True => true,
            Tristate::False => false,
            Tristate::Auto => autolimits && has_range,
        }
    }
}

/// MJCF coordinate convention. Currently only "local" is supported.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum Coordinate {
    /// Default — every element's pose is in the parent's local frame.
    #[default]
    Local,
    /// Deprecated; rejected by the parser.
    Global,
}
