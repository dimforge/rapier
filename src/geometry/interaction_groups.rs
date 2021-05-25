/// Pairwise filtering using bit masks.
///
/// This filtering method is based on two 32-bit values:
/// - The interaction groups memberships.
/// - The interaction groups filter.
///
/// An interaction is allowed between two filters `a` and `b` when two conditions
/// are met simultaneously:
/// - The groups membership of `a` has at least one bit set to `1` in common with the groups filter of `b`.
/// - The groups membership of `b` has at least one bit set to `1` in common with the groups filter of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met:
/// ```ignore
/// (self.memberships & rhs.filter) != 0 && (rhs.memberships & self.filter) != 0
/// ```
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[repr(C)]
pub struct InteractionGroups {
    /// Groups memberships.
    pub memberships: u32,
    /// Groups filter.
    pub filter: u32,
}

impl InteractionGroups {
    /// Initializes with the given interaction groups and interaction mask.
    pub const fn new(memberships: u32, filter: u32) -> Self {
        Self {
            memberships,
            filter,
        }
    }

    /// Allow interaction with everything.
    pub const fn all() -> Self {
        Self::new(u32::MAX, u32::MAX)
    }

    /// Prevent all interactions.
    pub const fn none() -> Self {
        Self::new(0, 0)
    }

    /// Sets the group this filter is part of.
    pub const fn with_memberships(mut self, memberships: u32) -> Self {
        self.memberships = memberships;
        self
    }

    /// Sets the interaction mask of this filter.
    pub const fn with_filter(mut self, filter: u32) -> Self {
        self.filter = filter;
        self
    }

    /// Check if interactions should be allowed based on the interaction memberships and filter.
    ///
    /// An interaction is allowed iff. the memberships of `self` contain at least one bit set to 1 in common
    /// with the filter of `rhs`, and vice-versa.
    #[inline]
    pub const fn test(self, rhs: Self) -> bool {
        (self.memberships & rhs.filter) != 0 && (rhs.memberships & self.filter) != 0
    }
}

impl Default for InteractionGroups {
    fn default() -> Self {
        Self::all()
    }
}
