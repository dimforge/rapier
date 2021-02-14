#[repr(transparent)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
/// Pairwise filtering using bit masks.
///
/// This filtering method is based on two 16-bit values:
/// - The interaction groups (the 16 left-most bits of `self.0`).
/// - The interaction mask (the 16 right-most bits of `self.0`).
///
/// An interaction is allowed between two filters `a` and `b` when two conditions
/// are met simultaneously:
/// - The interaction groups of `a` has at least one bit set to `1` in common with the interaction mask of `b`.
/// - The interaction groups of `b` has at least one bit set to `1` in common with the interaction mask of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met:
/// ```ignore
/// ((self.0 >> 16) & rhs.0) != 0 && ((rhs.0 >> 16) & self.0) != 0
/// ```
pub struct InteractionGroups(pub u32);

impl InteractionGroups {
    /// Initializes with the given interaction groups and interaction mask.
    pub const fn new(groups: u16, masks: u16) -> Self {
        Self::none().with_groups(groups).with_mask(masks)
    }

    /// Allow interaction with everything.
    pub const fn all() -> Self {
        Self(u32::MAX)
    }

    /// Prevent all interactions.
    pub const fn none() -> Self {
        Self(0)
    }

    /// Sets the group this filter is part of.
    pub const fn with_groups(self, groups: u16) -> Self {
        Self((self.0 & 0x0000ffff) | ((groups as u32) << 16))
    }

    /// Sets the interaction mask of this filter.
    pub const fn with_mask(self, mask: u16) -> Self {
        Self((self.0 & 0xffff0000) | (mask as u32))
    }

    /// Check if interactions should be allowed based on the interaction groups and mask.
    ///
    /// An interaction is allowed iff. the groups of `self` contain at least one bit set to 1 in common
    /// with the mask of `rhs`, and vice-versa.
    #[inline]
    pub const fn test(self, rhs: Self) -> bool {
        ((self.0 >> 16) & rhs.0) != 0 && ((rhs.0 >> 16) & self.0) != 0
    }
}

impl Default for InteractionGroups {
    fn default() -> Self {
        Self::all()
    }
}
