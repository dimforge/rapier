#[repr(transparent)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
/// Pairwise filtering using bit masks.
///
/// This filtering method is based on two 32-bit values:
/// - The interaction groups (the 32 left-most bits of `self.0`).
/// - The interaction mask (the 32 right-most bits of `self.0`).
///
/// An interaction is allowed between two filters `a` and `b` when two conditions
/// are met simultaneously for [`Self::test_and`] or individually for [`Self::test_or`]:
/// - The interaction groups of `a` has at least one bit set to `1` in common with the interaction mask of `b`.
/// - The interaction groups of `b` has at least one bit set to `1` in common with the interaction mask of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met
/// for [`Self::test_and`]:
/// ```ignore
/// ((self.0 >> 32) & rhs.0) != 0 && ((rhs.0 >> 32) & self.0) != 0
/// ```
/// or for [`Self::test_or`]:
/// ```ignore
/// ((self.0 >> 32) & rhs.0) != 0 || ((rhs.0 >> 32) & self.0) != 0
/// ```
pub struct InteractionGroups(pub u64);

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
/// Specifies which method should be used to test interactions
pub enum InteractionTestMode {
    /// Use [`InteractionGroups::test_and`].
    AND,
    /// Use [`InteractionGroups::test_or`].
    OR,
}

impl InteractionGroups {
    /// Initializes with the given interaction groups and interaction mask.
    pub const fn new(groups: u32, masks: u32) -> Self {
        Self::none().with_groups(groups).with_mask(masks)
    }

    /// Allow interaction with everything.
    pub const fn all() -> Self {
        Self(u64::MAX)
    }

    /// Prevent all interactions.
    pub const fn none() -> Self {
        Self(0)
    }

    /// Sets the group this filter is part of.
    pub const fn with_groups(self, groups: u32) -> Self {
        Self((self.0 & 0x0000_0000_ffff_ffff) | ((groups as u64) << 32))
    }

    /// Sets the interaction mask of this filter.
    pub const fn with_mask(self, mask: u32) -> Self {
        Self((self.0 & 0xffff_ffff_0000_0000) | (mask as u64))
    }

    /// Check if interactions should be allowed based on the interaction groups and mask.
    ///
    /// An interaction is allowed iff. the groups of `self` contain at least one bit set to 1 in common
    /// with the mask of `rhs`, **and** vice-versa.
    #[inline]
    pub const fn test_and(self, rhs: Self) -> bool {
        ((self.0 >> 32) & rhs.0) != 0 && ((rhs.0 >> 32) & self.0) != 0
    }

    /// Check if interactions should be allowed based on the interaction groups and mask.
    ///
    /// An interaction is allowed iff. the groups of `self` contain at least one bit set to 1 in common
    /// with the mask of `rhs`, **or** vice-versa.
    #[inline]
    pub const fn test_or(self, rhs: Self) -> bool {
        ((self.0 >> 32) & rhs.0) != 0 || ((rhs.0 >> 32) & self.0) != 0
    }

    /// Check if interactions should be allowed based on the interaction groups and mask.
    ///
    /// See [`InteractionTestMode`] for more info.
    #[inline]
    pub const fn test(self, rhs: Self, mode: InteractionTestMode) -> bool {
        match mode {
            InteractionTestMode::AND => self.test_and(rhs),
            InteractionTestMode::OR => self.test_or(rhs),
        }
    }
}

impl Default for InteractionGroups {
    fn default() -> Self {
        Self::all()
    }
}
