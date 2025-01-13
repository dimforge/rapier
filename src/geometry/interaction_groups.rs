#![allow(clippy::bad_bit_mask)] // Clippy will complain about the bitmasks due to Group::NONE being 0.

/// Pairwise filtering using bit masks.
///
/// This filtering method is based on two 32-bit values:
/// - The interaction groups memberships.
/// - The interaction groups filter.
///
/// An interaction is allowed between two filters `a` and `b` when two conditions
/// are met simultaneously for [`InteractionTestMode::And`] or individually for [`InteractionTestMode::Or`]::
/// - The groups membership of `a` has at least one bit set to `1` in common with the groups filter of `b`.
/// - The groups membership of `b` has at least one bit set to `1` in common with the groups filter of `a`.
///
/// In other words, interactions are allowed between two filter iff. the following condition is met
/// for [`InteractionTestMode::And`]:
/// ```ignore
/// (self.memberships.bits() & rhs.filter.bits()) != 0 && (rhs.memberships.bits() & self.filter.bits()) != 0
/// ```
/// or for [`InteractionTestMode::Or`]:
/// ```ignore
/// (self.memberships.bits() & rhs.filter.bits()) != 0 || (rhs.memberships.bits() & self.filter.bits()) != 0
/// ```
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
#[repr(C)]
pub struct InteractionGroups {
    /// Groups memberships.
    pub memberships: Group,
    /// Groups filter.
    pub filter: Group,
    /// Interaction test mode
    ///
    /// In case of different test modes between two [`InteractionGroups`], [`InteractionTestMode::And`] is given priority.
    pub test_mode: InteractionTestMode,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, Default)]
/// Specifies which method should be used to test interactions.
///
/// In case of different test modes between two [`InteractionGroups`], [`InteractionTestMode::And`] is given priority.
pub enum InteractionTestMode {
    /// Use [`InteractionGroups::test_and`].
    #[default]
    And,
    /// Use [`InteractionGroups::test_or`], iff. the `rhs` is also [`InteractionTestMode::Or`].
    ///
    /// If the `rhs` is not [`InteractionTestMode::Or`], use [`InteractionGroups::test_and`].
    Or,
}

impl InteractionGroups {
    /// Initializes with the given interaction groups and interaction mask.
    pub const fn new(memberships: Group, filter: Group, test_mode: InteractionTestMode) -> Self {
        Self {
            memberships,
            filter,
            test_mode,
        }
    }

    /// Allow interaction with everything.
    pub const fn all() -> Self {
        Self::new(Group::ALL, Group::ALL, InteractionTestMode::And)
    }

    /// Prevent all interactions.
    pub const fn none() -> Self {
        Self::new(Group::NONE, Group::NONE, InteractionTestMode::And)
    }

    /// Sets the group this filter is part of.
    pub const fn with_memberships(mut self, memberships: Group) -> Self {
        self.memberships = memberships;
        self
    }

    /// Sets the interaction mask of this filter.
    pub const fn with_filter(mut self, filter: Group) -> Self {
        self.filter = filter;
        self
    }

    /// Check if interactions should be allowed based on the interaction memberships and filter.
    ///
    /// An interaction is allowed iff. the memberships of `self` contain at least one bit set to 1 in common
    /// with the filter of `rhs`, **and** vice-versa.
    #[inline]
    pub const fn test_and(self, rhs: Self) -> bool {
        (self.memberships.bits() & rhs.filter.bits()) != 0
            && (rhs.memberships.bits() & self.filter.bits()) != 0
    }

    /// Check if interactions should be allowed based on the interaction memberships and filter.
    ///
    /// An interaction is allowed iff. the groups of `self` contain at least one bit set to 1 in common
    /// with the mask of `rhs`, **or** vice-versa.
    #[inline]
    pub const fn test_or(self, rhs: Self) -> bool {
        (self.memberships.bits() & rhs.filter.bits()) != 0
            || (rhs.memberships.bits() & self.filter.bits()) != 0
    }

    /// Check if interactions should be allowed based on the interaction memberships and filter.
    ///
    /// See [`InteractionTestMode`] for more info.
    #[inline]
    pub fn test(self, rhs: Self) -> bool {
        match (self.test_mode, rhs.test_mode) {
            (InteractionTestMode::And, _) => self.test_and(rhs),
            (InteractionTestMode::Or, InteractionTestMode::And) => self.test_and(rhs),
            (InteractionTestMode::Or, InteractionTestMode::Or) => self.test_or(rhs),
        }
    }
}

impl Default for InteractionGroups {
    fn default() -> Self {
        Self {
            memberships: Group::GROUP_1,
            filter: Group::ALL,
            test_mode: InteractionTestMode::And,
        }
    }
}

bitflags::bitflags! {
    /// A bit mask identifying groups for interaction.
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    #[derive(Copy, Clone, PartialEq, Eq, Debug, Hash)]
    pub struct Group: u32 {
        /// The group n°1.
        const GROUP_1 = 1 << 0;
        /// The group n°2.
        const GROUP_2 = 1 << 1;
        /// The group n°3.
        const GROUP_3 = 1 << 2;
        /// The group n°4.
        const GROUP_4 = 1 << 3;
        /// The group n°5.
        const GROUP_5 = 1 << 4;
        /// The group n°6.
        const GROUP_6 = 1 << 5;
        /// The group n°7.
        const GROUP_7 = 1 << 6;
        /// The group n°8.
        const GROUP_8 = 1 << 7;
        /// The group n°9.
        const GROUP_9 = 1 << 8;
        /// The group n°10.
        const GROUP_10 = 1 << 9;
        /// The group n°11.
        const GROUP_11 = 1 << 10;
        /// The group n°12.
        const GROUP_12 = 1 << 11;
        /// The group n°13.
        const GROUP_13 = 1 << 12;
        /// The group n°14.
        const GROUP_14 = 1 << 13;
        /// The group n°15.
        const GROUP_15 = 1 << 14;
        /// The group n°16.
        const GROUP_16 = 1 << 15;
        /// The group n°17.
        const GROUP_17 = 1 << 16;
        /// The group n°18.
        const GROUP_18 = 1 << 17;
        /// The group n°19.
        const GROUP_19 = 1 << 18;
        /// The group n°20.
        const GROUP_20 = 1 << 19;
        /// The group n°21.
        const GROUP_21 = 1 << 20;
        /// The group n°22.
        const GROUP_22 = 1 << 21;
        /// The group n°23.
        const GROUP_23 = 1 << 22;
        /// The group n°24.
        const GROUP_24 = 1 << 23;
        /// The group n°25.
        const GROUP_25 = 1 << 24;
        /// The group n°26.
        const GROUP_26 = 1 << 25;
        /// The group n°27.
        const GROUP_27 = 1 << 26;
        /// The group n°28.
        const GROUP_28 = 1 << 27;
        /// The group n°29.
        const GROUP_29 = 1 << 28;
        /// The group n°30.
        const GROUP_30 = 1 << 29;
        /// The group n°31.
        const GROUP_31 = 1 << 30;
        /// The group n°32.
        const GROUP_32 = 1 << 31;

        /// All of the groups.
        const ALL = u32::MAX;
        /// None of the groups.
        const NONE = 0;
    }
}

impl From<u32> for Group {
    #[inline]
    fn from(val: u32) -> Self {
        Self::from_bits_retain(val)
    }
}

impl From<Group> for u32 {
    #[inline]
    fn from(val: Group) -> Self {
        val.bits()
    }
}
