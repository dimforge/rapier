//! Physics-hook implementations for MJCF features that need runtime
//! filtering: `<contact><exclude>` pair-suppression and `<contact><pair>`
//! per-pair friction / margin overrides.

use std::collections::HashMap;

use rapier3d::geometry::{ColliderHandle, SolverFlags};
use rapier3d::math::Real;
use rapier3d::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooks};

/// Friction / margin override for a specific collider pair, sourced from a
/// `<contact><pair>` element.
#[derive(Copy, Clone, Debug, Default)]
pub struct PairOverride {
    /// Friction coefficient override.
    pub friction: Option<Real>,
    /// Margin override (added to the contact distance threshold).
    pub margin: Option<Real>,
}

/// Hook implementation honouring MJCF's `<contact><exclude>` (suppress
/// contact between two collider sets) and `<contact><pair>` (override
/// friction / margin on a specific pair).
///
/// The user opts in by passing this object to the rapier physics pipeline
/// (`pipeline.step(&hooks, ...)`). Without it, excludes are ignored and
/// pair overrides aren't applied.
#[derive(Default, Clone, Debug)]
pub struct MjcfContactHooks {
    /// Excluded ordered collider pairs. We store both `(a, b)` and `(b, a)`
    /// for O(1) lookup regardless of which side rapier presents first.
    pub(crate) exclude: std::collections::HashSet<(ColliderHandle, ColliderHandle)>,
    /// Per-pair friction / margin overrides. Both orderings stored.
    pub(crate) overrides: HashMap<(ColliderHandle, ColliderHandle), PairOverride>,
}

impl MjcfContactHooks {
    /// Build an empty hook set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a pair of colliders that should never produce contacts.
    pub fn exclude(&mut self, a: ColliderHandle, b: ColliderHandle) {
        self.exclude.insert((a, b));
        self.exclude.insert((b, a));
    }

    /// Register a friction / margin override for a specific pair.
    pub fn add_override(&mut self, a: ColliderHandle, b: ColliderHandle, ov: PairOverride) {
        self.overrides.insert((a, b), ov);
        self.overrides.insert((b, a), ov);
    }

    /// `true` when this hook set has any excludes registered (lets the
    /// caller skip installing it when there's nothing to do).
    pub fn has_excludes(&self) -> bool {
        !self.exclude.is_empty()
    }

    /// `true` when there are any pair overrides registered.
    pub fn has_overrides(&self) -> bool {
        !self.overrides.is_empty()
    }
}

impl PhysicsHooks for MjcfContactHooks {
    fn filter_contact_pair(&self, ctx: &PairFilterContext) -> Option<SolverFlags> {
        if self.exclude.contains(&(ctx.collider1, ctx.collider2)) {
            None
        } else {
            Some(SolverFlags::COMPUTE_IMPULSES)
        }
    }

    fn modify_solver_contacts(&self, ctx: &mut ContactModificationContext) {
        let key = (ctx.collider1, ctx.collider2);
        if let Some(ov) = self.overrides.get(&key) {
            if let Some(f) = ov.friction {
                for c in ctx.solver_contacts.iter_mut() {
                    c.friction = f;
                }
            }
            // Margin: rapier's solver uses `dist` as penetration depth;
            // shifting it acts like adding to the contact margin.
            if let Some(m) = ov.margin {
                for c in ctx.solver_contacts.iter_mut() {
                    c.dist -= m;
                }
            }
        }
    }
}
