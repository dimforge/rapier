use crate::dynamics::RigidBody;
use crate::geometry::{Collider, SolverFlags};

/// Context given to custom collision filters to filter-out collisions.
pub struct PairFilterContext<'a> {
    /// The first collider involved in the potential collision.
    pub rigid_body1: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub rigid_body2: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub collider1: &'a Collider,
    /// The first collider involved in the potential collision.
    pub collider2: &'a Collider,
}

/// User-defined filter for potential contact pairs detected by the broad-phase.
///
/// This can be used to apply custom logic in order to decide whether two colliders
/// should have their contact computed by the narrow-phase, and if these contact
/// should be solved by the constraints solver
pub trait ContactPairFilter: Send + Sync {
    /// Applies the contact pair filter.
    ///
    /// Note that using a contact pair filter will replace the default contact filtering
    /// which consists of preventing contact computation between two non-dynamic bodies.
    ///
    /// This filtering method is called after taking into account the colliders collision groups.
    ///
    /// If this returns `None`, then the narrow-phase will ignore this contact pair and
    /// not compute any contact manifolds for it.
    /// If this returns `Some`, then the narrow-phase will compute contact manifolds for
    /// this pair of colliders, and configure them with the returned solver flags. For
    /// example, if this returns `Some(SolverFlags::COMPUTE_IMPULSES)` then the contacts
    /// will be taken into account by the constraints solver. If this returns
    /// `Some(SolverFlags::empty())` then the constraints solver will ignore these
    /// contacts.
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags>;
}

/// User-defined filter for potential intersection pairs detected by the broad-phase.
///
/// This can be used to apply custom logic in order to decide whether two colliders
/// should have their intersection computed by the narrow-phase.
pub trait ProximityPairFilter: Send + Sync {
    /// Applies the intersection pair filter.
    ///
    /// Note that using an intersection pair filter will replace the default intersection filtering
    /// which consists of preventing intersection computation between two non-dynamic bodies.
    ///
    /// This filtering method is called after taking into account the colliders collision groups.
    ///
    /// If this returns `false`, then the narrow-phase will ignore this pair and
    /// not compute any intersection information for it.
    /// If this return `true` then the narrow-phase will compute intersection
    /// information for this pair.
    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool;
}
