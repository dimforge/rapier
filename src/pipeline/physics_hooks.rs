use crate::dynamics::RigidBody;
use crate::geometry::{Collider, ColliderHandle, ContactManifold, SolverContact, SolverFlags};

/// Context given to custom collision filters to filter-out collisions.
pub struct PairFilterContext<'a> {
    /// The first collider involved in the potential collision.
    pub rigid_body1: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub rigid_body2: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub collider_handle1: ColliderHandle,
    /// The first collider involved in the potential collision.
    pub collider_handle2: ColliderHandle,
    /// The first collider involved in the potential collision.
    pub collider1: &'a Collider,
    /// The first collider involved in the potential collision.
    pub collider2: &'a Collider,
}

pub struct ContactModificationContext<'a> {
    /// The first collider involved in the potential collision.
    pub rigid_body1: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub rigid_body2: &'a RigidBody,
    /// The first collider involved in the potential collision.
    pub collider_handle1: ColliderHandle,
    /// The first collider involved in the potential collision.
    pub collider_handle2: ColliderHandle,
    /// The first collider involved in the potential collision.
    pub collider1: &'a Collider,
    /// The first collider involved in the potential collision.
    pub collider2: &'a Collider,
    /// The contact manifold.
    pub manifold: &'a ContactManifold,
    /// The solver contacts that can be modified.
    pub solver_contacts: &'a mut Vec<SolverContact>,
    /// User-defined data attached to the manifold.
    // NOTE: we keep this a &'a mut u32 to emphasize the
    // fact that this can be modified.
    pub user_data: &'a mut u32,
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct PhysicsHooksFlags: u32 {
        /// If set, Rapier will call `PhysicsHooks::filter_contact_pair` whenever relevant.
        const FILTER_CONTACT_PAIR = 0b0001;
        /// If set, Rapier will call `PhysicsHooks::filter_intersection_pair` whenever relevant.
        const FILTER_INTERSECTION_PAIR = 0b0010;
        /// If set, Rapier will call `PhysicsHooks::modify_solver_contact` whenever relevant.
        const MODIFY_SOLVER_CONTACTS = 0b0100;
    }
}

/// User-defined functions called by the physics engines during one timestep in order to customize its behavior.
pub trait PhysicsHooks: Send + Sync {
    /// The sets of hooks that must be taken into account.
    fn active_hooks(&self) -> PhysicsHooksFlags;

    /// Applies the contact pair filter.
    ///
    /// Note that this method will only be called if `self.active_hooks()`
    /// contains the `PhysicsHooksFlags::FILTER_CONTACT_PAIR` flags.
    ///
    /// User-defined filter for potential contact pairs detected by the broad-phase.
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their contact computed by the narrow-phase, and if these contact
    /// should be solved by the constraints solver
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

    /// Applies the intersection pair filter.
    ///
    /// Note that this method will only be called if `self.active_hooks()`
    /// contains the `PhysicsHooksFlags::FILTER_INTERSECTION_PAIR` flags.
    ///
    /// User-defined filter for potential intersection pairs detected by the broad-phase.
    ///
    /// This can be used to apply custom logic in order to decide whether two colliders
    /// should have their intersection computed by the narrow-phase.
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

    /// Modifies the set of contacts seen by the constraints solver.
    ///
    /// Note that this method will only be called if `self.active_hooks()`
    /// contains the `PhysicsHooksFlags::MODIFY_SOLVER_CONTACTS` flags.
    ///
    /// By default, the content of `solver_contacts` is computed from `manifold.points`.
    /// This method will be called on each contact manifold which have the flag `SolverFlags::MODIFY_CONTACTS` set.
    /// This method can be used to modify the set of solver contacts seen by the constraints solver: contacts
    /// can be removed and modified.
    ///
    /// Note that if all the contacts have to be ignored by the constraint solver, you may simply
    /// do `context.solver_contacts.clear()`.
    ///
    /// Modifying the solver contacts allow you to achieve various effects, including:
    /// - Simulating conveyor belts by setting the `surface_velocity` of a solver contact.
    /// - Simulating shapes with multiply materials by modifying the friction and restitution
    ///   coefficient depending of the features in contacts.
    /// - Simulating one-way platforms depending on the contact normal.
    ///
    /// Each contact manifold is given a `u32` user-defined data that is persistent between
    /// timesteps (as long as the contact manifold exists). This user-defined data is initialized
    /// as 0 and can be modified in `context.user_data`.
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext);
}

impl PhysicsHooks for () {
    fn active_hooks(&self) -> PhysicsHooksFlags {
        PhysicsHooksFlags::empty()
    }

    fn filter_contact_pair(&self, _: &PairFilterContext) -> Option<SolverFlags> {
        None
    }

    fn filter_intersection_pair(&self, _: &PairFilterContext) -> bool {
        false
    }

    fn modify_solver_contacts(&self, _: &mut ContactModificationContext) {}
}
