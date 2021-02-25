use crate::dynamics::{BodyPair, RigidBodyHandle};
use crate::geometry::{ColliderPair, ContactManifold};
use crate::math::{Point, Real, Vector};
use parry::query::ContactManifoldsWorkspace;

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct SolverFlags: u32 {
        /// The constraint solver will take this contact manifold into
        /// account for force computation.
        const COMPUTE_IMPULSES = 0b001;
        /// The user-defined physics hooks will be used to
        /// modify the solver contacts of this contact manifold.
        const MODIFY_SOLVER_CONTACTS = 0b010;
    }
}

impl Default for SolverFlags {
    fn default() -> Self {
        SolverFlags::COMPUTE_IMPULSES
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A single contact between two collider.
pub struct ContactData {
    /// The impulse, along the contact normal, applied by this contact to the first collider's rigid-body.
    ///
    /// The impulse applied to the second collider's rigid-body is given by `-impulse`.
    pub impulse: Real,
    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim2")]
    pub tangent_impulse: Real,
    /// The friction impulses along the basis orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim3")]
    pub tangent_impulse: [Real; 2],
}

impl ContactData {
    #[cfg(feature = "dim2")]
    pub(crate) fn zero_tangent_impulse() -> Real {
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn zero_tangent_impulse() -> [Real; 2] {
        [0.0, 0.0]
    }
}

impl Default for ContactData {
    fn default() -> Self {
        Self {
            impulse: 0.0,
            tangent_impulse: Self::zero_tangent_impulse(),
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// The description of all the contacts between a pair of colliders.
pub struct ContactPair {
    /// The pair of colliders involved.
    pub pair: ColliderPair,
    /// The set of contact manifolds between the two colliders.
    ///
    /// All contact manifold contain themselves contact points between the colliders.
    pub manifolds: Vec<ContactManifold>,
    /// Is there any active contact in this contact pair?
    pub has_any_active_contact: bool,
    pub(crate) workspace: Option<ContactManifoldsWorkspace>,
}

impl ContactPair {
    pub(crate) fn new(pair: ColliderPair) -> Self {
        Self {
            pair,
            has_any_active_contact: false,
            manifolds: Vec::new(),
            workspace: None,
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A contact manifold between two colliders.
///
/// A contact manifold describes a set of contacts between two colliders. All the contact
/// part of the same contact manifold share the same contact normal and contact kinematics.
pub struct ContactManifoldData {
    // The following are set by the narrow-phase.
    /// The pair of body involved in this contact manifold.
    pub body_pair: BodyPair,
    pub(crate) warmstart_multiplier: Real,
    // The two following are set by the constraints solver.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) constraint_index: usize,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) position_constraint_index: usize,
    // We put the following fields here to avoids reading the colliders inside of the
    // contact preparation method.
    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub solver_flags: SolverFlags,
    /// The world-space contact normal shared by all the contact in this contact manifold.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub normal: Vector<Real>,
    /// The contacts that will be seen by the constraints solver for computing forces.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub solver_contacts: Vec<SolverContact>,
    /// The relative dominance of the bodies involved in this contact manifold.
    pub relative_dominance: i16,
    /// A user-defined piece of data.
    pub user_data: u32,
}

/// A contact seen by the constraints solver for computing forces.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SolverContact {
    /// The index of the manifold contact used to generate this solver contact.
    pub(crate) contact_id: u8,
    /// The world-space contact point.
    pub point: Point<Real>,
    /// The distance between the two original contacts points along the contact normal.
    /// If negative, this is measures the penetration depth.
    pub dist: Real,
    /// The effective friction coefficient at this contact point.
    pub friction: Real,
    /// The effective restitution coefficient at this contact point.
    pub restitution: Real,
    /// The desired tangent relative velocity at the contact point.
    ///
    /// This is set to zero by default. Set to a non-zero value to
    /// simulate, e.g., conveyor belts.
    pub tangent_velocity: Vector<Real>,
    /// Associated contact data used to warm-start the constraints
    /// solver.
    pub data: ContactData,
}

impl SolverContact {
    /// Should we treat this contact as a bouncy contact?
    /// If `true`, use [`Self::restitution`].
    pub fn is_bouncy(&self) -> bool {
        let is_new = self.data.impulse == 0.0;
        if is_new {
            // Treat new collisions as bouncing at first, unless we have zero restitution.
            self.restitution > 0.0
        } else {
            // If the contact is still here one step later, it is now a resting contact.
            // The exception is very high restitutions, which can never rest
            self.restitution >= 1.0
        }
    }
}

impl Default for ContactManifoldData {
    fn default() -> Self {
        Self::new(
            BodyPair::new(RigidBodyHandle::invalid(), RigidBodyHandle::invalid()),
            SolverFlags::empty(),
        )
    }
}

impl ContactManifoldData {
    pub(crate) fn new(body_pair: BodyPair, solver_flags: SolverFlags) -> ContactManifoldData {
        Self {
            body_pair,
            warmstart_multiplier: Self::min_warmstart_multiplier(),
            constraint_index: 0,
            position_constraint_index: 0,
            solver_flags,
            normal: Vector::zeros(),
            solver_contacts: Vec::new(),
            relative_dominance: 0,
            user_data: 0,
        }
    }

    /// Number of actives contacts, i.e., contacts that will be seen by
    /// the constraints solver.
    #[inline]
    pub fn num_active_contacts(&self) -> usize {
        self.solver_contacts.len()
    }

    pub(crate) fn min_warmstart_multiplier() -> Real {
        // Multiplier used to reduce the amount of warm-starting.
        // This coefficient increases exponentially over time, until it reaches 1.0.
        // This will reduce significant overshoot at the timesteps that
        // follow a timestep involving high-velocity impacts.
        1.0 // 0.01
    }

    // pub(crate) fn update_warmstart_multiplier(manifold: &mut ContactManifold) {
    //     // In 2D, tall stacks will actually suffer from this
    //     // because oscillation due to inaccuracies in 2D often
    //     // cause contacts to break, which would result in
    //     // a reset of the warmstart multiplier.
    //     if cfg!(feature = "dim2") {
    //         manifold.data.warmstart_multiplier = 1.0;
    //         return;
    //     }
    //
    //     for pt in &manifold.points {
    //         if pt.data.impulse != 0.0 {
    //             manifold.data.warmstart_multiplier =
    //                 (manifold.data.warmstart_multiplier * 2.0).min(1.0);
    //             return;
    //         }
    //     }
    //
    //     // Reset the multiplier.
    //     manifold.data.warmstart_multiplier = Self::min_warmstart_multiplier()
    // }
}
