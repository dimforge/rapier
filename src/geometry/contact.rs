use crate::buckler::query::TrackedData;
use crate::data::MaybeSerializableData;
use crate::dynamics::{BodyPair, RigidBodyHandle, RigidBodySet};
use crate::geometry::contact_generator::{ContactGeneratorWorkspace, ContactPhase};
use crate::geometry::{Collider, ColliderPair, ColliderSet, Contact, ContactManifold};
use crate::math::{Isometry, Point, Vector};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::math::{SimdReal, SIMD_WIDTH},
    simba::simd::SimdValue,
};

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub struct SolverFlags: u32 {
        /// The constraint solver will take this contact manifold into
        /// account for force computation.
        const COMPUTE_IMPULSES = 0b01;
    }
}

#[cfg(feature = "simd-is-enabled")]
pub(crate) struct WContact {
    pub local_p1: Point<SimdReal>,
    pub local_p2: Point<SimdReal>,
    pub local_n1: Vector<SimdReal>,
    pub local_n2: Vector<SimdReal>,
    pub dist: SimdReal,
    pub fid1: [u8; SIMD_WIDTH],
    pub fid2: [u8; SIMD_WIDTH],
}

#[cfg(feature = "simd-is-enabled")]
impl WContact {
    pub fn extract(&self, i: usize) -> (Contact, Vector<f32>, Vector<f32>) {
        let c = Contact {
            local_p1: self.local_p1.extract(i),
            local_p2: self.local_p2.extract(i),
            dist: self.dist.extract(i),
            fid1: self.fid1[i],
            fid2: self.fid2[i],
            data: ContactData::default(),
        };

        (c, self.local_n1.extract(i), self.local_n2.extract(i))
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A single contact between two collider.
pub struct ContactData {
    /// The impulse, along the contact normal, applied by this contact to the first collider's rigid-body.
    ///
    /// The impulse applied to the second collider's rigid-body is given by `-impulse`.
    pub impulse: f32,
    /// The friction impulse along the vector orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim2")]
    pub tangent_impulse: f32,
    /// The friction impulses along the basis orthonormal to the contact normal, applied to the first
    /// collider's rigid-body.
    #[cfg(feature = "dim3")]
    pub tangent_impulse: [f32; 2],
}

impl ContactData {
    #[cfg(feature = "dim2")]
    pub(crate) fn zero_tangent_impulse() -> f32 {
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn zero_tangent_impulse() -> [f32; 2] {
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
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) generator: Option<ContactPhase>,
    pub(crate) generator_workspace: Option<ContactGeneratorWorkspace>,
}

impl ContactPair {
    pub(crate) fn new(
        pair: ColliderPair,
        generator: ContactPhase,
        generator_workspace: Option<ContactGeneratorWorkspace>,
    ) -> Self {
        Self {
            pair,
            manifolds: Vec::new(),
            generator: Some(generator),
            generator_workspace,
        }
    }

    /// Does this contact pair have any active contact?
    ///
    /// An active contact is a contact that may result in a non-zero contact force.
    pub fn has_any_active_contact(&self) -> bool {
        for manifold in &self.manifolds {
            if manifold.num_active_contacts != 0 {
                return true;
            }
        }

        false
    }

    pub(crate) fn single_manifold<'a, 'b>(
        &'a mut self,
        colliders: &'b ColliderSet,
        flags: SolverFlags,
    ) -> (
        &'b Collider,
        &'b Collider,
        &'a mut ContactManifold,
        Option<&'a mut (dyn MaybeSerializableData)>,
    ) {
        let coll1 = &colliders[self.pair.collider1];
        let coll2 = &colliders[self.pair.collider2];

        if self.manifolds.len() == 0 {
            let manifold_data = ContactManifoldData::from_colliders(self.pair, coll1, coll2, flags);
            self.manifolds
                .push(ContactManifold::with_data((0, 0), manifold_data));
        }

        // We have to make sure the order of the returned collider
        // match the order of the pair stored inside of the manifold.
        // (This order can be modified by the contact determination algorithm).
        let manifold = &mut self.manifolds[0];
        if manifold.data.pair.collider1 == self.pair.collider1 {
            (
                coll1,
                coll2,
                manifold,
                self.generator_workspace.as_mut().map(|w| &mut *w.0),
            )
        } else {
            (
                coll2,
                coll1,
                manifold,
                self.generator_workspace.as_mut().map(|w| &mut *w.0),
            )
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
    /// The pair of colliders involved in this contact manifold.
    pub pair: ColliderPair,
    /// The pair of body involved in this contact manifold.
    pub body_pair: BodyPair,
    pub(crate) warmstart_multiplier: f32,
    // The two following are set by the constraints solver.
    pub(crate) constraint_index: usize,
    pub(crate) position_constraint_index: usize,
    // We put the following fields here to avoids reading the colliders inside of the
    // contact preparation method.
    /// The friction coefficient for of all the contacts on this contact manifold.
    pub friction: f32,
    /// The restitution coefficient for all the contacts on this contact manifold.
    pub restitution: f32,
    /// The relative position between the first collider and its parent at the time the
    /// contact points were generated.
    pub delta1: Isometry<f32>,
    /// The relative position between the second collider and its parent at the time the
    /// contact points were generated.
    pub delta2: Isometry<f32>,
    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub solver_flags: SolverFlags,
}

impl Default for ContactManifoldData {
    fn default() -> Self {
        Self::new(
            ColliderPair::new(ColliderSet::invalid_handle(), ColliderSet::invalid_handle()),
            BodyPair::new(
                RigidBodySet::invalid_handle(),
                RigidBodySet::invalid_handle(),
            ),
            Isometry::identity(),
            Isometry::identity(),
            0.0,
            0.0,
            SolverFlags::empty(),
        )
    }
}

impl TrackedData for ContactManifoldData {
    fn flip(&mut self) {
        std::mem::swap(&mut self.pair.collider1, &mut self.pair.collider2);
        std::mem::swap(&mut self.body_pair.body1, &mut self.body_pair.body2);
        std::mem::swap(&mut self.delta1, &mut self.delta2);
    }
}

impl ContactManifoldData {
    pub(crate) fn new(
        pair: ColliderPair,
        body_pair: BodyPair,
        delta1: Isometry<f32>,
        delta2: Isometry<f32>,
        friction: f32,
        restitution: f32,
        solver_flags: SolverFlags,
    ) -> ContactManifoldData {
        Self {
            pair,
            body_pair,
            warmstart_multiplier: Self::min_warmstart_multiplier(),
            friction,
            restitution,
            delta1,
            delta2,
            constraint_index: 0,
            position_constraint_index: 0,
            solver_flags,
        }
    }

    pub(crate) fn from_colliders(
        pair: ColliderPair,
        coll1: &Collider,
        coll2: &Collider,
        flags: SolverFlags,
    ) -> Self {
        Self::with_subshape_indices(pair, coll1, coll2, flags)
    }

    pub(crate) fn with_subshape_indices(
        pair: ColliderPair,
        coll1: &Collider,
        coll2: &Collider,
        solver_flags: SolverFlags,
    ) -> Self {
        Self::new(
            pair,
            BodyPair::new(coll1.parent, coll2.parent),
            *coll1.position_wrt_parent(),
            *coll2.position_wrt_parent(),
            (coll1.friction + coll2.friction) * 0.5,
            (coll1.restitution + coll2.restitution) * 0.5,
            solver_flags,
        )
    }

    pub(crate) fn min_warmstart_multiplier() -> f32 {
        // Multiplier used to reduce the amount of warm-starting.
        // This coefficient increases exponentially over time, until it reaches 1.0.
        // This will reduce significant overshoot at the timesteps that
        // follow a timestep involving high-velocity impacts.
        0.01
    }

    pub(crate) fn update_warmstart_multiplier(manifold: &mut ContactManifold) {
        // In 2D, tall stacks will actually suffer from this
        // because oscillation due to inaccuracies in 2D often
        // cause contacts to break, which would result in
        // a reset of the warmstart multiplier.
        if cfg!(feature = "dim2") {
            manifold.data.warmstart_multiplier = 1.0;
            return;
        }

        for pt in &manifold.points {
            if pt.data.impulse != 0.0 {
                manifold.data.warmstart_multiplier =
                    (manifold.data.warmstart_multiplier * 2.0).min(1.0);
                return;
            }
        }

        // Reset the multiplier.
        manifold.data.warmstart_multiplier = Self::min_warmstart_multiplier()
    }
}
