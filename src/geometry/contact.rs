use crate::data::MaybeSerializableData;
use crate::dynamics::BodyPair;
use crate::geometry::contact_generator::{ContactGeneratorWorkspace, ContactPhase};
use crate::geometry::{Collider, ColliderPair, ColliderSet};
use crate::math::{Isometry, Point, Vector};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::math::{SimdFloat, SIMD_WIDTH},
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The type local linear approximation of the neighborhood of a pair contact points on two shapes
pub enum KinematicsCategory {
    /// Both neighborhoods are assimilated to a single point.
    PointPoint,
    /// The first shape's neighborhood at the contact point is assimilated to a plane while
    /// the second is assimilated to a point.
    PlanePoint,
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// Local contact geometry at the neighborhood of a pair of contact points.
pub struct ContactKinematics {
    /// The local contact geometry.
    pub category: KinematicsCategory,
    /// The dilation applied to the first contact geometry.
    pub radius1: f32,
    /// The dilation applied to the second contact geometry.
    pub radius2: f32,
}

impl Default for ContactKinematics {
    fn default() -> Self {
        ContactKinematics {
            category: KinematicsCategory::PointPoint,
            radius1: 0.0,
            radius2: 0.0,
        }
    }
}

#[cfg(feature = "simd-is-enabled")]
pub(crate) struct WContact {
    pub local_p1: Point<SimdFloat>,
    pub local_p2: Point<SimdFloat>,
    pub local_n1: Vector<SimdFloat>,
    pub local_n2: Vector<SimdFloat>,
    pub dist: SimdFloat,
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
            impulse: 0.0,
            tangent_impulse: Contact::zero_tangent_impulse(),
            fid1: self.fid1[i],
            fid2: self.fid2[i],
        };

        (c, self.local_n1.extract(i), self.local_n2.extract(i))
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A single contact between two collider.
pub struct Contact {
    /// The contact point in the local-space of the first collider.
    pub local_p1: Point<f32>,
    /// The contact point in the local-space of the second collider.
    pub local_p2: Point<f32>,
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
    /// The identifier of the subshape of the first collider involved in this contact.
    ///
    /// For primitive shapes like cuboid, ball, etc., this is 0.
    /// For shapes like trimesh and heightfield this identifies the specific triangle
    /// involved in the contact.
    pub fid1: u8,
    /// The identifier of the subshape of the second collider involved in this contact.
    ///
    /// For primitive shapes like cuboid, ball, etc., this is 0.
    /// For shapes like trimesh and heightfield this identifies the specific triangle
    /// involved in the contact.
    pub fid2: u8,
    /// The distance between the two colliders along the contact normal.
    ///
    /// If this is negative, the colliders are penetrating.
    pub dist: f32,
}

impl Contact {
    pub(crate) fn new(
        local_p1: Point<f32>,
        local_p2: Point<f32>,
        fid1: u8,
        fid2: u8,
        dist: f32,
    ) -> Self {
        Self {
            local_p1,
            local_p2,
            impulse: 0.0,
            #[cfg(feature = "dim2")]
            tangent_impulse: 0.0,
            #[cfg(feature = "dim3")]
            tangent_impulse: [0.0; 2],
            fid1,
            fid2,
            dist,
        }
    }

    #[cfg(feature = "dim2")]
    pub(crate) fn zero_tangent_impulse() -> f32 {
        0.0
    }

    #[cfg(feature = "dim3")]
    pub(crate) fn zero_tangent_impulse() -> [f32; 2] {
        [0.0, 0.0]
    }

    pub(crate) fn copy_geometry_from(&mut self, contact: Contact) {
        self.local_p1 = contact.local_p1;
        self.local_p2 = contact.local_p2;
        self.fid1 = contact.fid1;
        self.fid2 = contact.fid2;
        self.dist = contact.dist;
    }

    // pub(crate) fn swap(self) -> Self {
    //     Self {
    //         local_p1: self.local_p2,
    //         local_p2: self.local_p1,
    //         impulse: self.impulse,
    //         tangent_impulse: self.tangent_impulse,
    //         fid1: self.fid2,
    //         fid2: self.fid1,
    //         dist: self.dist,
    //     }
    // }
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
            let manifold = ContactManifold::from_colliders(self.pair, coll1, coll2, flags);
            self.manifolds.push(manifold);
        }

        // We have to make sure the order of the returned collider
        // match the order of the pair stored inside of the manifold.
        // (This order can be modified by the contact determination algorithm).
        let manifold = &mut self.manifolds[0];
        if manifold.pair.collider1 == self.pair.collider1 {
            manifold.position1 = *coll1.position();
            manifold.position2 = *coll2.position();
            (
                coll1,
                coll2,
                manifold,
                self.generator_workspace.as_mut().map(|w| &mut *w.0),
            )
        } else {
            manifold.position1 = *coll2.position();
            manifold.position2 = *coll1.position();
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
pub struct ContactManifold {
    // NOTE: use a SmallVec instead?
    // And for 2D use an ArrayVec since there will never be more than 2 contacts anyways.
    #[cfg(feature = "dim2")]
    pub(super) points: arrayvec::ArrayVec<[Contact; 2]>,
    #[cfg(feature = "dim3")]
    pub(super) points: Vec<Contact>,
    /// The number of active contacts on this contact manifold.
    ///
    /// Active contacts are these that may result in contact forces.
    pub num_active_contacts: usize,
    /// The contact normal of all the contacts of this manifold, expressed in the local space of the first collider.
    pub local_n1: Vector<f32>,
    /// The contact normal of all the contacts of this manifold, expressed in the local space of the second collider.
    pub local_n2: Vector<f32>,
    /// The contact kinematics of all the contacts of this manifold.
    pub kinematics: ContactKinematics,
    // The following are set by the narrow-phase.
    /// The pair of body involved in this contact manifold.
    pub body_pair: BodyPair,
    /// The pair of colliders involved in this contact manifold.
    pub pair: ColliderPair,
    /// The pair of subshapes involved in this contact manifold.
    pub subshape_index_pair: (usize, usize),
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
    /// The world-space position of the first collider at the time the contact points were generated.
    pub position1: Isometry<f32>,
    /// The world-space position of the second collider at the time the contact points were generated.
    pub position2: Isometry<f32>,
    /// Flags used to control some aspects of the constraints solver for this contact manifold.
    pub solver_flags: SolverFlags,
}

impl ContactManifold {
    pub(crate) fn new(
        pair: ColliderPair,
        subshapes: (usize, usize),
        body_pair: BodyPair,
        position1: Isometry<f32>,
        position2: Isometry<f32>,
        friction: f32,
        restitution: f32,
        solver_flags: SolverFlags,
    ) -> ContactManifold {
        Self {
            #[cfg(feature = "dim2")]
            points: arrayvec::ArrayVec::new(),
            #[cfg(feature = "dim3")]
            points: Vec::new(),
            num_active_contacts: 0,
            local_n1: Vector::zeros(),
            local_n2: Vector::zeros(),
            pair,
            subshape_index_pair: subshapes,
            body_pair,
            kinematics: ContactKinematics::default(),
            warmstart_multiplier: Self::min_warmstart_multiplier(),
            friction,
            restitution,
            position1,
            position2,
            constraint_index: 0,
            position_constraint_index: 0,
            solver_flags,
        }
    }

    pub(crate) fn take(&mut self) -> Self {
        ContactManifold {
            #[cfg(feature = "dim2")]
            points: self.points.clone(),
            #[cfg(feature = "dim3")]
            points: std::mem::replace(&mut self.points, Vec::new()),
            num_active_contacts: self.num_active_contacts,
            local_n1: self.local_n1,
            local_n2: self.local_n2,
            kinematics: self.kinematics,
            body_pair: self.body_pair,
            pair: self.pair,
            subshape_index_pair: self.subshape_index_pair,
            warmstart_multiplier: self.warmstart_multiplier,
            friction: self.friction,
            restitution: self.restitution,
            position1: self.position1,
            position2: self.position2,
            constraint_index: self.constraint_index,
            position_constraint_index: self.position_constraint_index,
            solver_flags: self.solver_flags,
        }
    }

    pub(crate) fn from_colliders(
        pair: ColliderPair,
        coll1: &Collider,
        coll2: &Collider,
        flags: SolverFlags,
    ) -> Self {
        Self::with_subshape_indices(pair, coll1, coll2, 0, 0, flags)
    }

    pub(crate) fn with_subshape_indices(
        pair: ColliderPair,
        coll1: &Collider,
        coll2: &Collider,
        subshape1: usize,
        subshape2: usize,
        solver_flags: SolverFlags,
    ) -> Self {
        Self::new(
            pair,
            (subshape1, subshape2),
            BodyPair::new(coll1.parent, coll2.parent),
            *coll1.position(),
            *coll2.position(),
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

    /// Number of active contacts on this contact manifold.
    #[inline]
    pub fn num_active_contacts(&self) -> usize {
        self.num_active_contacts
    }

    /// The slice of all the active contacts on this contact manifold.
    ///
    /// Active contacts are contacts that may end up generating contact forces.
    #[inline]
    pub fn active_contacts(&self) -> &[Contact] {
        &self.points[..self.num_active_contacts]
    }

    #[inline]
    pub(crate) fn active_contacts_mut(&mut self) -> &mut [Contact] {
        &mut self.points[..self.num_active_contacts]
    }

    /// The slice of all the contacts, active or not, on this contact manifold.
    #[inline]
    pub fn all_contacts(&self) -> &[Contact] {
        &self.points
    }

    pub(crate) fn swap_identifiers(&mut self) {
        self.pair = self.pair.swap();
        self.body_pair = self.body_pair.swap();
        self.subshape_index_pair = (self.subshape_index_pair.1, self.subshape_index_pair.0);
        std::mem::swap(&mut self.position1, &mut self.position2);
    }

    pub(crate) fn update_warmstart_multiplier(&mut self) {
        // In 2D, tall stacks will actually suffer from this
        // because oscillation due to inaccuracies in 2D often
        // cause contacts to break, which would result in
        // a reset of the warmstart multiplier.
        if cfg!(feature = "dim2") {
            self.warmstart_multiplier = 1.0;
            return;
        }

        for pt in &self.points {
            if pt.impulse != 0.0 {
                self.warmstart_multiplier = (self.warmstart_multiplier * 2.0).min(1.0);
                return;
            }
        }

        // Reset the multiplier.
        self.warmstart_multiplier = Self::min_warmstart_multiplier()
    }

    #[inline]
    pub(crate) fn try_update_contacts(&mut self, pos12: &Isometry<f32>) -> bool {
        //        const DOT_THRESHOLD: f32 = 0.crate::COS_10_DEGREES;
        const DOT_THRESHOLD: f32 = crate::utils::COS_5_DEGREES;
        const DIST_SQ_THRESHOLD: f32 = 0.001; // FIXME: this should not be hard-coded.
        self.try_update_contacts_eps(pos12, DOT_THRESHOLD, DIST_SQ_THRESHOLD)
    }

    #[inline]
    pub(crate) fn try_update_contacts_eps(
        &mut self,
        pos12: &Isometry<f32>,
        angle_dot_threshold: f32,
        dist_sq_threshold: f32,
    ) -> bool {
        if self.points.len() == 0 {
            return false;
        }

        let local_n2 = pos12 * self.local_n2;

        if -self.local_n1.dot(&local_n2) < angle_dot_threshold {
            return false;
        }

        for pt in &mut self.points {
            let local_p2 = pos12 * pt.local_p2;
            let dpt = local_p2 - pt.local_p1;
            let dist = dpt.dot(&self.local_n1);

            if dist * pt.dist < 0.0 {
                // We switched between penetrating/non-penetrating.
                // The may result in other contacts to appear.
                return false;
            }
            let new_local_p1 = local_p2 - self.local_n1 * dist;

            if na::distance_squared(&pt.local_p1, &new_local_p1) > dist_sq_threshold {
                return false;
            }

            pt.dist = dist;
            pt.local_p1 = new_local_p1;
        }

        true
    }

    /// Sort the contacts of this contact manifold such that the active contacts are in the first
    /// positions of the array.
    #[inline]
    pub(crate) fn sort_contacts(&mut self, prediction_distance: f32) {
        let num_contacts = self.points.len();
        match num_contacts {
            0 => {
                self.num_active_contacts = 0;
            }
            1 => {
                self.num_active_contacts = (self.points[0].dist < prediction_distance) as usize;
            }
            _ => {
                let mut first_inactive_index = num_contacts;

                self.num_active_contacts = 0;
                while self.num_active_contacts != first_inactive_index {
                    if self.points[self.num_active_contacts].dist >= prediction_distance {
                        // Swap with the last contact.
                        self.points
                            .swap(self.num_active_contacts, first_inactive_index - 1);
                        first_inactive_index -= 1;
                    } else {
                        self.num_active_contacts += 1;
                    }
                }
            }
        }
    }
}
