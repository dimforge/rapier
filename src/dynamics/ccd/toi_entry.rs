use crate::dynamics::{RigidBody, RigidBodyHandle};
use crate::geometry::{Collider, ColliderHandle};
use crate::math::Real;
use parry::query::{NonlinearRigidMotion, QueryDispatcher};

#[derive(Copy, Clone, Debug)]
pub struct TOIEntry {
    pub toi: Real,
    pub c1: ColliderHandle,
    pub b1: RigidBodyHandle,
    pub c2: ColliderHandle,
    pub b2: RigidBodyHandle,
    pub is_intersection_test: bool,
    pub timestamp: usize,
}

impl TOIEntry {
    fn new(
        toi: Real,
        c1: ColliderHandle,
        b1: RigidBodyHandle,
        c2: ColliderHandle,
        b2: RigidBodyHandle,
        is_intersection_test: bool,
        timestamp: usize,
    ) -> Self {
        Self {
            toi,
            c1,
            b1,
            c2,
            b2,
            is_intersection_test,
            timestamp,
        }
    }

    pub fn try_from_colliders<QD: ?Sized + QueryDispatcher>(
        query_dispatcher: &QD,
        ch1: ColliderHandle,
        ch2: ColliderHandle,
        c1: &Collider,
        c2: &Collider,
        b1: &RigidBody,
        b2: &RigidBody,
        frozen1: Option<Real>,
        frozen2: Option<Real>,
        start_time: Real,
        end_time: Real,
        smallest_contact_dist: Real,
    ) -> Option<Self> {
        assert!(start_time <= end_time);

        let linvel1 = frozen1.is_none() as u32 as Real * b1.linvel();
        let linvel2 = frozen2.is_none() as u32 as Real * b2.linvel();
        let angvel1 = frozen1.is_none() as u32 as Real * b1.angvel();
        let angvel2 = frozen2.is_none() as u32 as Real * b2.angvel();

        #[cfg(feature = "dim2")]
        let vel12 = (linvel2 - linvel1).norm()
            + angvel1.abs() * b1.ccd_max_dist
            + angvel2.abs() * b2.ccd_max_dist;
        #[cfg(feature = "dim3")]
        let vel12 = (linvel2 - linvel1).norm()
            + angvel1.norm() * b1.ccd_max_dist
            + angvel2.norm() * b2.ccd_max_dist;

        // We may be slightly over-conservative by taking the `max(0.0)` here.
        // But removing the `max` doesn't really affect performances so let's
        // keep it since more conservatism is good at this stage.
        let thickness = (c1.shape().ccd_thickness() + c2.shape().ccd_thickness())
            + smallest_contact_dist.max(0.0);
        let is_intersection_test = c1.is_sensor() || c2.is_sensor();

        if (end_time - start_time) * vel12 < thickness {
            return None;
        }

        // Compute the TOI.
        let mut motion1 = Self::body_motion(b1);
        let mut motion2 = Self::body_motion(b2);

        if let Some(t) = frozen1 {
            motion1.freeze(t);
        }

        if let Some(t) = frozen2 {
            motion2.freeze(t);
        }

        let motion_c1 = motion1.prepend(*c1.position_wrt_parent());
        let motion_c2 = motion2.prepend(*c2.position_wrt_parent());

        // println!("start_time: {}", start_time);

        // If this is just an intersection test (i.e. with sensors)
        // then we can stop the TOI search immediately if it starts with
        // a penetration because we don't care about the whether the velocity
        // at the impact is a separating velocity or not.
        // If the TOI search involves two non-sensor colliders then
        // we don't want to stop the TOI search at the first penetration
        // because the colliders may be in a separating trajectory.
        let stop_at_penetration = is_intersection_test;

        let res_toi = query_dispatcher
            .nonlinear_time_of_impact(
                &motion_c1,
                c1.shape(),
                &motion_c2,
                c2.shape(),
                start_time,
                end_time,
                stop_at_penetration,
            )
            .ok();

        let toi = res_toi??;

        Some(Self::new(
            toi.toi,
            ch1,
            c1.parent(),
            ch2,
            c2.parent(),
            is_intersection_test,
            0,
        ))
    }

    fn body_motion(body: &RigidBody) -> NonlinearRigidMotion {
        if body.is_ccd_active() {
            NonlinearRigidMotion::new(
                body.position,
                body.mass_properties.local_com,
                body.linvel,
                body.angvel,
            )
        } else {
            NonlinearRigidMotion::constant_position(body.next_position)
        }
    }
}

impl PartialOrd for TOIEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        (-self.toi).partial_cmp(&(-other.toi))
    }
}

impl Ord for TOIEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl PartialEq for TOIEntry {
    fn eq(&self, other: &Self) -> bool {
        self.toi == other.toi
    }
}

impl Eq for TOIEntry {}
