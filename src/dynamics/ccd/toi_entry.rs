use crate::data::Coarena;
use crate::dynamics::ccd::ccd_solver::CCDContact;
use crate::dynamics::ccd::CCDData;
use crate::dynamics::{IntegrationParameters, RigidBody, RigidBodyHandle};
use crate::geometry::{Collider, ColliderHandle};
use crate::math::{Isometry, Real};
use crate::parry::query::PersistentQueryDispatcher;
use crate::utils::WCross;
use na::{RealField, Unit};
use parry::query::{NonlinearRigidMotion, QueryDispatcher, TOI};

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

    pub fn try_from_colliders<QD: ?Sized + PersistentQueryDispatcher<(), ()>>(
        params: &IntegrationParameters,
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
        body_params: &Coarena<CCDData>,
    ) -> Option<Self> {
        assert!(start_time <= end_time);

        let linvel1 = frozen1.is_none() as u32 as Real * b1.linvel;
        let linvel2 = frozen2.is_none() as u32 as Real * b2.linvel;

        let vel12 = linvel2 - linvel1;
        let thickness = (c1.shape().ccd_thickness() + c2.shape().ccd_thickness());

        if params.dt * vel12.norm() < thickness {
            return None;
        }

        let is_intersection_test = c1.is_sensor() || c2.is_sensor();

        let body_params1 = body_params.get(c1.parent.0)?;
        let body_params2 = body_params.get(c2.parent.0)?;

        // Compute the TOI.
        let mut motion1 = body_params1.motion(params.dt, b1, 0.0);
        let mut motion2 = body_params2.motion(params.dt, b2, 0.0);

        if let Some(t) = frozen1 {
            motion1.freeze(t);
        }

        if let Some(t) = frozen2 {
            motion2.freeze(t);
        }

        let mut toi;
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

        toi = res_toi??;

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
