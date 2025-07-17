//! MultibodyJoints using the reduced-coordinates formalism or using constraints.

pub use self::multibody::Multibody;
pub use self::multibody_ik::InverseKinematicsOption;
pub use self::multibody_joint::MultibodyJoint;
pub use self::multibody_joint_set::{
    MultibodyIndex, MultibodyJointHandle, MultibodyJointSet, MultibodyLinkId,
};
pub use self::multibody_link::MultibodyLink;
pub use self::unit_multibody_joint::{unit_joint_limit_constraint, unit_joint_motor_constraint};

mod multibody;
mod multibody_joint_set;
mod multibody_link;
mod multibody_workspace;

mod multibody_ik;
mod multibody_joint;
mod unit_multibody_joint;

#[cfg(test)]
mod test {
    use crate::prelude::{
        BroadPhaseMultiSap, CCDSolver, ColliderSet, ImpulseJointSet, IntegrationParameters,
        IslandManager, NarrowPhase, PhysicsPipeline, RigidBodySet,
    };
    use crate::prelude::{MultibodyJointSet, RevoluteJoint};
    use parry::math::Vector;

    #[test]
    fn multibody_joint_remove_and_step() {
        let mut bodies = RigidBodySet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut islands = IslandManager::new();

        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhaseMultiSap::new();
        let mut nf = NarrowPhase::new();

        let num_links = 2;
        let mut handles = vec![];

        for _ in 0..num_links {
            use crate::prelude::RigidBodyBuilder;

            handles.push(bodies.insert(RigidBodyBuilder::dynamic()));
        }

        #[cfg(feature = "dim2")]
        let joint = RevoluteJoint::new();
        #[cfg(feature = "dim3")]
        let joint = RevoluteJoint::new(na::Vector::x_axis());

        for i in 0..num_links - 1 {
            multibody_joints
                .insert(handles[i], handles[i + 1], joint, true)
                .unwrap();
        }
        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut CCDSolver::new(),
            None,
            &(),
            &(),
        );

        for handle in handles.clone().iter() {
            bodies.remove(
                *handle,
                &mut islands,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                true,
            );
            // Remove from our handles the one we just removed.
            handles.retain(|value| value != handle);
            // All handles left should be correctly set up.
            for handle in handles.iter() {
                // Note debug #847: rigid_body_link should return none when attempting to remove the last (second) body.
                if let Some(link) = multibody_joints.rigid_body_link(*handle).copied() {
                    if multibody_joints
                        .get_multibody_mut_internal(link.multibody)
                        .is_none()
                    {
                        panic!("multibody_joints should be able to get all links returned from rigid_body_link.");
                    }
                    panic!("This test has only 1 link, it should lead to rigid_body_link returning `None` immediately after first removal.");
                }
            }
            pipeline.step(
                &Vector::zeros(),
                &IntegrationParameters::default(),
                &mut islands,
                &mut bf,
                &mut nf,
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut CCDSolver::new(),
                None,
                &(),
                &(),
            );
        }
    }
}
