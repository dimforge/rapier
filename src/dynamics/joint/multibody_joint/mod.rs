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
        let mut rnd = oorandom::Rand32::new(1234);

        for k in 0..10 {
            let mut bodies = RigidBodySet::new();
            let mut multibody_joints = MultibodyJointSet::new();
            let mut colliders = ColliderSet::new();
            let mut impulse_joints = ImpulseJointSet::new();
            let mut islands = IslandManager::new();

            let mut pipeline = PhysicsPipeline::new();
            let mut bf = BroadPhaseMultiSap::new();
            let mut nf = NarrowPhase::new();

            let num_links = 100;
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
            match k {
                0 => {} // Remove in insertion order.
                1 => {
                    // Remove from leaf to root.
                    handles.reverse();
                }
                _ => {
                    // Shuffle the vector a bit.
                    // (This test checks multiple shuffle arrangements due to k > 2).
                    for l in 0..num_links {
                        handles.swap(l, rnd.rand_range(0..num_links as u32) as usize);
                    }
                }
            }

            for (i, handle) in handles.iter().enumerate() {
                dbg!(i);
                bodies.remove(
                    *handle,
                    &mut islands,
                    &mut colliders,
                    &mut impulse_joints,
                    &mut multibody_joints,
                    true,
                );
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
}
