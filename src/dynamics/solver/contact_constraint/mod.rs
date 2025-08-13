pub(crate) use generic_one_body_constraint::*;
// pub(crate) use generic_one_body_constraint_element::*;
pub(crate) use contact_constraints_set::{
    ConstraintsCounts, ContactConstraintTypes, ContactConstraintsSet,
};
pub(crate) use generic_two_body_constraint::*;
pub(crate) use generic_two_body_constraint_element::*;
pub(crate) use one_body_constraint::*;
pub(crate) use one_body_constraint_element::*;
#[cfg(feature = "simd-is-enabled")]
pub(crate) use one_body_constraint_simd::*;
pub(crate) use two_body_constraint::*;
pub(crate) use two_body_constraint_element::*;
#[cfg(feature = "simd-is-enabled")]
pub(crate) use two_body_constraint_simd::*;

mod contact_constraints_set;
mod generic_one_body_constraint;
mod generic_one_body_constraint_element;
mod generic_two_body_constraint;
mod generic_two_body_constraint_element;
mod one_body_constraint;
mod one_body_constraint_element;
#[cfg(feature = "simd-is-enabled")]
mod one_body_constraint_simd;
mod two_body_constraint;
mod two_body_constraint_element;
#[cfg(feature = "simd-is-enabled")]
mod two_body_constraint_simd;

#[cfg(feature = "dim2")]
#[cfg(test)]
mod test {
    use crate::prelude::{
        BroadPhaseMultiSap, CCDSolver, ColliderBuilder, ColliderSet, ImpulseJointSet,
        IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
        RigidBodyBuilder, RigidBodySet,
    };
    use na::{vector, Vector};

    #[test]
    fn tangent_not_nan_818() {
        /*
         * World
         */

        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhaseMultiSap::new();
        let mut nf = NarrowPhase::new();
        let mut islands = IslandManager::new();

        /*
         * The ground
         */
        let ground_size = 20.0;
        let ground_height = 1.0;

        let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -3.0]);
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(ground_size, ground_height)
            .rotation((0.25 as crate::math::Real).to_radians())
            .friction(0.9);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        /*
         * A tilted capsule that cannot rotate.
         */
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 4.0])
            .lock_rotations();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::capsule_y(2.0, 1.0).friction(0.9);
        colliders.insert_with_parent(collider, handle, &mut bodies);

        // Steps
        let gravity = Vector::y() * -9.81;

        for _ in 0..50 {
            pipeline.step(
                &gravity,
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
        let pairs: Vec<_> = nf.contact_pairs().collect();
        println!("contact pairs: {:?}", pairs);
        for contact in pairs {
            for manifold in contact.manifolds.iter() {
                for point in manifold.points.iter() {
                    assert!(
                        point.data.tangent_impulse.index(0).is_finite(),
                        "tangent impulse from a contact pair point data should be normal."
                    );
                }
            }
        }
    }
}
