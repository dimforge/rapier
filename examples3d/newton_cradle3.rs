use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let radius = 0.5;
    let length = 10.0 * radius;
    let rb = RigidBodyBuilder::dynamic();
    let co = ColliderBuilder::ball(radius).restitution(1.0);

    let n = 5;

    for i in 0..n {
        let (ball_pos, attach) = (
            Vector::new(i as Real * 2.2 * radius, 0.0, 0.0),
            Vector::Y * length,
        );
        let vel = if i >= n - 1 {
            Vector::new(7.0, 0.0, 0.0)
        } else {
            Vector::ZERO
        };

        let ground = bodies.insert(RigidBodyBuilder::fixed().translation(ball_pos + attach));
        let rb = rb.clone().translation(ball_pos).linvel(vel);
        let handle = bodies.insert(rb);
        colliders.insert_with_parent(co.clone(), handle, &mut bodies);

        let joint = SphericalJointBuilder::new().local_anchor2(attach);
        impulse_joints.insert(ground, handle, joint, true);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
