use na::{Point2, Vector2};
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Create the balls
     */
    let num = 10;
    let rad = 0.2;

    let subdiv = 1.0 / (num as f32);

    for i in 0usize..num {
        let (x, y) = (i as f32 * subdiv * std::f32::consts::PI * 2.0).sin_cos();

        // Build the rigid body.
        let rb = RigidBodyBuilder::new_dynamic()
            .translation(x, y)
            .linvel(x * 10.0, y * 10.0)
            .angvel(100.0)
            .linear_damping((i + 1) as f32 * subdiv * 10.0)
            .angular_damping((num - i) as f32 * subdiv * 10.0)
            .build();
        let rb_handle = bodies.insert(rb);

        // Build the collider.
        let co = ColliderBuilder::cuboid(rad, rad).build();
        colliders.insert(co, rb_handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(bodies, colliders, joints, Vector2::zeros(), ());
    testbed.look_at(Point2::new(3.0, 2.0), 50.0);
}
