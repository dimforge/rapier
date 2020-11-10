use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    let rad = 1.0;
    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0 * rad, 0.0)
        .can_sleep(false)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
