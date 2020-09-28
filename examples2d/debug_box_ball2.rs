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
     * Ground
     */
    let rad = 1.0;
    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -rad)
        .rotation(std::f32::consts::PI / 4.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(rad, rad).build();
    colliders.insert(collider, handle, &mut bodies);

    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0 * rad)
        .can_sleep(false)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    //    testbed.look_at(Point2::new(10.0, 10.0, 10.0), Point2::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
