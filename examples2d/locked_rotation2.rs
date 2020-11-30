use na::Point2;
use rapier2d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

// This shows a bug when a cylinder is in contact with a very large
// but very thin cuboid. In this case the EPA returns an incorrect
// contact normal, resulting in the cylinder falling through the floor.
pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * A rectangle that only rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 3.0)
        .lock_translations()
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(2.0, 0.6).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * A tilted capsule that cannot rotate.
     */
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 5.0)
        .rotation(1.0)
        .lock_rotations()
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::capsule_y(0.6, 0.4).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 0.0), 40.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
