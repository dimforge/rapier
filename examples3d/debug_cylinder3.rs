use na::Point3;
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

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
     * Ground
     */
    let ground_size = 100.1;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 1;
    let rad = 1.0;

    let shiftx = rad * 2.0 + rad;
    let shifty = rad * 2.0 + rad;
    let shiftz = rad * 2.0 + rad;
    let centerx = shiftx * (num / 2) as f32;
    let centery = shifty / 2.0;
    let centerz = shiftz * (num / 2) as f32;

    let offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    let x = -centerx + offset;
    let y = centery + 3.0;
    let z = -centerz + offset;

    // Build the rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cylinder(rad, rad).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(100.0, 100.0, 100.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
