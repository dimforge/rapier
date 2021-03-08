use na::{Point3, Vector3};
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

    /*
     * Ground
     */
    let ground_size = 100.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
        .friction(1.5)
        .build();
    colliders.insert(collider, handle, &mut bodies);

    // Build a dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 1.1, 0.0)
        .rotation(Vector3::y() * 0.3)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(2.0, 1.0, 3.0).friction(1.5).build();
    colliders.insert(collider, handle, &mut bodies);

    let rigid_body = &mut bodies[handle];
    let force = rigid_body.position() * Vector3::z() * 50.0;
    rigid_body.set_linvel(force, true);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(10.0, 10.0, 10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
