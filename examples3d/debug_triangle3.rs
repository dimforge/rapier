use na::Point3;
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

    // Triangle ground.
    let vtx = [
        Point3::new(-10.0, 0.0, -10.0),
        Point3::new(10.0, 0.0, -10.0),
        Point3::new(0.0, 0.0, 10.0),
    ];

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, 0.0, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::triangle(vtx[0], vtx[1], vtx[2]).build();
    colliders.insert(collider, handle, &mut bodies);

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(1.1, 0.01, 0.0)
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(20.0, 0.1, 1.0).build();
    colliders.insert(collider, handle, &mut bodies);

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
