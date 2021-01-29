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
    let width = 0.5;
    let vtx = vec![
        Point3::new(-width, 0.0, -width),
        Point3::new(width, 0.0, -width),
        Point3::new(width, 0.0, width),
        Point3::new(-width, 0.0, width),
        Point3::new(-width, -width, -width),
        Point3::new(width, -width, -width),
        Point3::new(width, -width, width),
        Point3::new(-width, -width, width),
    ];
    let idx = vec![
        [0, 1, 2],
        [0, 2, 3],
        [4, 5, 6],
        [4, 6, 7],
        [0, 4, 7],
        [0, 7, 3],
        [1, 5, 6],
        [1, 6, 2],
        [3, 2, 7],
        [2, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
    ];

    // Dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 35.0, 0.0)
        // .rotation(Vector3::new(0.8, 0.2, 0.1))
        .can_sleep(false)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(1.0, 2.0, 1.0).build();
    colliders.insert(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, 0.0, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::trimesh(vtx, idx).build();
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
