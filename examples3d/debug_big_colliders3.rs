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

    let mut curr_y = 0.0;
    let mut curr_width = 1_000.0;

    for _ in 0..6 {
        curr_y += curr_width;

        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(0.0, curr_y, 0.0)
            .build();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(curr_width, curr_width, curr_width).build();
        colliders.insert(collider, handle, &mut bodies);

        curr_width /= 10.0;
    }

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
