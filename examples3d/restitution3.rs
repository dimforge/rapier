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
    let ground_size = 20.;
    let ground_height = 1.0;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 2.0)
        .restitution(1.0)
        .build();
    colliders.insert(collider, handle, &mut bodies);

    let num = 10;
    let rad = 0.5;

    for j in 0..2 {
        for i in 0..=num {
            let x = (i as f32) - num as f32 / 2.0;
            let rigid_body = RigidBodyBuilder::new_dynamic()
                .translation(x * 2.0, 10.0 * (j as f32 + 1.0), 0.0)
                .build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad)
                .restitution((i as f32) / (num as f32))
                .build();
            colliders.insert(collider, handle, &mut bodies);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(0.0, 3.0, 30.0), Point3::new(0.0, 3.0, 0.0));
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
