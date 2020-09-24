use na::Point2;
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
    let ground_size = 25.0;

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::new_cuboid(ground_size, 1.2).build();
    colliders.insert(&mut bodies, collider, handle);

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(ground_size, ground_size * 4.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::new_cuboid(ground_size * 4.0, 1.2).build();
    colliders.insert(&mut bodies, collider, handle);

    let rigid_body = RigidBodyBuilder::new_static()
        .rotation(std::f32::consts::FRAC_PI_2)
        .translation(-ground_size, ground_size * 4.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::new_cuboid(ground_size * 4.0, 1.2).build();
    colliders.insert(&mut bodies, collider, handle);

    /*
     * Create the cubes
     */
    let num = 26;
    let rad = 0.5;

    let shift = rad * 2.0;
    let shifty = rad * 5.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shifty + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y).build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::new_capsule_y(rad * 1.5, rad)
                .density(1.0)
                .build();
            colliders.insert(&mut bodies, collider, handle);
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 50.0), 10.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Balls", init_world)]);
    testbed.run()
}
