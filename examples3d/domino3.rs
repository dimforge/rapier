use na::{Point3, Translation3, UnitQuaternion, Vector3};
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
    let ground_size = 200.1;
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
    let num = 4000;
    let width = 1.0;
    let thickness = 0.1;

    let colors = [Point3::new(0.7, 0.5, 0.9), Point3::new(0.6, 1.0, 0.6)];

    let mut curr_angle = 0.0;
    let mut curr_rad = 10.0;
    let mut prev_angle;
    let mut skip = 0;
    for i in 0..num {
        let perimeter = 2.0 * std::f32::consts::PI * curr_rad;
        let spacing = thickness * 4.0;
        prev_angle = curr_angle;
        curr_angle += 2.0 * std::f32::consts::PI * spacing / perimeter;
        let (x, z) = curr_angle.sin_cos();

        // Build the rigid body.
        let two_pi = 2.0 * std::f32::consts::PI;
        let nudged = curr_angle % two_pi < prev_angle % two_pi;
        let tilt = if nudged || i == num - 1 { 0.2 } else { 0.0 };

        if skip == 0 {
            let rot = UnitQuaternion::new(Vector3::y() * curr_angle);
            let tilt = UnitQuaternion::new(rot * Vector3::z() * tilt);
            let position =
                Translation3::new(x * curr_rad, width * 2.0 + ground_height, z * curr_rad)
                    * tilt
                    * rot;
            let rigid_body = RigidBodyBuilder::new_dynamic().position(position).build();
            let handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(thickness, width * 2.0, width).build();
            colliders.insert(collider, handle, &mut bodies);
            testbed.set_body_color(handle, colors[i % 2]);
        } else {
            skip -= 1;
        }

        if nudged {
            skip = 5;
        }

        curr_rad += 1.5 / perimeter;
    }

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
