use na::{Point2, Unit, Vector2};
use rapier2d::dynamics::{JointSet, PrismaticJoint, RigidBodyBuilder, RigidBodySet};
use rapier2d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    /*
     * Create the balls
     */
    // Build the rigid body.
    let rad = 0.4;
    let num = 10;
    let shift = 1.0;

    for l in 0..25 {
        let y = l as f32 * shift * (num as f32 + 2.0) * 2.0;

        for j in 0..50 {
            let x = j as f32 * shift * 4.0;

            let ground = RigidBodyBuilder::new_static().translation(x, y).build();
            let mut curr_parent = bodies.insert(ground);
            let collider = ColliderBuilder::cuboid(rad, rad).build();
            colliders.insert(collider, curr_parent, &mut bodies);

            for i in 0..num {
                let y = y - (i + 1) as f32 * shift;
                let density = 1.0;
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y).build();
                let curr_child = bodies.insert(rigid_body);
                let collider = ColliderBuilder::cuboid(rad, rad).density(density).build();
                colliders.insert(collider, curr_child, &mut bodies);

                let axis = if i % 2 == 0 {
                    Unit::new_normalize(Vector2::new(1.0, 1.0))
                } else {
                    Unit::new_normalize(Vector2::new(-1.0, 1.0))
                };

                let mut prism =
                    PrismaticJoint::new(Point2::origin(), axis, Point2::new(0.0, shift), axis);
                prism.limits_enabled = true;
                prism.limits[0] = -1.5;
                prism.limits[1] = 1.5;
                joints.insert(&mut bodies, curr_parent, curr_child, prism);

                curr_parent = curr_child;
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(80.0, 80.0), 15.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
