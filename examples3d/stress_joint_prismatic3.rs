use na::{Point3, Unit, Vector3};
use rapier3d::dynamics::{JointSet, PrismaticJoint, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    let rad = 0.4;
    let num = 5;
    let shift = 1.0;

    for m in 0..8 {
        let z = m as f32 * shift * (num as f32 + 2.0);

        for l in 0..8 {
            let y = l as f32 * shift * (num as f32) * 2.0;

            for j in 0..50 {
                let x = j as f32 * shift * 4.0;

                let ground = RigidBodyBuilder::new_static().translation(x, y, z).build();
                let mut curr_parent = bodies.insert(ground);
                let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
                colliders.insert(collider, curr_parent, &mut bodies);

                for i in 0..num {
                    let z = z + (i + 1) as f32 * shift;
                    let density = 1.0;
                    let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                    let curr_child = bodies.insert(rigid_body);
                    let collider = ColliderBuilder::cuboid(rad, rad, rad)
                        .density(density)
                        .build();
                    colliders.insert(collider, curr_child, &mut bodies);

                    let axis = if i % 2 == 0 {
                        Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0))
                    } else {
                        Unit::new_normalize(Vector3::new(-1.0, 1.0, 0.0))
                    };

                    let z = Vector3::z();
                    let mut prism = PrismaticJoint::new(
                        Point3::origin(),
                        axis,
                        z,
                        Point3::new(0.0, 0.0, -shift),
                        axis,
                        z,
                    );
                    prism.limits_enabled = true;
                    prism.limits[0] = -2.0;
                    prism.limits[1] = 2.0;
                    joints.insert(&mut bodies, curr_parent, curr_child, prism);

                    curr_parent = curr_child;
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(
        Point3::new(262.0, 63.0, 124.0),
        Point3::new(101.0, 4.0, -3.0),
    );
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Joints", init_world)]);
    testbed.run()
}
