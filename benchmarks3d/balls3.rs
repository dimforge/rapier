use na::Point3;
use rapier3d::dynamics::{BodyStatus, JointSet, RigidBodyBuilder, RigidBodySet};
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
     * Create the balls
     */
    let num = 20;
    let rad = 1.0;

    let shift = rad * 2.0 + 1.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let centerz = shift * (num as f32) / 2.0;

    for i in 0..num {
        for j in 0usize..num {
            for k in 0..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                let status = if j == 0 {
                    BodyStatus::Static
                } else {
                    BodyStatus::Dynamic
                };
                let density = 0.477;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new(status).translation(x, y, z).build();
                let handle = bodies.insert(rigid_body);
                let collider = ColliderBuilder::ball(rad).density(density).build();
                colliders.insert(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(100.0, 100.0, 100.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Balls", init_world)]);
    testbed.run()
}
