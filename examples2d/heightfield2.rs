use na::{DVector, Point2, Vector2};
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
    let ground_size = Vector2::new(50.0, 1.0);
    let nsubdivs = 2000;

    let heights = DVector::from_fn(nsubdivs + 1, |i, _| {
        if i == 0 || i == nsubdivs {
            80.0
        } else {
            (i as f32 * ground_size.x / (nsubdivs as f32)).cos() * 2.0
        }
    });

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::new_heightfield(heights, ground_size).build();
    colliders.insert(&mut bodies, collider, handle);

    /*
     * Create the cubes
     */
    let num = 26;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 3.0;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y).build();
            let handle = bodies.insert(rigid_body);

            if j % 2 == 0 {
                let collider = ColliderBuilder::new_cuboid(rad, rad).density(1.0).build();
                colliders.insert(&mut bodies, collider, handle);
            } else {
                let collider = ColliderBuilder::new_ball(rad).density(1.0).build();
                colliders.insert(&mut bodies, collider, handle);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point2::new(0.0, 50.0), 10.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Heightfield", init_world)]);
    testbed.run()
}
