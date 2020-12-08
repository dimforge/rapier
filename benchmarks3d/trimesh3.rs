use na::{ComplexField, DMatrix, Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, HeightField};
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
    let ground_size = Vector3::new(200.0, 1.0, 200.0);
    let nsubdivs = 20;

    let heights = DMatrix::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        if i == 0 || i == nsubdivs || j == 0 || j == nsubdivs {
            10.0
        } else {
            let x = i as f32 * ground_size.x / (nsubdivs as f32);
            let z = j as f32 * ground_size.z / (nsubdivs as f32);

            // NOTE: make sure we use the sin/cos from simba to ensure
            // cross-platform determinism of the example when the
            // enhanced_determinism feature is enabled.
            <f32 as ComplexField>::sin(x) + <f32 as ComplexField>::cos(z)
        }
    });

    // Here we will build our trimesh from the mesh representation of an
    // heightfield.
    let heightfield = HeightField::new(heights, ground_size);
    let (vertices, indices) = heightfield.to_trimesh();

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::trimesh(vertices, indices).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    for j in 0usize..47 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                let handle = bodies.insert(rigid_body);

                if j % 2 == 0 {
                    let collider = ColliderBuilder::cuboid(rad, rad, rad).build();
                    colliders.insert(collider, handle, &mut bodies);
                } else {
                    let collider = ColliderBuilder::ball(rad).build();
                    colliders.insert(collider, handle, &mut bodies);
                }
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
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
