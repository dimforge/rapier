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
    let ground_size = 200.0f32;
    let ground_height = 1.0;
    let nsubdivs = 20;

    let quad = rapier3d::ncollide::procedural::quad(ground_size, ground_size, nsubdivs, nsubdivs);
    let indices = quad
        .flat_indices()
        .chunks(3)
        .map(|is| Point3::new(is[0], is[2], is[1]))
        .collect();
    let mut vertices = quad.coords;

    // ncollide generates a quad with `z` as the normal.
    // so we switch z and y here and set a random altitude at each point.
    for p in vertices.iter_mut() {
        p.z = p.y;
        p.y = (p.x.sin() + p.z.cos()) * ground_height;

        if p.x.abs() == ground_size / 2.0 || p.z.abs() == ground_size / 2.0 {
            p.y = 10.0;
        }
    }

    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::new_trimesh(vertices, indices).build();
    colliders.insert(&mut bodies, collider, handle);

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
                    let collider = ColliderBuilder::new_cuboid(rad, rad, rad)
                        .density(1.0)
                        .build();
                    colliders.insert(&mut bodies, collider, handle);
                } else {
                    let collider = ColliderBuilder::new_ball(rad).density(1.0).build();
                    colliders.insert(&mut bodies, collider, handle);
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
