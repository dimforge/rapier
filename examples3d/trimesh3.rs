use rapier3d::na::ComplexField;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = vector![100.0, 1.0, 100.0];
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

    let rigid_body = RigidBodyBuilder::fixed();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::trimesh_with_flags(
        vertices,
        indices,
        TriMeshFlags::MERGE_DUPLICATE_VERTICES,
    )
    .unwrap();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(rigid_body);

                let collider = match j % 6 {
                    0 => ColliderBuilder::cuboid(rad, rad, rad),
                    1 => ColliderBuilder::ball(rad),
                    // Rounded cylinders are much more efficient that cylinder, even if the
                    // rounding margin is small.
                    2 => ColliderBuilder::round_cylinder(rad, rad, rad / 10.0),
                    3 => ColliderBuilder::cone(rad, rad),
                    4 => ColliderBuilder::capsule_y(rad, rad),
                    _ => {
                        let shapes = vec![
                            (
                                Isometry::identity(),
                                SharedShape::cuboid(rad, rad / 2.0, rad / 2.0),
                            ),
                            (
                                Isometry::translation(rad, 0.0, 0.0),
                                SharedShape::cuboid(rad / 2.0, rad, rad / 2.0),
                            ),
                            (
                                Isometry::translation(-rad, 0.0, 0.0),
                                SharedShape::cuboid(rad / 2.0, rad, rad / 2.0),
                            ),
                        ];

                        ColliderBuilder::compound(shapes)
                    }
                };

                colliders.insert_with_parent(collider, handle, &mut bodies);
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}
