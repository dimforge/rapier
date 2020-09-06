use na::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

fn create_pyramid(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector3<f32>,
    stack_height: usize,
    half_extents: Vector3<f32>,
) {
    let shift = half_extents * 2.5;
    for i in 0usize..stack_height {
        for j in i..stack_height {
            for k in i..stack_height {
                let fi = i as f32;
                let fj = j as f32;
                let fk = k as f32;
                let x = (fi * shift.x / 2.0) + (fk - fi) * shift.x + offset.x
                    - stack_height as f32 * half_extents.x;
                let y = fi * shift.y + offset.y;
                let z = (fi * shift.z / 2.0) + (fj - fi) * shift.z + offset.z
                    - stack_height as f32 * half_extents.z;

                // Build the rigid body.
                let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                let rigid_body_handle = bodies.insert(rigid_body);

                let collider =
                    ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z).build();
                colliders.insert(collider, rigid_body_handle, bodies);
            }
        }
    }
}

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
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, ground_handle, &mut bodies);

    /*
     * Create the cubes
     */
    let cube_size = 1.0;
    let hext = Vector3::repeat(cube_size);
    let bottomy = cube_size;
    create_pyramid(
        &mut bodies,
        &mut colliders,
        Vector3::new(0.0, bottomy, 0.0),
        24,
        hext,
    );

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
