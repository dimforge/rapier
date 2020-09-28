use na::{Point3, Translation3, UnitQuaternion, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

fn create_tower_circle(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector3<f32>,
    stack_height: usize,
    nsubdivs: usize,
    half_extents: Vector3<f32>,
) {
    let ang_step = std::f32::consts::PI * 2.0 / nsubdivs as f32;
    let radius = 1.3 * nsubdivs as f32 * half_extents.x / std::f32::consts::PI;

    let shift = half_extents * 2.0;
    for i in 0usize..stack_height {
        for j in 0..nsubdivs {
            let fj = j as f32;
            let fi = i as f32;
            let y = fi * shift.y;
            let pos = Translation3::from(offset)
                * UnitQuaternion::new(Vector3::y() * (fi / 2.0 + fj) * ang_step)
                * Translation3::new(0.0, y, radius);

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().position(pos).build();
            let handle = bodies.insert(rigid_body);
            let collider =
                ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z).build();
            colliders.insert(collider, handle, bodies);
        }
    }
}

fn create_wall(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector3<f32>,
    stack_height: usize,
    half_extents: Vector3<f32>,
) {
    let shift = half_extents * 2.0;
    for i in 0usize..stack_height {
        for j in i..stack_height {
            let fj = j as f32;
            let fi = i as f32;
            let x = offset.x;
            let y = fi * shift.y + offset.y;
            let z = (fi * shift.z / 2.0) + (fj - fi) * shift.z + offset.z
                - stack_height as f32 * half_extents.z;

            // Build the rigid body.
            let rigid_body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
            let handle = bodies.insert(rigid_body);
            let collider =
                ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z).build();
            colliders.insert(collider, handle, bodies);
        }
    }
}

fn create_pyramid(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    offset: Vector3<f32>,
    stack_height: usize,
    half_extents: Vector3<f32>,
) {
    let shift = half_extents * 2.0;

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
                let handle = bodies.insert(rigid_body);
                let collider =
                    ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z).build();
                colliders.insert(collider, handle, bodies);
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
    let ground_size = 200.0;
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
    let cube_size = 1.0;
    let hext = Vector3::repeat(cube_size);
    let bottomy = cube_size * 50.0;
    create_pyramid(
        &mut bodies,
        &mut colliders,
        Vector3::new(-110.0, bottomy, 0.0),
        12,
        hext,
    );
    create_pyramid(
        &mut bodies,
        &mut colliders,
        Vector3::new(-80.0, bottomy, 0.0),
        12,
        hext,
    );
    create_pyramid(
        &mut bodies,
        &mut colliders,
        Vector3::new(-50.0, bottomy, 0.0),
        12,
        hext,
    );
    create_pyramid(
        &mut bodies,
        &mut colliders,
        Vector3::new(-20.0, bottomy, 0.0),
        12,
        hext,
    );
    create_wall(
        &mut bodies,
        &mut colliders,
        Vector3::new(-2.0, bottomy, 0.0),
        12,
        hext,
    );
    create_wall(
        &mut bodies,
        &mut colliders,
        Vector3::new(4.0, bottomy, 0.0),
        12,
        hext,
    );
    create_wall(
        &mut bodies,
        &mut colliders,
        Vector3::new(10.0, bottomy, 0.0),
        12,
        hext,
    );
    create_tower_circle(
        &mut bodies,
        &mut colliders,
        Vector3::new(25.0, bottomy, 0.0),
        8,
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
