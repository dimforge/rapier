use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    let heights = Array2::zeros(100, 100);
    let heightfield = HeightField::with_flags(
        heights,
        Vector::new(60.0, 1.0, 60.0),
        HeightFieldFlags::all(),
    );
    let rotation = Vector::new(0.0, 0.0, 0.0); // Vector::new(-0.1, 0.0, 0.0);
    colliders
        .insert(ColliderBuilder::new(SharedShape::new(heightfield.clone())).rotation(rotation));

    // let mut trimesh = TriMesh::from(heightfield);
    // trimesh.set_flags(TriMeshFlags::MERGE_DUPLICATE_VERTICES)
    // colliders.insert(ColliderBuilder::new(SharedShape::new(trimesh.clone())).rotation(rotation));
    // // NOTE: we add a sensor just because we want the testbed to display the mesh’s wireframe.
    // colliders.insert(
    //     ColliderBuilder::new(SharedShape::new(trimesh))
    //         .sensor(true)
    //         .rotation(rotation),
    // );

    // Dynamic rigid bodies.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(4.0, 0.5, 0.0))
        .linvel(Vector::new(0.0, -40.0, 20.0))
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(0.5);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(-3.0, 5.0, 0.0))
        .linvel(Vector::new(0.0, -4.0, 20.0))
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(0.5, 0.5, 0.5);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(8.0, 0.2, 0.0))
        .linvel(Vector::new(0.0, -4.0, 20.0))
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cylinder(0.5, 0.2).rotation(Vector::new(
        0.0,
        0.0,
        std::f32::consts::PI / 2.0,
    ));
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
