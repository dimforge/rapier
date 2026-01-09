use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

// This shows a bug when a cylinder is in contact with a very large
// but very thin cuboid. In this case the EPA returns an incorrect
// contact normal, resulting in the cylinder falling through the floor.
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
    // let vertices = vec![
    //     Vec3::new(-50.0, 0.0, -50.0),
    //     Vec3::new(-50.0, 0.0, 50.0),
    //     Vec3::new(50.0, 0.0, 50.0),
    //     Vec3::new(50.0, 0.0, -50.0),
    // ];
    // let indices = vec![[0, 1, 2], [0, 2, 3]];
    //
    // let collider = ColliderBuilder::trimesh_with_flags(vertices, indices, TriMeshFlags::all());
    // colliders.insert(collider);

    let heights = Array2::repeat(2, 2, 0.0);
    let collider = ColliderBuilder::heightfield_with_flags(
        heights,
        Vector::new(50.0, 1.0, 50.0),
        HeightFieldFlags::FIX_INTERNAL_EDGES,
    );
    colliders.insert(collider);

    /*
     * Create the cubes
     */
    // Build the rigid body.
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 5.0, 0.0))
        .rotation(Vector::new(0.5, 0.0, 0.5))
        .linvel(Vector::new(0.0, -100.0, 0.0))
        .soft_ccd_prediction(10.0);
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(5.0, 0.015, 5.0);
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(100.0, 100.0, 100.0), Vec3::ZERO);
}
