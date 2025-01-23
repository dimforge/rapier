use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body_floor = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height]);
    let floor_handle = bodies.insert(rigid_body_floor);
    let floor_collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(floor_collider, floor_handle, &mut bodies);

    /*
     * Character we will control manually.
     */
    let rigid_body_character =
        RigidBodyBuilder::kinematic_position_based().translation(vector![0.0, 0.3]);
    let character_handle = bodies.insert(rigid_body_character);
    let character_collider = ColliderBuilder::cuboid(0.15, 0.3);
    colliders.insert_with_parent(character_collider, character_handle, &mut bodies);

    /*
     * Tethered cube.
     */
    let rad = 0.4;

    let rigid_body_cube =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(vector![1.0, 1.0]);
    let cube_handle = bodies.insert(rigid_body_cube);
    let cube_collider = ColliderBuilder::cuboid(rad, rad);
    colliders.insert_with_parent(cube_collider, cube_handle, &mut bodies);

    /*
     * Rotation axis indicator ball.
     */
    let rigid_body_ball =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(vector![1.0, 1.0]);
    let ball_handle = bodies.insert(rigid_body_ball);
    let ball_collider = ColliderBuilder::ball(0.1);
    colliders.insert_with_parent(ball_collider, ball_handle, &mut bodies);

    /*
     * Fixed joint between rotation axis indicator and cube.
     */
    let fixed_joint = FixedJointBuilder::new()
        .local_anchor1(point![0.0, 0.0])
        .local_anchor2(point![0.0, -0.4])
        .build();
    impulse_joints.insert(cube_handle, ball_handle, fixed_joint, true);

    /*
     * Pin slot joint between cube and ground.
     */
    let axis: nalgebra::Unit<
        nalgebra::Matrix<
            f32,
            nalgebra::Const<2>,
            nalgebra::Const<1>,
            nalgebra::ArrayStorage<f32, 2, 1>,
        >,
    > = UnitVector::new_normalize(vector![1.0, 1.0]);
    let pin_slot_joint = PinSlotJointBuilder::new(axis)
        .local_anchor1(point![2.0, 2.0])
        .local_anchor2(point![0.0, 0.4])
        .limits([-1.0, f32::INFINITY]) // Set the limits for the pin slot joint
        .build();
    impulse_joints.insert(character_handle, cube_handle, pin_slot_joint, true);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.set_character_body(character_handle);
    testbed.look_at(point![0.0, 1.0], 100.0);
}
