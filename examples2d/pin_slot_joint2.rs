use crate::utils::character;
use crate::utils::character::CharacterControlMode;
use rapier_testbed2d::Testbed;
use rapier2d::control::{KinematicCharacterController, PidController};
use rapier2d::prelude::*;

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

    let rigid_body_floor = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let floor_handle = bodies.insert(rigid_body_floor);
    let floor_collider = ColliderBuilder::cuboid(ground_size, ground_height);
    colliders.insert_with_parent(floor_collider, floor_handle, &mut bodies);

    /*
     * Character we will control manually.
     */
    let rigid_body_character =
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 0.3));
    let character_handle = bodies.insert(rigid_body_character);
    let character_collider = ColliderBuilder::cuboid(0.15, 0.3);
    colliders.insert_with_parent(character_collider, character_handle, &mut bodies);

    /*
     * Tethered cube.
     */
    let rad = 0.4;

    let rigid_body_cube =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(Vector::new(1.0, 1.0));
    let cube_handle = bodies.insert(rigid_body_cube);
    let cube_collider = ColliderBuilder::cuboid(rad, rad);
    colliders.insert_with_parent(cube_collider, cube_handle, &mut bodies);

    /*
     * SimdRotation axis indicator ball.
     */
    let rigid_body_ball =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(Vector::new(1.0, 1.0));
    let ball_handle = bodies.insert(rigid_body_ball);
    let ball_collider = ColliderBuilder::ball(0.1);
    colliders.insert_with_parent(ball_collider, ball_handle, &mut bodies);

    /*
     * Fixed joint between rotation axis indicator and cube.
     */
    let fixed_joint = FixedJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 0.0))
        .local_anchor2(Vector::new(0.0, -0.4))
        .build();
    impulse_joints.insert(cube_handle, ball_handle, fixed_joint, true);

    /*
     * Pin slot joint between cube and ground.
     */
    let axis = Vector::new(1.0, 1.0).normalize();
    let pin_slot_joint = PinSlotJointBuilder::new(axis)
        .local_anchor1(Vector::new(2.0, 2.0))
        .local_anchor2(Vector::new(0.0, 0.4))
        .limits([-1.0, f32::INFINITY]) // Set the limits for the pin slot joint
        .build();
    impulse_joints.insert(character_handle, cube_handle, pin_slot_joint, true);

    /*
     * Callback to update the character based on user inputs.
     */
    let mut control_mode = CharacterControlMode::Kinematic(0.1);
    let mut controller = KinematicCharacterController::default();
    let mut pid = PidController::default();

    testbed.add_callback(move |graphics, physics, _, _| {
        if let Some(graphics) = graphics {
            character::update_character(
                graphics,
                physics,
                &mut control_mode,
                &mut controller,
                &mut pid,
                character_handle,
            );
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec2::new(0.0, 1.0), 100.0);
}
