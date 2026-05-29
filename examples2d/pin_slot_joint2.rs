use crate::utils::character;
use crate::utils::character::CharacterControlMode;
use rapier_testbed2d::Testbed;
use rapier2d::control::{KinematicCharacterController, PidController};
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 3.0;
    let ground_height = 0.1;

    let rigid_body_floor = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height));
    let floor_collider = ColliderBuilder::cuboid(ground_size, ground_height);
    let _ = world.insert(rigid_body_floor, floor_collider);

    /*
     * Character we will control manually.
     */
    let rigid_body_character =
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 0.3));
    let character_collider = ColliderBuilder::cuboid(0.15, 0.3);
    let (character_handle, _) = world.insert(rigid_body_character, character_collider);

    /*
     * Tethered cube.
     */
    let rad = 0.4;

    let rigid_body_cube =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(Vector::new(1.0, 1.0));
    let cube_collider = ColliderBuilder::cuboid(rad, rad);
    let (cube_handle, _) = world.insert(rigid_body_cube, cube_collider);

    /*
     * SimdRotation axis indicator ball.
     */
    let rigid_body_ball =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(Vector::new(1.0, 1.0));
    let ball_collider = ColliderBuilder::ball(0.1);
    let (ball_handle, _) = world.insert(rigid_body_ball, ball_collider);

    /*
     * Fixed joint between rotation axis indicator and cube.
     */
    let fixed_joint = FixedJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 0.0))
        .local_anchor2(Vector::new(0.0, -0.4))
        .build();
    world.insert_impulse_joint(cube_handle, ball_handle, fixed_joint);

    /*
     * Pin slot joint between cube and ground.
     */
    let axis = Vector::new(1.0, 1.0).normalize();
    let pin_slot_joint = PinSlotJointBuilder::new(axis)
        .local_anchor1(Vector::new(2.0, 2.0))
        .local_anchor2(Vector::new(0.0, 0.4))
        .limits([-1.0, f32::INFINITY]) // Set the limits for the pin slot joint
        .build();
    world.insert_impulse_joint(character_handle, cube_handle, pin_slot_joint);

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
    testbed.set_physics_world(world);
    testbed.look_at(Vec2::new(0.0, 1.0), 100.0);
}
