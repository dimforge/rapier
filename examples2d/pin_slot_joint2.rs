use crate::utils::character;
use crate::utils::character::CharacterControlMode;
use rapier_testbed2d::TestbedViewer;
use rapier2d::control::{KinematicCharacterController, PidController};
use rapier2d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
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
    let character_pos = Vector::new(0.0, 0.3);
    let rigid_body_character =
        RigidBodyBuilder::kinematic_position_based().translation(character_pos);
    let character_collider = ColliderBuilder::cuboid(0.15, 0.3);
    let (character_handle, _) = world.insert(rigid_body_character, character_collider);

    /*
     * Tethered cube.
     */
    let rad = 0.4;
    let slot_axis = Vector::new(1.0, 1.0).normalize();
    let slot_anchor = Vector::new(2.0, 2.0);
    let slot_limits = [-1.0, f32::INFINITY];
    // Start the cube on the slot's lower limit so the joint doesn't have to push it back.
    let cube_pos = character_pos + slot_anchor + slot_axis * slot_limits[0] - Vector::new(0.0, rad);

    let rigid_body_cube = RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(cube_pos);
    let cube_collider = ColliderBuilder::cuboid(rad, rad);
    let (cube_handle, _) = world.insert(rigid_body_cube, cube_collider);

    /*
     * SimdRotation axis indicator ball.
     */
    let rigid_body_ball =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(cube_pos + Vector::new(0.0, rad));
    let ball_collider = ColliderBuilder::ball(0.1);
    let (ball_handle, _) = world.insert(rigid_body_ball, ball_collider);

    /*
     * Fixed joint between rotation axis indicator and cube.
     */
    let fixed_joint = FixedJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 0.0))
        .local_anchor2(Vector::new(0.0, -rad))
        .build();
    world.insert_impulse_joint(cube_handle, ball_handle, fixed_joint);

    /*
     * Pin slot joint between cube and ground.
     */
    let pin_slot_joint = PinSlotJointBuilder::new(slot_axis)
        .local_anchor1(slot_anchor)
        .local_anchor2(Vector::new(0.0, rad))
        .limits(slot_limits) // Set the limits for the pin slot joint
        .build();
    world.insert_impulse_joint(character_handle, cube_handle, pin_slot_joint);

    /*
     * Callback to update the character based on user inputs.
     */
    let mut control_mode = CharacterControlMode::Kinematic(0.1);
    let mut controller = KinematicCharacterController::default();
    let mut pid = PidController::default();

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec2::new(0.0, 1.0), 100.0);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
            character::update_character(
                viewer,
                &mut world,
                &mut control_mode,
                &mut controller,
                &mut pid,
                character_handle,
            );
        }
    }
    Ok(())
}
