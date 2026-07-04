use crate::utils::character::{self, CharacterControlMode};
use kiss3d::color::Color;
use rapier_testbed3d::TestbedViewer;
use rapier3d::control::{KinematicCharacterController, PidController};
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    /*
     * Ground
     */
    let ground_size = 0.75;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(
        -ground_size - ground_height,
        ground_height,
        0.0,
    ));
    let collider = ColliderBuilder::cuboid(ground_height, ground_height, ground_size);
    world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(
        ground_size + ground_height,
        ground_height,
        0.0,
    ));
    let collider = ColliderBuilder::cuboid(ground_height, ground_height, ground_size);
    world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(
        0.0,
        ground_height,
        -ground_size - ground_height,
    ));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_height);
    world.insert(rigid_body, collider);

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(
        0.0,
        ground_height,
        ground_size + ground_height,
    ));
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_height);
    world.insert(rigid_body, collider);

    /*
     * Character we will control manually.
     */

    let rigid_body =
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 0.3, 0.0));
    let collider = ColliderBuilder::cuboid(0.15, 0.3, 0.15);
    let (character_handle, _) = world.insert(rigid_body, collider);

    viewer.set_initial_body_color(
        character_handle,
        Color::new(1., 131. / 255., 244.0 / 255., 1.0),
    );

    /*
     * Tethered Ball
     */
    let rad = 0.04;

    let rigid_body =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(Vector::new(1.0, 1.0, 0.0));
    let collider = ColliderBuilder::ball(rad);
    let (child_handle, _) = world.insert(rigid_body, collider);

    let joint = RopeJointBuilder::new(2.0);
    world.insert_impulse_joint(character_handle, child_handle, joint);

    /*
     * State to update the character based on user inputs.
     */
    let mut control_mode = CharacterControlMode::Kinematic(0.1);
    let mut controller = KinematicCharacterController::default();
    let mut pid = PidController::default();

    /*
     * Set up the testbed.
     */
    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::new(0.0, 0.0, 0.0));

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
