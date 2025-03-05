use crate::utils::character::{self, CharacterControlMode};
use rapier3d::control::{KinematicCharacterController, PidController};
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

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
    let ground_size = 0.75;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![0.0, -ground_height, 0.0]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![
        -ground_size - ground_height,
        ground_height,
        0.0
    ]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_height, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![
        ground_size + ground_height,
        ground_height,
        0.0
    ]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_height, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![
        0.0,
        ground_height,
        -ground_size - ground_height
    ]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_height);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    let rigid_body = RigidBodyBuilder::fixed().translation(vector![
        0.0,
        ground_height,
        ground_size + ground_height
    ]);
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_height);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Character we will control manually.
     */

    let rigid_body =
        RigidBodyBuilder::kinematic_position_based().translation(vector![0.0, 0.3, 0.0]);
    let character_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(0.15, 0.3, 0.15);
    colliders.insert_with_parent(collider, character_handle, &mut bodies);

    testbed.set_initial_body_color(character_handle, [1., 131. / 255., 244.0 / 255.]);

    /*
     * Tethered Ball
     */
    let rad = 0.04;

    let rigid_body =
        RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(vector![1.0, 1.0, 0.0]);
    let child_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::ball(rad);
    colliders.insert_with_parent(collider, child_handle, &mut bodies);

    let joint = RopeJointBuilder::new(2.0);
    impulse_joints.insert(character_handle, child_handle, joint, true);

    /*
     * Callback to update the character based on user inputs.
     */
    let mut control_mode = CharacterControlMode::Kinematic;
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
    testbed.look_at(point![10.0, 10.0, 10.0], point![0.0, 0.0, 0.0]);
}
