//! A dynamic body driven by the `PidController` component toward a target pose moved with the
//! keyboard.
//!
//! Controls: WASD/arrows move the target horizontally, Q/E move it down/up, Z/X rotate it, and
//! Space toggles the control of the vertical axis (letting gravity take over).

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const TARGET_SPEED: f32 = 5.0;
const TARGET_ANGULAR_SPEED: f32 = 1.5;

#[derive(Resource)]
struct Target(Transform);

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Target(Transform::from_xyz(0.0, 3.0, 0.0)))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            (move_target, update_controller, draw_target).chain(),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 12.0, 20.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    commands.spawn((
        Transform::from_xyz(0.0, -0.1, 0.0),
        Collider::cuboid(20.0, 0.1, 20.0),
    ));

    /*
     * Some boxes to push around.
     */
    for i in 0..5 {
        for j in 0..2 {
            commands.spawn((
                Transform::from_xyz(i as f32 * 2.0 - 4.0, 0.5 + j as f32, -4.0),
                RigidBody::Dynamic,
                Collider::cuboid(0.5, 0.5, 0.5),
            ));
        }
    }

    /*
     * The controlled body, using the same default gains as Rapier's PID controller.
     */
    commands.spawn((
        Transform::from_xyz(0.0, 1.0, 4.0),
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 0.25, 0.5),
        PidController::default(),
        ColliderDebugColor(Hsla::hsl(20.0, 1.0, 0.5)),
    ));
}

fn move_target(keyboard: Res<ButtonInput<KeyCode>>, time: Res<Time>, mut target: ResMut<Target>) {
    let axis = |neg: &[KeyCode], pos: &[KeyCode]| {
        let pressed = |keys: &[KeyCode]| keyboard.any_pressed(keys.iter().copied()) as i32;
        (pressed(pos) - pressed(neg)) as f32
    };
    let direction = Vec3::new(
        axis(
            &[KeyCode::KeyA, KeyCode::ArrowLeft],
            &[KeyCode::KeyD, KeyCode::ArrowRight],
        ),
        axis(&[KeyCode::KeyQ], &[KeyCode::KeyE]),
        axis(
            &[KeyCode::KeyW, KeyCode::ArrowUp],
            &[KeyCode::KeyS, KeyCode::ArrowDown],
        ),
    );
    let yaw = axis(&[KeyCode::KeyX], &[KeyCode::KeyZ]);

    let dt = time.delta_secs();
    let target = &mut target.0;
    target.translation += direction.normalize_or_zero() * TARGET_SPEED * dt;
    target.translation.y = target.translation.y.max(0.5);
    target.rotate_y(yaw * TARGET_ANGULAR_SPEED * dt);
}

fn update_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    target: Res<Target>,
    mut controllers: Query<&mut PidController>,
) {
    for mut controller in &mut controllers {
        controller.target = PidTarget::new(target.0.translation, target.0.rotation);

        if keyboard.just_pressed(KeyCode::Space) {
            controller.axes.toggle(AxesMask::LIN_Y);
            // Forget the error accumulated while fighting gravity.
            controller.reset_integrals();
            info!(
                "Vertical axis control: {}",
                controller.axes.contains(AxesMask::LIN_Y)
            );
        }
    }
}

fn draw_target(
    mut gizmos: Gizmos,
    target: Res<Target>,
    controlled: Query<&GlobalTransform, With<PidController>>,
) {
    gizmos.axes(target.0, 1.5);
    for transform in &controlled {
        gizmos.line(
            transform.translation(),
            target.0.translation,
            Color::srgb(0.9, 0.2, 0.2),
        );
    }
}
