//! Multibody inverse kinematics: the end of a chain of ten revolute multibody joints follows the
//! mouse cursor (or a circle while the cursor is outside of the window).

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (update_target, track_target).chain())
        .run();
}

/// The last link of the chain, driven by the inverse kinematics solver.
#[derive(Resource)]
struct EndEffector(Entity);

/// The world-space point the end effector tries to reach.
#[derive(Resource, Default)]
struct IkTarget(Vec2);

pub fn setup_graphics(mut commands: Commands) {
    // The scene is expressed in meters: zoom in so that one meter spans 300 pixels.
    commands.spawn((
        Camera2d,
        Projection::from(OrthographicProjection {
            scale: 1.0 / 300.0,
            ..OrthographicProjection::default_2d()
        }),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground.
     */
    let ground_size = 1.0;
    let ground_height = 0.01;
    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    /*
     * The chain, attached to a fixed root with revolute multibody joints.
     */
    let num_segments = 10;
    let size = 1.0 / num_segments as f32;
    let mut last_body = commands
        .spawn((Transform::default(), RigidBody::Fixed))
        .id();

    for i in 0..num_segments {
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vec2::new(0.0, if i == 0 { 0.0 } else { size / 2.0 }))
            .local_anchor2(Vec2::new(0.0, -size / 2.0));

        // The sensor collider is only here to let the debug-renderer draw the links.
        last_body = commands
            .spawn((
                Transform::from_xyz(0.0, (i as f32 + 0.5) * size, 0.0),
                RigidBody::Dynamic,
                Sleeping::disabled(),
                Collider::cuboid(size / 8.0, size / 2.0),
                ColliderMassProperties::Density(0.0),
                Sensor,
                MultibodyJoint::new(last_body, joint),
            ))
            .id();
    }

    commands.insert_resource(EndEffector(last_body));
    commands.insert_resource(IkTarget::default());
}

/// Moves the target to the mouse cursor, or along a circle if the cursor isn’t in the window.
fn update_target(
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut target: ResMut<IkTarget>,
    mut gizmos: Gizmos,
) {
    let (camera, camera_transform) = *camera;
    let cursor = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world_2d(camera_transform, cursor).ok());

    target.0 = cursor.unwrap_or_else(|| {
        let t = time.elapsed_secs();
        Vec2::new(0.0, 0.5) + 0.4 * Vec2::new(t.cos(), t.sin())
    });
    gizmos.circle_2d(target.0, 0.02, Color::srgb(0.9, 0.2, 0.2));
}

/// Runs the inverse kinematics solver on the chain and applies the resulting displacements.
fn track_target(
    mut context: WriteRapierContext,
    end_effector: Res<EndEffector>,
    target: Res<IkTarget>,
    mut displacements: Local<Vec<f32>>,
) -> Result<()> {
    let options = InverseKinematicsOption {
        constrained_axes: JointAxesMask::LIN_AXES,
        ..Default::default()
    };

    // The displacements buffer is reset by the solver, so it can be reused across frames.
    let mut context = context.single_mut()?;
    if context.multibody_inverse_kinematics(
        end_effector.0,
        Transform::from_translation(target.0.extend(0.0)),
        &options,
        |_| true,
        &mut displacements,
    ) {
        context.multibody_apply_displacements(end_effector.0, &displacements);
    }

    Ok(())
}
