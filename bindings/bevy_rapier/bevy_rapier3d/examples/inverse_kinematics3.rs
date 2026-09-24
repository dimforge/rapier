//! Multibody inverse kinematics: the end of a chain of ten spherical multibody joints follows the
//! mouse cursor projected on a camera-facing plane (or a circle while the cursor is outside of the
//! window).

use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_rapier3d::prelude::*;

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
struct IkTarget(Vec3);

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.5, 2.5).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground.
     */
    let ground_size = 0.2;
    let ground_height = 0.01;
    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * The chain, attached to a fixed root with spherical multibody joints.
     */
    let num_segments = 10;
    let size = 1.0 / num_segments as f32;
    let mut last_body = commands
        .spawn((Transform::default(), RigidBody::Fixed))
        .id();

    for i in 0..num_segments {
        let joint = SphericalJointBuilder::new()
            .local_anchor1(Vec3::new(0.0, if i == 0 { 0.0 } else { size / 2.0 }, 0.0))
            .local_anchor2(Vec3::new(0.0, -size / 2.0, 0.0));

        // The sensor collider is only here to let the debug-renderer draw the links.
        last_body = commands
            .spawn((
                Transform::from_xyz(0.0, (i as f32 + 0.5) * size, 0.0),
                RigidBody::Dynamic,
                Sleeping::disabled(),
                Collider::cuboid(size / 8.0, size / 2.0, size / 8.0),
                ColliderMassProperties::Density(0.0),
                Sensor,
                MultibodyJoint::new(last_body, joint),
            ))
            .id();
    }

    commands.insert_resource(EndEffector(last_body));
    commands.insert_resource(IkTarget::default());
}

/// Moves the target to the intersection between the mouse ray and the camera-facing plane
/// passing through the origin, or along a circle if the cursor isn’t in the window.
fn update_target(
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut target: ResMut<IkTarget>,
    mut gizmos: Gizmos,
) {
    let (camera, camera_transform) = *camera;
    let plane = InfinitePlane3d::new(camera_transform.back());
    let cursor = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor).ok())
        .and_then(|ray| Some(ray.get_point(ray.intersect_plane(Vec3::ZERO, plane)?)));

    target.0 = cursor.unwrap_or_else(|| {
        let t = time.elapsed_secs();
        Vec3::new(0.0, 0.5, 0.0) + 0.4 * Vec3::new(t.cos(), t.sin(), (0.5 * t).sin())
    });
    gizmos.sphere(
        Isometry3d::from_translation(target.0),
        0.02,
        Color::srgb(0.9, 0.2, 0.2),
    );
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
        Transform::from_translation(target.0),
        &options,
        |_| true,
        &mut displacements,
    ) {
        context.multibody_apply_displacements(end_effector.0, &displacements);
    }

    Ok(())
}
