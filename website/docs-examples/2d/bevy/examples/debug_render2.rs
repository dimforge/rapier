use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        // DOCUSAURUS: DebugRenderPlugin start
        .add_plugins(RapierDebugRenderPlugin {
            // Only draw the collider shapes and the joints.
            mode: DebugRenderMode::COLLIDER_SHAPES
                | DebugRenderMode::IMPULSE_JOINTS
                | DebugRenderMode::MULTIBODY_JOINTS,
            ..default()
        })
        // DOCUSAURUS: DebugRenderPlugin stop
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(
            Update,
            modify_debug_render.run_if(input_just_pressed(KeyCode::KeyD)),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2d);
}

// DOCUSAURUS: DebugRenderContext start
fn modify_debug_render(mut debug_render: ResMut<DebugRenderContext>) {
    // Toggle the debug-renderer.
    debug_render.enabled = !debug_render.enabled;
    // Draw the contacts and the AABBs too.
    debug_render.mode.contacts = true;
    debug_render.mode.collider_aabbs = true;
    // Expressed in meters: multiplied by the length unit (here, 100 pixels per meter).
    debug_render.style.rigid_body_axes_length = 1.0;
}
// DOCUSAURUS: DebugRenderContext stop

fn setup_physics(mut commands: Commands) {
    commands.spawn((
        Transform::from_xyz(0.0, -100.0, 0.0),
        Collider::cuboid(500.0, 50.0),
    ));

    // DOCUSAURUS: DebugRenderOverrides start
    // This collider is drawn in red.
    commands.spawn((
        Transform::from_xyz(0.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::ball(50.0),
        ColliderDebugColor(Hsla::hsl(0.0, 1.0, 0.5)),
    ));
    // Everything attached to this entity (its collider, its rigid-body, its joint, etc.) is
    // drawn in blue.
    commands.spawn((
        Transform::from_xyz(200.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(50.0, 50.0),
        DebugRenderColor(Hsla::hsl(220.0, 1.0, 0.3)),
    ));
    // Nothing attached to this entity is drawn.
    commands.spawn((
        Transform::from_xyz(-200.0, 400.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(50.0, 50.0),
        DebugRenderVisibility::Hidden,
    ));
    // The shape of this collider is never drawn, whatever the `default_collider_debug`.
    commands.spawn((
        Transform::from_xyz(-400.0, 400.0, 0.0),
        Collider::cuboid(50.0, 50.0),
        ColliderDebug::NeverRender,
    ));
    // DOCUSAURUS: DebugRenderOverrides stop
}
