//! Collision filtering with `CollisionGroups`, including a group relying on
//! `InteractionTestMode::Or`.
//!
//! Green cubes only collide with the green floor and blue cubes only with the blue floor. Orange
//! cubes fall through both and land on the orange floor: their memberships match the floor's
//! filter but not the other way around, which is enough since both use `InteractionTestMode::Or`.

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

const GREEN_GROUP: CollisionGroups = CollisionGroups::new(Group::GROUP_1, Group::GROUP_1);
const BLUE_GROUP: CollisionGroups = CollisionGroups::new(Group::GROUP_2, Group::GROUP_2);
// The orange floor's memberships are not in the orange cubes' filter: with the default
// `InteractionTestMode::And`, they would never interact.
const ORANGE_GROUP: CollisionGroups =
    CollisionGroups::new(Group::GROUP_3, Group::GROUP_1.union(Group::GROUP_3))
        .with_test_mode(InteractionTestMode::Or);
const ORANGE_FLOOR_GROUP: CollisionGroups =
    CollisionGroups::new(Group::GROUP_4, Group::GROUP_3).with_test_mode(InteractionTestMode::Or);

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
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(4.0, 4.0, 4.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    let green = Hsla::hsl(120.0, 1.0, 0.35);
    let blue = Hsla::hsl(220.0, 1.0, 0.4);
    let orange = Hsla::hsl(30.0, 1.0, 0.5);

    /*
     * Ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * Floors colliding only with the cubes of their color.
     */
    let floors = [
        (0.5, ORANGE_FLOOR_GROUP, orange),
        (1.0, GREEN_GROUP, green),
        (2.0, BLUE_GROUP, blue),
    ];

    for (y, groups, color) in floors {
        commands.spawn((
            Transform::from_xyz(0.0, y, 0.0),
            Collider::cuboid(1.0, 0.1, 1.0),
            groups,
            ColliderDebugColor(color),
        ));
    }

    /*
     * Create the cubes
     */
    let num = 9;
    let rad = 0.1;

    let shift = rad * 2.0;
    let centerx = shift * (num / 2) as f32;
    let centery = 2.5;
    let centerz = shift * (num / 2) as f32;

    for j in 0usize..4 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery;
                let z = k as f32 * shift - centerz;

                // Cycle through the green, blue, and orange groups.
                let (groups, color) = match k % 3 {
                    0 => (GREEN_GROUP, green),
                    1 => (BLUE_GROUP, blue),
                    _ => (ORANGE_GROUP, orange),
                };

                commands.spawn((
                    Transform::from_xyz(x, y, z),
                    RigidBody::Dynamic,
                    Collider::cuboid(rad, rad, rad),
                    groups,
                    ColliderDebugColor(color),
                ));
            }
        }
    }
}
