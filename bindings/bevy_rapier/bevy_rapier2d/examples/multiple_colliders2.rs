use bevy::prelude::*;
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
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2d);
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 500.0;
    let ground_height = 1.0;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height),
    ));

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 2.0;

    let shift = rad * 4.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;

    for j in 0usize..20 {
        for i in 0..num {
            let x = i as f32 * shift * 5.0 - centerx + offset;
            let y = j as f32 * (shift * 5.0) + centery + 3.0;

            commands
                .spawn((Transform::from_xyz(x, y, 0.0), RigidBody::Dynamic))
                .with_children(|children| {
                    children.spawn(Collider::cuboid(rad * 10.0, rad));
                    children.spawn((
                        Transform::from_xyz(rad * 10.0, rad * 10.0, 0.0),
                        Collider::cuboid(rad, rad * 10.0),
                    ));
                    children.spawn((
                        Transform::from_xyz(-rad * 10.0, rad * 10.0, 0.0),
                        Collider::cuboid(rad, rad * 10.0),
                    ));
                });
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
