use bevy::prelude::*;
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
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * Create the cubes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    let mut offset = -(num as f32) * (rad * 2.0 + rad) * 0.5;
    let mut color = 0;
    let colors = [
        Hsla::hsl(220.0, 1.0, 0.3),
        Hsla::hsl(180.0, 1.0, 0.3),
        Hsla::hsl(260.0, 1.0, 0.7),
    ];

    for j in 0usize..20 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx + offset;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz + offset;
                color += 1;

                commands
                    .spawn(Transform::from_rotation(Quat::from_rotation_x(0.2)))
                    .with_children(|child| {
                        child.spawn((
                            Transform::from_xyz(x, y, z),
                            RigidBody::Dynamic,
                            Collider::cuboid(rad, rad, rad),
                            ColliderDebugColor(colors[color % 3]),
                        ));
                    });
            }
        }

        offset -= 0.05 * rad * (num as f32 - 1.0);
    }
}
