//! Various shapes falling on a heightfield built with `Collider::heightfield_with_flags` and
//! `HeightFieldFlags::FIX_INTERNAL_EDGES` (preventing bumps at the triangle boundaries).

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

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(60.0, 60.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = Vec3::new(100.0, 1.0, 100.0);
    let nsubdivs = 20;
    let num_rows = nsubdivs + 1;
    let num_cols = nsubdivs + 1;

    // The heights are given in column-major order, with raised borders to keep the shapes in.
    let heights: Vec<f32> = (0..num_cols)
        .flat_map(|j| (0..num_rows).map(move |i| (i, j)))
        .map(|(i, j)| {
            if i == 0 || i == nsubdivs || j == 0 || j == nsubdivs {
                10.0
            } else {
                let x = i as f32 * ground_size.x / nsubdivs as f32;
                let z = j as f32 * ground_size.z / nsubdivs as f32;
                x.sin() + z.cos()
            }
        })
        .collect();

    commands.spawn(Collider::heightfield_with_flags(
        heights,
        num_rows,
        num_cols,
        ground_size,
        HeightFieldFlags::FIX_INTERNAL_EDGES,
    ));

    /*
     * Create the shapes
     */
    let num = 8;
    let rad = 1.0;

    let shift = rad * 2.0 + rad;
    let centerx = shift * (num / 2) as f32;
    let centery = shift / 2.0;
    let centerz = shift * (num / 2) as f32;

    for j in 0usize..6 {
        for i in 0..num {
            for k in 0usize..num {
                let x = i as f32 * shift - centerx;
                let y = j as f32 * shift + centery + 3.0;
                let z = k as f32 * shift - centerz;

                let collider = match j % 6 {
                    0 => Collider::cuboid(rad, rad, rad),
                    1 => Collider::ball(rad),
                    // Rounded cylinders are much more efficient than cylinders, even if the
                    // rounding margin is small.
                    2 => Collider::round_cylinder(rad, rad, rad / 10.0),
                    3 => Collider::cone(rad, rad),
                    4 => Collider::capsule_y(rad, rad),
                    _ => Collider::compound(vec![
                        (
                            Vec3::ZERO,
                            Quat::IDENTITY,
                            Collider::cuboid(rad, rad / 2.0, rad / 2.0),
                        ),
                        (
                            Vec3::new(rad, 0.0, 0.0),
                            Quat::IDENTITY,
                            Collider::cuboid(rad / 2.0, rad, rad / 2.0),
                        ),
                        (
                            Vec3::new(-rad, 0.0, 0.0),
                            Quat::IDENTITY,
                            Collider::cuboid(rad / 2.0, rad, rad / 2.0),
                        ),
                    ]),
                };

                commands.spawn((Transform::from_xyz(x, y, z), RigidBody::Dynamic, collider));
            }
        }
    }
}
