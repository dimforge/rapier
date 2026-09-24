use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rapier3d::math::Pose;

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
        Transform::from_xyz(-100.0, 100.0, -100.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Create a voxelized wavy floor.
     */
    let voxel_size = Vec3::new(1f32, 1f32, 1f32);
    let mut samples = vec![];
    let n = 200;
    for i in 0..n {
        for j in 0..n {
            let y = (i as f32 / n as f32 * 10.0).sin().clamp(-0.8, 0.8)
                * (j as f32 / n as f32 * 10.0).cos().clamp(-0.8, 0.8)
                * 16.0;

            samples.push(Vec3::new(
                i as f32 * voxel_size.x,
                y * voxel_size.y,
                j as f32 * voxel_size.z,
            ));

            if i == 0 || i == n - 1 || j == 0 || j == n - 1 {
                // Create walls so the object at the edge don’t fall into the infinite void.
                for k in 0..4 {
                    samples.push(Vec3::new(
                        i as f32 * voxel_size.x,
                        (y + k as f32) * voxel_size.y,
                        j as f32 * voxel_size.z,
                    ));
                }
            }
        }
    }
    let collider = Collider::voxels_from_points(voxel_size, &samples);
    let ground_position = Vec3::new(voxel_size.x / 2f32, 0.0, voxel_size.z / 2f32);
    let floor_aabb = collider.raw.compute_aabb(&Pose::IDENTITY);
    commands.spawn((Transform::from_translation(ground_position), collider));

    /*
     * Create dynamic objects to fall on voxels.
     */
    let extents = floor_aabb.extents() * 0.75;
    let margin = (floor_aabb.extents() - extents) / 2.0;
    let nik = 30;
    for i in 0..nik {
        for j in 0..5 {
            for k in 0..nik {
                let falling_objects = (j + i + k) % 3;

                let ball_radius = 0.5;
                let co = match falling_objects {
                    0 => Collider::ball(ball_radius),
                    1 => Collider::cuboid(ball_radius, ball_radius, ball_radius),
                    2 => Collider::capsule_y(ball_radius, ball_radius),
                    _ => unreachable!(),
                };
                commands.spawn((
                    Transform::from_xyz(
                        floor_aabb.mins.x + margin.x + i as f32 * extents.x / nik as f32
                            - ground_position.x,
                        floor_aabb.maxs.y + j as f32 * 2.0 - ground_position.y,
                        floor_aabb.mins.z + margin.z + k as f32 * extents.z / nik as f32
                            - -ground_position.z,
                    ),
                    RigidBody::Dynamic,
                    co,
                ));
            }
        }
    }
}
