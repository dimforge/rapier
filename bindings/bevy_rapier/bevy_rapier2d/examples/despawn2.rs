use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

#[derive(Component, Default)]
pub struct Despawn;
#[derive(Component, Default)]
pub struct Resize;

#[derive(Resource, Default)]
pub struct DespawnResource {
    timer: Timer,
}

#[derive(Resource, Default)]
pub struct ResizeResource {
    timer: Timer,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(DespawnResource::default())
        .insert_resource(ResizeResource::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (despawn, resize))
        .run();
}

pub fn setup_graphics(
    mut commands: Commands,
    mut despawn: ResMut<DespawnResource>,
    mut resize: ResMut<ResizeResource>,
) {
    resize.timer = Timer::from_seconds(6.0, TimerMode::Once);
    despawn.timer = Timer::from_seconds(5.0, TimerMode::Once);

    commands.spawn((Camera2d, Transform::from_xyz(0.0, 20.0, 0.0)));
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    let ground_size = 250.0;

    commands.spawn((Collider::cuboid(ground_size, 12.0), Despawn));

    commands.spawn((
        Transform::from_xyz(ground_size, ground_size * 2.0, 0.0),
        Collider::cuboid(12.0, ground_size * 2.0),
    ));

    commands.spawn((
        Transform::from_xyz(-ground_size, ground_size * 2.0, 0.0),
        Collider::cuboid(12.0, ground_size * 2.0),
    ));

    /*
     * Create the cubes
     */
    let num = 20;
    let rad = 5.0;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = i as f32 * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;

            let mut entity = commands.spawn((
                Transform::from_xyz(x, y, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(rad, rad),
            ));

            if (i + j * num).is_multiple_of(100) {
                entity.insert(Resize);
            }
        }
    }
}

pub fn despawn(
    mut commands: Commands,
    time: Res<Time>,
    mut despawn: ResMut<DespawnResource>,
    query: Query<Entity, With<Despawn>>,
) {
    if despawn.timer.tick(time.delta()).just_finished() {
        for e in &query {
            commands.entity(e).despawn();
        }
    }
}

pub fn resize(
    mut commands: Commands,
    time: Res<Time>,
    mut resize: ResMut<ResizeResource>,
    query: Query<Entity, With<Resize>>,
) {
    if resize.timer.tick(time.delta()).just_finished() {
        for e in &query {
            commands.entity(e).insert(Collider::cuboid(20.0, 20.0));
        }
    }
}
