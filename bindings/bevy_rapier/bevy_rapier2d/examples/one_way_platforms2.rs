//! One-way platforms and conveyor belts implemented with `BevyPhysicsHooks` and
//! `ContactModificationContextView::update_as_oneway_platform`.

use bevy::{ecs::system::SystemParam, prelude::*};
use bevy_rapier2d::prelude::*;

const PIXELS_PER_METER: f32 = 10.0;
const MAX_CUBES: usize = 7;

/// A platform only solid for contacts whose normal (in the platform's local-space) is close to
/// `allowed_normal`, and moving what lies on it at `surface_speed`.
#[derive(Component, Copy, Clone)]
struct OneWayPlatform {
    allowed_normal: Vec2,
    surface_speed: f32,
}

#[derive(Component)]
struct Cube;

#[derive(SystemParam)]
struct OneWayPlatformHook<'w, 's> {
    platforms: Query<'w, 's, &'static OneWayPlatform>,
}

impl BevyPhysicsHooks for OneWayPlatformHook<'_, '_> {
    fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
        // The manifold normal points outward from the first collider, so the allowed direction
        // and the surface velocity are flipped if the platform is the second collider.
        let (platform, sign) = if let Ok(platform) = self.platforms.get(context.collider1()) {
            (platform, 1.0)
        } else if let Ok(platform) = self.platforms.get(context.collider2()) {
            (platform, -1.0)
        } else {
            return;
        };

        context.update_as_oneway_platform(platform.allowed_normal * sign, 0.1);

        if let Some(contacts) = context.solver_contacts_mut() {
            for contact in contacts.iter_mut() {
                contact.tangent_velocity.x = platform.surface_speed * sign;
            }
        }
    }
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<OneWayPlatformHook>::pixels_per_meter(PIXELS_PER_METER),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (spawn_cubes, flip_gravity))
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn(Camera2d);
}

fn setup_physics(mut commands: Commands) {
    // The top platform can be landed on from above, the bottom one from below.
    commands.spawn((
        Transform::from_xyz(300.0, 20.0, 0.0),
        Collider::cuboid(250.0, 5.0),
        ActiveHooks::MODIFY_SOLVER_CONTACTS,
        OneWayPlatform {
            allowed_normal: Vec2::Y,
            surface_speed: -12.0 * PIXELS_PER_METER,
        },
    ));
    commands.spawn((
        Transform::from_xyz(-300.0, -20.0, 0.0),
        Collider::cuboid(250.0, 5.0),
        ActiveHooks::MODIFY_SOLVER_CONTACTS,
        OneWayPlatform {
            allowed_normal: -Vec2::Y,
            surface_speed: 12.0 * PIXELS_PER_METER,
        },
    ));
}

fn spawn_cubes(
    mut commands: Commands,
    time: Res<Time>,
    mut timer: Local<Option<Timer>>,
    cubes: Query<(), With<Cube>>,
) {
    let timer = timer.get_or_insert_with(|| Timer::from_seconds(3.0, TimerMode::Repeating));
    timer.tick(time.delta());

    if timer.just_finished() && cubes.iter().count() < MAX_CUBES {
        commands.spawn((
            Transform::from_xyz(200.0, 100.0, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(15.0, 20.0),
            GravityScale(1.0),
            Cube,
        ));
    }
}

/// Bodies above the platforms fall down, and bodies below them fall up.
fn flip_gravity(mut cubes: Query<(&Transform, &mut GravityScale), With<Cube>>) {
    for (transform, mut gravity_scale) in &mut cubes {
        let y = transform.translation.y;
        if y > 10.0 && gravity_scale.0 != 1.0 {
            gravity_scale.0 = 1.0;
        } else if y < -10.0 && gravity_scale.0 != -1.0 {
            gravity_scale.0 = -1.0;
        }
    }
}
