//! A ray-cast vehicle: use the up/down arrows to accelerate forward/backward, the left/right
//! arrows to steer, and space to brake.

use bevy::{prelude::*, transform::TransformSystems};
use bevy_rapier3d::prelude::*;

const HALF_EXTENTS: Vec3 = Vec3::new(2.0, 0.4, 1.0);
const WHEEL_RADIUS: f32 = 0.4;
const WHEEL_WIDTH: f32 = 0.3;
const ENGINE_FORCE: f32 = 300.0;
const BRAKE: f32 = 20.0;
const MAX_STEERING: f32 = 0.5;

/// Marks the entity rendering the wheel with the given index.
#[derive(Component)]
struct WheelVisual(usize);

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
        .add_systems(Update, drive_vehicle)
        .add_systems(
            PostUpdate,
            (update_wheel_visuals, follow_vehicle)
                .after(PhysicsSet::Writeback)
                .before(TransformSystems::Propagate),
        )
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-15.0, 8.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    /*
     * Ground
     */
    let ground_size = 100.0;
    let ground_height = 0.1;
    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));

    /*
     * A few boxes to push around.
     */
    for i in 0..5 {
        commands.spawn((
            Transform::from_xyz(15.0 + i as f32 * 1.2, 0.5, (i as f32 - 2.0) * 1.2),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 0.5),
        ));
    }

    /*
     * The vehicle, moving forward along its local +X axis.
     */
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let suspension_rest_length = 0.4;
    let wheel_positions = [
        Vec3::new(HALF_EXTENTS.x * 0.75, -HALF_EXTENTS.y, HALF_EXTENTS.z),
        Vec3::new(HALF_EXTENTS.x * 0.75, -HALF_EXTENTS.y, -HALF_EXTENTS.z),
        Vec3::new(-HALF_EXTENTS.x * 0.75, -HALF_EXTENTS.y, HALF_EXTENTS.z),
        Vec3::new(-HALF_EXTENTS.x * 0.75, -HALF_EXTENTS.y, -HALF_EXTENTS.z),
    ];
    let wheels = wheel_positions
        .iter()
        .map(|pos| {
            VehicleWheel::new(
                *pos,
                -Vec3::Y,
                Vec3::Z,
                suspension_rest_length,
                WHEEL_RADIUS,
                tuning,
            )
        })
        .collect();

    let wheel_mesh = meshes.add(Cylinder::new(WHEEL_RADIUS, WHEEL_WIDTH));
    let wheel_material = materials.add(Color::srgb(0.2, 0.2, 0.2));

    commands
        .spawn((
            Transform::from_xyz(0.0, 2.0, 0.0),
            RigidBody::Dynamic,
            Collider::cuboid(HALF_EXTENTS.x, HALF_EXTENTS.y, HALF_EXTENTS.z),
            ColliderMassProperties::Density(10.0),
            RayCastVehicleController::new(wheels),
        ))
        .with_children(|chassis| {
            for (i, pos) in wheel_positions.iter().enumerate() {
                chassis
                    .spawn((WheelVisual(i), Transform::from_translation(*pos)))
                    .with_children(|wheel| {
                        // Cylinders are aligned with Y; align them with the wheel axle (Z).
                        wheel.spawn((
                            Mesh3d(wheel_mesh.clone()),
                            MeshMaterial3d(wheel_material.clone()),
                            Transform::from_rotation(Quat::from_rotation_x(
                                std::f32::consts::FRAC_PI_2,
                            )),
                        ));
                    });
            }
        });
}

fn drive_vehicle(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut vehicles: Query<&mut RayCastVehicleController>,
) {
    let mut engine_force = 0.0;
    let mut steering = 0.0;
    let mut brake = 0.0;

    if keyboard.pressed(KeyCode::ArrowUp) {
        engine_force += ENGINE_FORCE;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        engine_force -= ENGINE_FORCE;
    }
    if keyboard.pressed(KeyCode::ArrowLeft) {
        steering += MAX_STEERING;
    }
    if keyboard.pressed(KeyCode::ArrowRight) {
        steering -= MAX_STEERING;
    }
    if keyboard.pressed(KeyCode::Space) {
        brake = BRAKE;
    }

    for mut vehicle in vehicles.iter_mut() {
        for (i, wheel) in vehicle.wheels.iter_mut().enumerate() {
            let is_front = i < 2;
            wheel.engine_force = if is_front { engine_force } else { 0.0 };
            wheel.steering = if is_front { steering } else { 0.0 };
            wheel.brake = brake;
        }
    }
}

fn update_wheel_visuals(
    vehicles: Query<&RayCastVehicleController>,
    mut wheels: Query<(&WheelVisual, &ChildOf, &mut Transform)>,
) {
    for (visual, child_of, mut transform) in wheels.iter_mut() {
        if let Some(wheel) = vehicles
            .get(child_of.parent())
            .ok()
            .and_then(|vehicle| vehicle.wheels.get(visual.0))
        {
            *transform = wheel.local_transform();
        }
    }
}

fn follow_vehicle(
    vehicles: Query<&Transform, With<RayCastVehicleController>>,
    mut cameras: Query<&mut Transform, (With<Camera3d>, Without<RayCastVehicleController>)>,
) {
    let (Ok(vehicle), Ok(mut camera)) = (vehicles.single(), cameras.single_mut()) else {
        return;
    };
    let back = vehicle.rotation * -Vec3::X;
    let back = Vec3::new(back.x, 0.0, back.z).normalize_or(-Vec3::X);
    let eye = vehicle.translation + back * 12.0 + Vec3::Y * 6.0;
    *camera = Transform::from_translation(eye).looking_at(vehicle.translation, Vec3::Y);
}
