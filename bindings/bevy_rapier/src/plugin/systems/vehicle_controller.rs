use crate::control::controller_filter::ControllerExclusions;
use crate::control::ControllerIgnored;
use crate::control::{RayCastVehicleController, VehicleWheel, VehicleWheelState};
use crate::dynamics::RapierRigidBodyHandle;
use crate::plugin::context::{
    RapierContextColliders, RapierContextEntityLink, RapierContextSimulation, RapierRigidBodySet,
};
use crate::plugin::RapierConfiguration;
use bevy::ecs::entity::EntityHashSet;
use bevy::prelude::*;
use rapier::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier::geometry::{Collider as RapierCollider, ColliderHandle, ColliderSet};
use rapier::pipeline::QueryFilter;

/// Rebuilds the Rapier vehicle if its wheel count no longer matches the component's wheels.
fn sync_raw_vehicle(
    vehicle: &mut RayCastVehicleController,
    chassis: rapier::dynamics::RigidBodyHandle,
) {
    let wheels_match = vehicle
        .raw
        .as_ref()
        .is_some_and(|raw| raw.wheels().len() == vehicle.wheels.len());

    if !wheels_match {
        let mut raw = DynamicRayCastVehicleController::new(chassis);
        for wheel in &vehicle.wheels {
            let raw_wheel = raw.add_wheel(
                wheel.chassis_connection_point,
                wheel.direction,
                wheel.axle,
                wheel.suspension_rest_length,
                wheel.radius,
                &WheelTuning::default(),
            );
            raw_wheel.rotation = wheel.state.rotation;
        }
        vehicle.raw = Some(raw);
    }

    let RayCastVehicleController {
        wheels,
        index_up_axis,
        index_forward_axis,
        raw,
        ..
    } = vehicle;
    let raw = raw.as_mut().expect("the raw vehicle was just created");
    raw.chassis = chassis;
    raw.index_up_axis = *index_up_axis;
    raw.index_forward_axis = *index_forward_axis;

    for (wheel, raw_wheel) in wheels.iter().zip(raw.wheels_mut()) {
        wheel.sync_to_raw(raw_wheel);
    }
}

fn read_wheel_state(
    wheel: &mut VehicleWheel,
    raw: &rapier::control::Wheel,
    colliders: &ColliderSet,
) {
    let info = raw.raycast_info();
    wheel.state = VehicleWheelState {
        rotation: raw.rotation,
        suspension_length: info.suspension_length,
        is_in_contact: info.is_in_contact,
        contact_point: info.contact_point_ws,
        contact_normal: info.contact_normal_ws,
        hard_point: info.hard_point_ws,
        center: raw.center(),
        suspension_direction: raw.suspension(),
        axle_direction: raw.axle(),
        ground_entity: info
            .ground_object
            .and_then(|h| RapierContextColliders::collider_entity_with_set(colliders, h)),
        suspension_force: raw.wheel_suspension_force,
        forward_impulse: raw.forward_impulse,
        side_impulse: raw.side_impulse,
    };
}

/// System responsible for updating the [`RayCastVehicleController`]s before the simulation
/// step, and writing back their wheels' state.
pub fn update_vehicle_controllers(
    mut context_access: Query<(
        &RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierRigidBodySet,
        &RapierConfiguration,
    )>,
    mut vehicles: Query<(
        &RapierContextEntityLink,
        &RapierRigidBodyHandle,
        &mut RayCastVehicleController,
    )>,
    ignored: Query<Entity, With<ControllerIgnored>>,
) {
    let ignored: EntityHashSet = ignored.iter().collect();
    for (link, handle, mut vehicle) in vehicles.iter_mut() {
        let Ok((context, mut context_colliders, mut rigidbody_set, config)) =
            context_access.get_mut(link.0)
        else {
            continue;
        };
        if !config.physics_pipeline_active {
            continue;
        }
        let context_colliders = &mut *context_colliders;
        let rigidbody_set = &mut *rigidbody_set;
        if !rigidbody_set
            .bodies
            .get(handle.0)
            .is_some_and(|body| body.is_enabled())
        {
            continue;
        }

        let vehicle = &mut *vehicle;
        sync_raw_vehicle(vehicle, handle.0);

        let RayCastVehicleController {
            wheels,
            filter_flags,
            filter_groups,
            exclude_colliders,
            exclude_rigid_bodies,
            filter_predicate,
            current_vehicle_speed,
            raw,
            ..
        } = vehicle;
        let raw = raw.as_mut().expect("the raw vehicle was just synchronized");

        let exclusions = ControllerExclusions::new(
            exclude_colliders,
            exclude_rigid_bodies,
            &ignored,
            filter_predicate.as_ref(),
            rigidbody_set,
        );
        let predicate = |_: ColliderHandle, collider: &RapierCollider| exclusions.test(collider);
        let filter = QueryFilter {
            flags: *filter_flags,
            groups: filter_groups.map(|g| g.into()),
            exclude_collider: None,
            // The wheels' rays start inside the chassis, so it must never be hit.
            exclude_rigid_body: Some(handle.0),
            predicate: (!exclusions.is_empty())
                .then_some(&predicate as &dyn Fn(ColliderHandle, &RapierCollider) -> bool),
        };
        let query_pipeline = context.broad_phase.as_query_pipeline_mut(
            context.narrow_phase.query_dispatcher(),
            &mut rigidbody_set.bodies,
            &mut context_colliders.colliders,
            filter,
        );
        raw.update_vehicle(context.integration_parameters.dt, query_pipeline);

        *current_vehicle_speed = raw.current_vehicle_speed;
        for (wheel, raw_wheel) in wheels.iter_mut().zip(raw.wheels()) {
            read_wheel_state(wheel, raw_wheel, &context_colliders.colliders);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Collider;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use crate::prelude::{RigidBody, Velocity};
    use bevy::time::{TimePlugin, TimeUpdateStrategy};

    fn vehicle_wheels(hw: f32, hh: f32) -> Vec<VehicleWheel> {
        let tuning = WheelTuning {
            suspension_stiffness: 100.0,
            suspension_damping: 10.0,
            ..WheelTuning::default()
        };
        [
            Vec3::new(hw * 1.5, -hh, hw),
            Vec3::new(hw * 1.5, -hh, -hw),
            Vec3::new(-hw * 1.5, -hh, hw),
            Vec3::new(-hw * 1.5, -hh, -hw),
        ]
        .into_iter()
        .map(|pos| VehicleWheel::new(pos, -Vec3::Y, Vec3::Z, hh, hh / 4.0, tuning))
        .collect()
    }

    #[test]
    fn vehicle_rests_on_wheels_and_moves_forward() {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1.0 / 60.0),
        ));
        app.finish();

        app.world_mut().spawn((
            Transform::from_xyz(0.0, -0.1, 0.0),
            Collider::cuboid(50.0, 0.1, 50.0),
        ));

        let (hw, hh) = (0.3, 0.15);
        let chassis = app
            .world_mut()
            .spawn((
                Transform::from_xyz(0.0, 0.5, 0.0),
                RigidBody::Dynamic,
                Collider::cuboid(hw * 2.0, hh, hw),
                crate::geometry::ColliderMassProperties::Density(100.0),
                Velocity::zero(),
                RayCastVehicleController::new(vehicle_wheels(hw, hh)),
            ))
            .id();

        // Let the vehicle settle on its suspension.
        for _ in 0..120 {
            app.update();
        }

        let vehicle = app
            .world()
            .get::<RayCastVehicleController>(chassis)
            .unwrap();
        assert!(
            vehicle.wheels.iter().all(|w| w.state.is_in_contact),
            "All the wheels should touch the ground: {:?}",
            vehicle.wheels
        );
        assert!(vehicle
            .wheels
            .iter()
            .all(|w| w.state.ground_entity.is_some()));
        let rest_height = app.world().get::<Transform>(chassis).unwrap().translation.y;
        assert!(
            rest_height > hh + 0.05,
            "The chassis should be held above the ground by the suspension"
        );
        let start_x = app.world().get::<Transform>(chassis).unwrap().translation.x;

        // Accelerate with the front wheels.
        {
            let mut vehicle = app
                .world_mut()
                .get_mut::<RayCastVehicleController>(chassis)
                .unwrap();
            vehicle.wheels[0].engine_force = 30.0;
            vehicle.wheels[1].engine_force = 30.0;
        }
        for _ in 0..60 {
            app.update();
        }

        let end_x = app.world().get::<Transform>(chassis).unwrap().translation.x;
        let vehicle = app
            .world()
            .get::<RayCastVehicleController>(chassis)
            .unwrap();
        assert!(
            end_x > start_x + 0.1,
            "The vehicle should move forward: {start_x} -> {end_x}"
        );
        assert!(vehicle.current_vehicle_speed > 0.0);
        assert!(vehicle.wheels[0].state.rotation > 0.0);
        assert!(vehicle.any_wheel_in_contact());
    }
}
