use crate::control::{PdController, PidController};
use crate::dynamics::RapierRigidBodyHandle;
use crate::plugin::context::{
    RapierContextEntityLink, RapierContextSimulation, RapierRigidBodySet,
};
use crate::plugin::RapierConfiguration;
use bevy::prelude::*;
use rapier::dynamics::{RigidBody as RapierRigidBody, RigidBodyVelocity};

fn apply_correction(
    body: &mut RapierRigidBody,
    linvel: crate::math::Vect,
    angvel: crate::math::AngVector,
) {
    let correction = RigidBodyVelocity { linvel, angvel };
    if correction != RigidBodyVelocity::zero() {
        let vels = *body.vels() + correction;
        body.set_vels(vels, true);
    }
}

/// System responsible for driving rigid-bodies with a [`PdController`] or a [`PidController`]
/// toward their target, by adding the controllers' corrections to their velocities.
pub fn update_pid_controllers(
    config: Query<&RapierConfiguration>,
    mut context_access: Query<(&RapierContextSimulation, &mut RapierRigidBodySet)>,
    pd_controllers: Query<(
        &RapierContextEntityLink,
        &RapierRigidBodyHandle,
        &PdController,
    )>,
    mut pid_controllers: Query<(
        &RapierContextEntityLink,
        &RapierRigidBodyHandle,
        &mut PidController,
    )>,
) {
    let is_active = |link: &RapierContextEntityLink| {
        config
            .get(link.0)
            .is_ok_and(|config| config.physics_pipeline_active)
    };

    for (link, handle, controller) in pd_controllers.iter() {
        if !is_active(link) {
            continue;
        }
        let Ok((_, mut rigidbody_set)) = context_access.get_mut(link.0) else {
            continue;
        };
        let Some(body) = rigidbody_set.bodies.get_mut(handle.0) else {
            continue;
        };
        if body.is_fixed() || !body.is_enabled() {
            continue;
        }
        let correction = controller.rigid_body_correction(body);
        apply_correction(body, correction.linear, correction.angular);
    }

    for (link, handle, mut controller) in pid_controllers.iter_mut() {
        if !is_active(link) {
            continue;
        }
        let Ok((context, mut rigidbody_set)) = context_access.get_mut(link.0) else {
            continue;
        };
        let dt = context.integration_parameters.dt;
        let Some(body) = rigidbody_set.bodies.get_mut(handle.0) else {
            continue;
        };
        if body.is_fixed() || !body.is_enabled() {
            continue;
        }
        let correction = controller.rigid_body_correction(dt, body);
        apply_correction(body, correction.linear, correction.angular);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::{AxesMask, PidTarget};
    use crate::geometry::Collider;
    use crate::math::Vect;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use crate::prelude::RigidBody;
    use bevy::time::{TimePlugin, TimeUpdateStrategy};

    #[test]
    fn pid_controller_drives_body_toward_target() {
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

        let target = Vect::splat(3.0);
        let controlled = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                Collider::ball(0.5),
                PidController::new(10.0, 1.0, 0.5, AxesMask::LIN_AXES)
                    .with_target(PidTarget::from_translation(target)),
            ))
            .id();
        let pd_controlled = app
            .world_mut()
            .spawn((
                Transform::from_xyz(10.0, 0.0, 0.0),
                RigidBody::Dynamic,
                Collider::ball(0.5),
                PdController::new(10.0, 0.5, AxesMask::LIN_AXES)
                    .with_target(PidTarget::from_translation(Vect::X * 10.0 + target)),
            ))
            .id();

        for _ in 0..300 {
            app.update();
        }

        let translation = |entity: Entity| {
            let t = app.world().get::<Transform>(entity).unwrap().translation;
            #[cfg(feature = "dim2")]
            return t.truncate();
            #[cfg(feature = "dim3")]
            return t;
        };
        let pid_error = (translation(controlled) - target).length();
        let pd_error = (translation(pd_controlled) - Vect::X * 10.0 - target).length();
        // Both bodies start about 5.2 units away from their target and fight gravity.
        assert!(
            pid_error < 0.05,
            "PID controller error too large: {pid_error}"
        );
        assert!(pd_error < 0.05, "PD controller error too large: {pd_error}");
        assert_ne!(
            app.world()
                .get::<PidController>(controlled)
                .unwrap()
                .lin_integral,
            Vect::ZERO,
            "The integral error should be written back"
        );
    }
}
