//! Conversions from Rapier objects built by the robot loaders into `bevy_rapier` components.

use crate::dynamics::{
    AdditionalMassProperties, AdditionalPgsIterations, AdditionalSolverIterations,
    AllowFastRotation, Ccd, Damping, Dominance, GravityScale, GyroscopicForces, LockedAxes,
    MassProperties, RigidBody, RigidBodyDisabled, Sleeping, SoftCcd, Velocity,
};
use crate::geometry::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, Collider, ColliderDisabled,
    ColliderMassProperties, CollisionGroups, ContactForceEventThreshold, ContactSkin, Friction,
    Restitution, Sensor, SolverGroups,
};
use crate::plugin::context::RapierContextEntityLink;
use crate::utils::iso_to_transform;
use bevy::ecs::system::EntityCommands;
use bevy::prelude::*;
use rapier::dynamics::{JointAxis, RigidBody as RapierRigidBody, RigidBodyAdditionalMassProps};
use rapier::geometry::Collider as RapierCollider;

/// The error returned when a robot description fails to load.
pub type LoaderError = Box<dyn std::error::Error + Send + Sync>;

/// Converts an axis index (`0..6`) into a [`JointAxis`].
pub(crate) fn joint_axis(axis: usize) -> JointAxis {
    match axis {
        0 => JointAxis::LinX,
        1 => JointAxis::LinY,
        2 => JointAxis::LinZ,
        3 => JointAxis::AngX,
        4 => JointAxis::AngY,
        _ => JointAxis::AngZ,
    }
}

/// Inserts the components describing the given Rapier rigid-body (except its pose).
pub(crate) fn insert_rigid_body(entity: &mut EntityCommands, rb: &RapierRigidBody) {
    let activation = rb.activation();
    entity.insert((
        RigidBody::from(rb.body_type()),
        Velocity {
            linear: rb.linvel(),
            angular: rb.angvel(),
        },
        Damping {
            linear_damping: rb.linear_damping(),
            angular_damping: rb.angular_damping(),
        },
        GravityScale(rb.gravity_scale()),
        Ccd {
            enabled: rb.is_ccd_enabled(),
        },
        SoftCcd {
            prediction: rb.soft_ccd_prediction(),
        },
        Dominance::group(rb.dominance_group()),
        Sleeping {
            normalized_linear_threshold: activation.normalized_linear_threshold,
            angular_threshold: activation.angular_threshold,
            time_until_sleep: activation.time_until_sleep,
            sleeping: activation.sleeping,
        },
        LockedAxes::from_bits_retain(rb.locked_axes().bits()),
        AdditionalSolverIterations(rb.additional_solver_iterations()),
        AdditionalPgsIterations(rb.additional_pgs_iterations()),
        GyroscopicForces {
            enabled: rb.gyroscopic_forces_enabled(),
        },
    ));

    if let Some(mprops) = rb.mass_properties().additional_local_mprops.as_deref() {
        entity.insert(match mprops {
            RigidBodyAdditionalMassProps::MassProps(mprops) => {
                AdditionalMassProperties::MassProperties(MassProperties::from_rapier(*mprops))
            }
            RigidBodyAdditionalMassProps::Mass(mass) => AdditionalMassProperties::Mass(*mass),
        });
    }
    if !rb.is_enabled() {
        entity.insert(RigidBodyDisabled);
    }
    if rb.is_fast_rotation_allowed() {
        entity.insert(AllowFastRotation);
    }
}

/// Inserts the components describing the given Rapier collider, including its local pose
/// relative to its rigid-body.
///
/// The [`ActiveHooks`] of the collider are replaced by `hooks`.
pub(crate) fn insert_collider(
    entity: &mut EntityCommands,
    co: &RapierCollider,
    hooks: ActiveHooks,
) {
    // A collider with a zero density doesn't contribute to its body's mass. Other colliders keep
    // their exact mass properties, whatever the way they were specified.
    let mprops = if co.mass() == 0.0 {
        ColliderMassProperties::Density(0.0)
    } else {
        ColliderMassProperties::MassProperties(MassProperties::from_rapier(co.mass_properties()))
    };

    entity.insert((
        Collider::from(co.shared_shape().clone()),
        iso_to_transform(co.position()),
        mprops,
        Friction {
            coefficient: co.friction(),
            combine_rule: co.friction_combine_rule().into(),
        },
        Restitution {
            coefficient: co.restitution(),
            combine_rule: co.restitution_combine_rule().into(),
        },
        CollisionGroups::from(co.collision_groups()),
        SolverGroups::from(co.solver_groups()),
        hooks,
        ActiveEvents::from_bits_retain(co.active_events().bits()),
        ActiveCollisionTypes::from_bits_retain(co.active_collision_types().bits()),
        ContactSkin(co.contact_skin()),
        ContactForceEventThreshold(co.contact_force_event_threshold()),
    ));
    if co.is_sensor() {
        entity.insert(Sensor);
    }
    if !co.is_enabled() {
        entity.insert(ColliderDisabled);
    }
}

/// The [`ActiveHooks`] set on the given Rapier collider.
pub(crate) fn active_hooks(co: &RapierCollider) -> ActiveHooks {
    ActiveHooks::from_bits_retain(co.active_hooks().bits())
}

/// Inserts the [`RapierContextEntityLink`] of `context`, if any.
pub(crate) fn insert_context_link(entity: &mut EntityCommands, context: Option<Entity>) {
    if let Some(context) = context {
        entity.insert(RapierContextEntityLink(context));
    }
}
