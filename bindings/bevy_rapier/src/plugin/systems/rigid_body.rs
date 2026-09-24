use crate::dynamics::RapierRigidBodyHandle;
use crate::plugin::context::systemparams::RAPIER_CONTEXT_EXPECT_ERROR;
use crate::plugin::context::{
    RapierContextColliders, RapierContextEntityLink, RapierContextSimulation, RapierRigidBodySet,
};
use crate::plugin::{configuration::TimestepMode, RapierConfiguration};
use crate::{dynamics::RigidBody, plugin::context::SimulationToRenderTime};
use crate::{prelude::*, utils};
use bevy::prelude::*;

use super::RapierContextLinkResolver;
use rapier::dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodyType};
use std::collections::HashMap;

/// Components that will be updated after a physics step.
pub type RigidBodyWritebackComponents<'a> = (
    &'a RapierRigidBodyHandle,
    &'a RapierContextEntityLink,
    Option<&'a ChildOf>,
    Option<&'a mut Transform>,
    Option<&'a mut TransformInterpolation>,
    Option<&'a mut Velocity>,
    Option<&'a mut Sleeping>,
    Option<&'a mut ReadWorldMassProperties>,
);

/// Components related to rigid-bodies.
pub type RigidBodyComponents<'a> = (
    (Entity, Option<&'a RapierContextEntityLink>),
    &'a RigidBody,
    Option<&'a GlobalTransform>,
    Option<&'a Velocity>,
    Option<&'a AdditionalMassProperties>,
    Option<&'a ReadMassProperties>,
    Option<&'a LockedAxes>,
    Option<&'a ExternalForce>,
    Option<&'a GravityScale>,
    (Option<&'a Ccd>, Option<&'a SoftCcd>),
    Option<&'a Dominance>,
    Option<&'a Sleeping>,
    Option<&'a Damping>,
    Option<&'a RigidBodyDisabled>,
    (
        Option<&'a AdditionalSolverIterations>,
        Option<&'a AdditionalPgsIterations>,
        Option<&'a AllowFastRotation>,
    ),
);

/// System responsible for applying changes the user made to a rigid-body-related component.
pub fn apply_rigid_body_user_changes(
    (mut rigid_body_sets, mut context_simulations): (
        Query<&mut RapierRigidBodySet>,
        Query<&mut RapierContextSimulation>,
    ),
    config: Query<&RapierConfiguration>,
    changed_rb_types: Query<
        (&RapierRigidBodyHandle, &RapierContextEntityLink, &RigidBody),
        Changed<RigidBody>,
    >,
    mut changed_transforms: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &GlobalTransform,
            Option<&mut TransformInterpolation>,
        ),
        Changed<GlobalTransform>,
    >,
    changed_velocities: Query<
        (&RapierRigidBodyHandle, &RapierContextEntityLink, &Velocity),
        Changed<Velocity>,
    >,
    changed_additional_mass_props: Query<
        (
            Entity,
            &RapierContextEntityLink,
            &RapierRigidBodyHandle,
            &AdditionalMassProperties,
        ),
        Changed<AdditionalMassProperties>,
    >,
    changed_locked_axes: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &LockedAxes,
        ),
        Changed<LockedAxes>,
    >,
    changed_forces: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &ExternalForce,
        ),
        Changed<ExternalForce>,
    >,
    mut changed_impulses: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &mut ExternalImpulse,
        ),
        Changed<ExternalImpulse>,
    >,
    changed_gravity_scale: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &GravityScale,
        ),
        Changed<GravityScale>,
    >,
    (changed_ccd, changed_soft_ccd): (
        Query<(&RapierRigidBodyHandle, &RapierContextEntityLink, &Ccd), Changed<Ccd>>,
        Query<(&RapierRigidBodyHandle, &RapierContextEntityLink, &SoftCcd), Changed<SoftCcd>>,
    ),
    (changed_dominance, changed_damping): (
        Query<(&RapierRigidBodyHandle, &RapierContextEntityLink, &Dominance), Changed<Dominance>>,
        Query<(&RapierRigidBodyHandle, &RapierContextEntityLink, &Damping), Changed<Damping>>,
    ),
    changed_sleeping: Query<
        (&RapierRigidBodyHandle, &RapierContextEntityLink, &Sleeping),
        Changed<Sleeping>,
    >,
    (
        changed_disabled,
        changed_additional_solver_iterations,
        changed_additional_pgs_iterations,
        changed_fast_rotation,
    ): (
        Query<
            (
                &RapierRigidBodyHandle,
                &RapierContextEntityLink,
                &RigidBodyDisabled,
            ),
            Changed<RigidBodyDisabled>,
        >,
        Query<
            (
                &RapierRigidBodyHandle,
                &RapierContextEntityLink,
                &AdditionalSolverIterations,
            ),
            Changed<AdditionalSolverIterations>,
        >,
        Query<
            (
                &RapierRigidBodyHandle,
                &RapierContextEntityLink,
                &AdditionalPgsIterations,
            ),
            Changed<AdditionalPgsIterations>,
        >,
        Query<(&RapierRigidBodyHandle, &RapierContextEntityLink), Changed<AllowFastRotation>>,
    ),
    #[cfg(feature = "dim3")] changed_gyroscopic_forces: Query<
        (
            &RapierRigidBodyHandle,
            &RapierContextEntityLink,
            &GyroscopicForces,
        ),
        Changed<GyroscopicForces>,
    >,
    mut mass_modified: MessageWriter<MassModifiedEvent>,
) {
    // Deal with sleeping first, because other changes may then wake-up the
    // rigid-body again.
    for (handle, link, sleeping) in changed_sleeping.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();

        let mut wake_up_island = false;
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            let activation = rb.activation_mut();
            activation.normalized_linear_threshold = sleeping.normalized_linear_threshold;
            activation.angular_threshold = sleeping.angular_threshold;
            activation.time_until_sleep = sleeping.time_until_sleep;

            if !sleeping.sleeping && activation.sleeping {
                rb.wake_up(true);
                wake_up_island = true;
            } else if sleeping.sleeping && !activation.sleeping {
                rb.sleep();
            }
        }

        if wake_up_island {
            // Wake up the whole island, not only this body.
            let mut context = context_simulations
                .get_mut(link.0)
                .expect(RAPIER_CONTEXT_EXPECT_ERROR);
            context
                .islands
                .wake_up(&mut rigidbody_set.bodies, handle.0, true);
        }
    }

    // NOTE: we must change the rigid-body type before updating the
    //       transform or velocity. Otherwise, if the rigid-body was fixed
    //       and changed to anything else, the velocity change wouldn’t have any effect.
    //       Similarly, if the rigid-body was kinematic position-based before and
    //       changed to anything else, a transform change would modify the next
    //       position instead of the current one.
    for (handle, link, rb_type) in changed_rb_types.iter() {
        let context = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = context.bodies.get_mut(handle.0) {
            // The type of a soft-body cluster proxy is managed by Rapier.
            if !rb.is_soft_frame() {
                rb.set_body_type((*rb_type).into(), true);
            }
        }
    }

    // Manually checks if the transform changed.
    // This is needed for detecting if the user actually changed the rigid-body
    // transform, or if it was just the change we made in our `writeback_rigid_bodies`
    // system.
    let transform_changed_fn =
        |handle: &RigidBodyHandle,
         config: &RapierConfiguration,
         transform: &GlobalTransform,
         last_transform_set: &HashMap<RigidBodyHandle, GlobalTransform>| {
            if config.force_update_from_transform_changes {
                true
            } else if let Some(prev) = last_transform_set.get(handle) {
                *prev != *transform
            } else {
                true
            }
        };

    for (handle, link, global_transform, mut interpolation) in changed_transforms.iter_mut() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        let config = config
            .get(link.0)
            .expect("Could not get `RapierConfiguration`");
        // Use an Option<bool> to avoid running the check twice.
        let mut transform_changed = None;

        if let Some(interpolation) = interpolation.as_deref_mut() {
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle.0,
                    config,
                    global_transform,
                    &rigidbody_set.last_body_transform_set,
                ))
            });

            if transform_changed == Some(true) {
                // Reset the interpolation so we don’t overwrite
                // the user’s input.
                interpolation.start = None;
                interpolation.end = None;
            }
        }
        // TODO: avoid to run multiple times the mutable deref ?
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            // The pose of a soft-body cluster proxy follows its particles: it is written back
            // to the transform, but never read from it.
            if rb.is_soft_frame() {
                continue;
            }
            transform_changed = transform_changed.or_else(|| {
                Some(transform_changed_fn(
                    &handle.0,
                    config,
                    global_transform,
                    &rigidbody_set.last_body_transform_set,
                ))
            });

            match rb.body_type() {
                RigidBodyType::KinematicPositionBased => {
                    if transform_changed == Some(true) {
                        rb.set_next_kinematic_position(utils::transform_to_iso(
                            &global_transform.compute_transform(),
                        ));
                        rigidbody_set
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
                    }
                }
                _ => {
                    if transform_changed == Some(true) {
                        rb.set_position(
                            utils::transform_to_iso(&global_transform.compute_transform()),
                            true,
                        );
                        rigidbody_set
                            .last_body_transform_set
                            .insert(handle.0, *global_transform);
                    }
                }
            }
        }
    }

    for (handle, link, velocity) in changed_velocities.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            // The velocity of a soft-body cluster proxy follows its particles.
            if rb.is_soft_frame() {
                continue;
            }
            rb.set_linvel(velocity.linear, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.set_angvel(velocity.angular.into(), true);
        }
    }

    for (entity, link, handle, mprops) in changed_additional_mass_props.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    rb.set_additional_mass_properties(mprops.into_rapier(), true);
                }
                AdditionalMassProperties::Mass(mass) => {
                    rb.set_additional_mass(*mass, true);
                }
            }

            mass_modified.write(entity.into());
        }
    }

    for (handle, link, additional_solver_iters) in changed_additional_solver_iterations.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_additional_solver_iterations(additional_solver_iters.0);
        }
    }

    for (handle, link, additional_pgs_iters) in changed_additional_pgs_iterations.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_additional_pgs_iterations(additional_pgs_iters.0);
        }
    }

    for (handle, link) in changed_fast_rotation.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_allow_fast_rotation(true);
        }
    }

    #[cfg(feature = "dim3")]
    for (handle, link, gyroscopic_forces) in changed_gyroscopic_forces.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.enable_gyroscopic_forces(gyroscopic_forces.enabled);
        }
    }

    for (handle, link, locked_axes) in changed_locked_axes.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_locked_axes((*locked_axes).into(), true);
        }
    }

    for (handle, link, forces) in changed_forces.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.reset_forces(true);
            rb.reset_torques(true);
            rb.add_force(forces.force, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.add_torque(forces.torque.into(), true);
        }
    }

    for (handle, link, mut impulses) in changed_impulses.iter_mut() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.apply_impulse(impulses.impulse, true);
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulses.torque_impulse.into(), true);
            impulses.reset();
        }
    }

    for (handle, link, gravity_scale) in changed_gravity_scale.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_gravity_scale(gravity_scale.0, true);
        }
    }

    for (handle, link, ccd) in changed_ccd.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.enable_ccd(ccd.enabled);
        }
    }

    for (handle, link, soft_ccd) in changed_soft_ccd.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_soft_ccd_prediction(soft_ccd.prediction);
        }
    }

    for (handle, link, dominance) in changed_dominance.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_dominance_group(dominance.groups);
        }
    }

    for (handle, link, damping) in changed_damping.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(rb) = rigidbody_set.bodies.get_mut(handle.0) {
            rb.set_linear_damping(damping.linear_damping);
            rb.set_angular_damping(damping.angular_damping);
        }
    }

    for (handle, link, _) in changed_disabled.iter() {
        let rigidbody_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        if let Some(co) = rigidbody_set.bodies.get_mut(handle.0) {
            co.set_enabled(false);
        }
    }
}

/// System responsible for writing the result of the last simulation step into our `bevy_rapier`
/// components and the [`GlobalTransform`] component.
pub fn writeback_rigid_bodies(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    timestep_mode: Res<TimestepMode>,
    config: Query<&RapierConfiguration>,
    sim_to_render_time: Query<&SimulationToRenderTime>,
    global_transforms: Query<&GlobalTransform>,
    mut writeback: Query<
        RigidBodyWritebackComponents,
        (With<RigidBody>, Without<RigidBodyDisabled>),
    >,
) {
    for (
        handle,
        link,
        child_of,
        transform,
        mut interpolation,
        mut velocity,
        mut sleeping,
        mut world_mass_props,
    ) in writeback.iter_mut()
    {
        let config = config
            .get(link.0)
            .expect("Could not get `RapierConfiguration`");
        if !config.physics_pipeline_active {
            continue;
        }
        let handle = handle.0;

        let rigid_body_set = rigid_body_sets
            .get_mut(link.0)
            .expect(RAPIER_CONTEXT_EXPECT_ERROR)
            .into_inner();
        let sim_to_render_time = sim_to_render_time
            .get(link.0)
            .expect("Could not get `SimulationToRenderTime`");
        // TODO: do this the other way round: iterate through Rapier’s RigidBodySet on the active bodies,
        // and update the components accordingly. That way, we don’t have to iterate through the entities that weren’t changed
        // by physics (for example because they are sleeping).
        if let Some(rb) = rigid_body_set.bodies.get(handle) {
            let mut interpolated_pos = utils::iso_to_transform(rb.position());

            if let TimestepMode::Interpolated { dt, .. } = *timestep_mode {
                if let Some(interpolation) = interpolation.as_deref_mut() {
                    if interpolation.end.is_none() {
                        interpolation.end = Some(*rb.position());
                    }

                    if let Some(interpolated) =
                        interpolation.lerp_slerp((dt + sim_to_render_time.diff) / dt)
                    {
                        interpolated_pos = utils::iso_to_transform(&interpolated);
                    }
                }
            }

            if let Some(mut transform) = transform {
                // NOTE: Rapier's `RigidBody` doesn't know its own scale as it is encoded
                //       directly within its collider, so we have to retrieve it from
                //       the scale of its bevy transform.
                interpolated_pos = interpolated_pos.with_scale(transform.scale);

                // NOTE: we query the parent’s global transform here, which is a bit
                //       unfortunate (performance-wise). An alternative would be to
                //       deduce the parent’s global transform from the current entity’s
                //       global transform. However, this makes it nearly impossible
                //       (because of rounding errors) to predict the exact next value this
                //       entity’s global transform will get after the next transform
                //       propagation, which breaks our transform modification detection
                //       that we do to detect if the user’s transform has to be written
                //       into the rigid-body.
                if let Some(parent_global_transform) =
                    child_of.and_then(|c| global_transforms.get(c.parent()).ok())
                {
                    // We need to compute the new local transform such that:
                    // curr_parent_global_transform * new_transform = interpolated_pos
                    // new_transform = curr_parent_global_transform.inverse() * interpolated_pos
                    let (inverse_parent_scale, inverse_parent_rotation, inverse_parent_translation) =
                        parent_global_transform
                            .affine()
                            .inverse()
                            .to_scale_rotation_translation();
                    let new_rotation = inverse_parent_rotation * interpolated_pos.rotation;

                    #[allow(unused_mut)] // mut is needed in 2D but not in 3D.
                    let mut new_translation = inverse_parent_rotation
                        * inverse_parent_scale
                        * interpolated_pos.translation
                        + inverse_parent_translation;

                    // In 2D, preserve the transform `z` component that may have been set by the user
                    #[cfg(feature = "dim2")]
                    {
                        new_translation.z = transform.translation.z;
                    }

                    if transform.rotation != new_rotation
                        || transform.translation != new_translation
                    {
                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        transform.rotation = new_rotation;
                        transform.translation = new_translation;
                    }

                    // NOTE: we need to compute the result of the next transform propagation
                    //       to make sure that our change detection for transforms is exact
                    //       despite rounding errors.
                    let new_global_transform = parent_global_transform.mul_transform(*transform);

                    rigid_body_set
                        .last_body_transform_set
                        .insert(handle, new_global_transform);
                } else {
                    // In 2D, preserve the transform `z` component that may have been set by the user
                    #[cfg(feature = "dim2")]
                    {
                        interpolated_pos.translation.z = transform.translation.z;
                    }

                    if transform.rotation != interpolated_pos.rotation
                        || transform.translation != interpolated_pos.translation
                    {
                        // NOTE: we write the new value only if there was an
                        //       actual change, in order to not trigger bevy’s
                        //       change tracking when the values didn’t change.
                        transform.rotation = interpolated_pos.rotation;
                        transform.translation = interpolated_pos.translation;
                    }

                    rigid_body_set
                        .last_body_transform_set
                        .insert(handle, GlobalTransform::from(interpolated_pos));
                }
            }

            if let Some(velocity) = &mut velocity {
                let new_vel = Velocity {
                    linear: rb.linvel(),
                    #[cfg(feature = "dim3")]
                    angular: rb.angvel(),
                    #[cfg(feature = "dim2")]
                    angular: rb.angvel(),
                };

                // NOTE: we write the new value only if there was an
                //       actual change, in order to not trigger bevy’s
                //       change tracking when the values didn’t change.
                if **velocity != new_vel {
                    **velocity = new_vel;
                }
            }

            if let Some(sleeping) = &mut sleeping {
                // NOTE: we write the new value only if there was an
                //       actual change, in order to not trigger bevy’s
                //       change tracking when the values didn’t change.
                if sleeping.sleeping != rb.is_sleeping() {
                    sleeping.sleeping = rb.is_sleeping();
                }
            }

            if let Some(world_mass_props) = &mut world_mass_props {
                let new_props = ReadWorldMassProperties::from_rapier(rb.mass_properties());

                // NOTE: we write the new value only if there was an
                //       actual change, in order to not trigger bevy’s
                //       change tracking when the values didn’t change.
                if **world_mass_props != new_props {
                    **world_mass_props = new_props;
                }
            }
        }
    }
}

/// System responsible for creating new Rapier rigid-bodies from the related `bevy_rapier` components.
pub fn init_rigid_bodies(
    mut commands: Commands,
    context_links: RapierContextLinkResolver,
    mut rigidbody_sets: Query<(Entity, &mut RapierRigidBodySet)>,
    rigid_bodies: Query<RigidBodyComponents, Without<RapierRigidBodyHandle>>,
    #[cfg(feature = "dim3")] gyroscopic_forces: Query<&GyroscopicForces>,
) {
    for (
        (entity, entity_context_link),
        rb,
        transform,
        vel,
        additional_mass_props,
        _mass_props,
        locked_axes,
        force,
        gravity_scale,
        (ccd, soft_ccd),
        dominance,
        sleep,
        damping,
        disabled,
        (additional_solver_iters, additional_pgs_iters, fast_rotation),
    ) in rigid_bodies.iter()
    {
        let mut builder = RigidBodyBuilder::new((*rb).into());
        builder = builder.enabled(disabled.is_none());

        if let Some(transform) = transform {
            builder = builder.pose(utils::transform_to_iso(&transform.compute_transform()));
        }

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(vel) = vel {
            builder = builder.linvel(vel.linear.into()).angvel(vel.angular.into());
        }

        if let Some(locked_axes) = locked_axes {
            builder = builder.locked_axes((*locked_axes).into())
        }

        if let Some(gravity_scale) = gravity_scale {
            builder = builder.gravity_scale(gravity_scale.0);
        }

        if let Some(ccd) = ccd {
            builder = builder.ccd_enabled(ccd.enabled)
        }

        if let Some(soft_ccd) = soft_ccd {
            builder = builder.soft_ccd_prediction(soft_ccd.prediction)
        }

        if let Some(dominance) = dominance {
            builder = builder.dominance_group(dominance.groups)
        }

        if let Some(sleep) = sleep {
            builder = builder.sleeping(sleep.sleeping);
        }

        if let Some(damping) = damping {
            builder = builder
                .linear_damping(damping.linear_damping)
                .angular_damping(damping.angular_damping);
        }

        if let Some(mprops) = additional_mass_props {
            builder = match mprops {
                AdditionalMassProperties::MassProperties(mprops) => {
                    builder.additional_mass_properties(mprops.into_rapier())
                }
                AdditionalMassProperties::Mass(mass) => builder.additional_mass(*mass),
            };
        }

        if let Some(added_iters) = additional_solver_iters {
            builder = builder.additional_solver_iterations(added_iters.0);
        }

        if let Some(added_iters) = additional_pgs_iters {
            builder = builder.additional_pgs_iterations(added_iters.0);
        }

        if fast_rotation.is_some() {
            builder = builder.allow_fast_rotation(true);
        }

        #[cfg(feature = "dim3")]
        if let Ok(gyroscopic_forces) = gyroscopic_forces.get(entity) {
            builder = builder.gyroscopic_forces_enabled(gyroscopic_forces.enabled);
        }

        builder = builder.user_data(entity.to_bits() as u128);

        let mut rb = builder.build();

        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        if let Some(force) = force {
            rb.add_force(force.force.into(), false);
            rb.add_torque(force.torque.into(), false);
        }

        // NOTE: we can’t apply impulses yet at this point because
        //       the rigid-body’s mass isn’t up-to-date yet (its
        //       attached colliders, if any, haven’t been created yet).

        if let Some(sleep) = sleep {
            let activation = rb.activation_mut();
            activation.normalized_linear_threshold = sleep.normalized_linear_threshold;
            activation.angular_threshold = sleep.angular_threshold;
            activation.time_until_sleep = sleep.time_until_sleep;
        }
        // Use the RapierContextEntityLink, or insert the context of an ancestor or the default one.
        let context_entity = context_links.resolve(entity, entity_context_link, &mut commands);
        let Some(context_entity) = context_entity else {
            continue;
        };

        let Ok((_, mut rigidbody_set)) = rigidbody_sets.get_mut(context_entity) else {
            log::error!("Could not find entity {context_entity} with rapier context while initializing {entity}");
            continue;
        };
        let handle = rigidbody_set.bodies.insert(rb);
        commands
            .entity(entity)
            .insert(RapierRigidBodyHandle(handle));
        rigidbody_set.entity2body.insert(entity, handle);

        if let Some(transform) = transform {
            rigidbody_set
                .last_body_transform_set
                .insert(handle, *transform);
        }
    }
}

/// This applies the initial impulse given to a rigid-body when it is created.
///
/// This cannot be done inside `init_rigid_bodies` because impulses require the rigid-body
/// mass to be available, which it was not because colliders were not created yet. As a
/// result, we run this system after the collider creation.
pub fn apply_initial_rigid_body_impulses(
    mut context: Query<(&mut RapierRigidBodySet, &RapierContextColliders)>,
    // We can’t use RapierRigidBodyHandle yet because its creation command hasn’t been
    // executed yet.
    mut init_impulses: Query<
        (Entity, &RapierContextEntityLink, &mut ExternalImpulse),
        Without<RapierRigidBodyHandle>,
    >,
) {
    for (entity, link, mut impulse) in init_impulses.iter_mut() {
        let (mut rigidbody_set, context_colliders) =
            context.get_mut(link.0).expect(RAPIER_CONTEXT_EXPECT_ERROR);
        let rigidbody_set = &mut *rigidbody_set;

        let bodies = &mut rigidbody_set.bodies;
        if let Some(rb) = rigidbody_set
            .entity2body
            .get(&entity)
            .and_then(|h| bodies.get_mut(*h))
        {
            // Make sure the mass-properties are computed.
            rb.recompute_mass_properties_from_colliders(&context_colliders.colliders);
            // Apply the impulse.
            rb.apply_impulse(impulse.impulse, false);

            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            rb.apply_torque_impulse(impulse.torque_impulse.into(), false);

            impulse.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugin::{NoUserData, RapierPhysicsPlugin};
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use rapier::dynamics::LockedAxes as RapierLockedAxes;

    fn test_app() -> App {
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
        app
    }

    fn with_body<R>(
        app: &mut App,
        entity: Entity,
        f: impl FnOnce(&RapierRigidBodySet, &rapier::dynamics::RigidBody) -> R,
    ) -> R {
        let world = app.world_mut();
        let set = world.query::<&RapierRigidBodySet>().single(world).unwrap();
        let rb = set.bodies.get(set.entity2body()[&entity]).unwrap();
        f(set, rb)
    }

    #[test]
    fn rigid_body_components_are_reset_on_removal() {
        let mut app = test_app();
        app.update();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                ExternalForce {
                    force: Vect::X,
                    ..default()
                },
                Damping {
                    linear_damping: 1.0,
                    angular_damping: 2.0,
                },
                GravityScale(3.0),
                (Ccd::enabled(), SoftCcd { prediction: 4.0 }),
                Dominance::group(5),
                LockedAxes::ROTATION_LOCKED,
                AdditionalMassProperties::Mass(6.0),
                (AdditionalSolverIterations(7), AdditionalPgsIterations(8)),
                AllowFastRotation,
                Sleeping {
                    time_until_sleep: 9.0,
                    ..Sleeping::disabled()
                },
            ))
            .id();
        #[cfg(feature = "dim3")]
        app.world_mut()
            .entity_mut(entity)
            .insert(GyroscopicForces::disabled());
        app.update();

        with_body(&mut app, entity, |_, rb| {
            assert_eq!(rb.user_force(), Vect::X);
            assert_eq!(rb.linear_damping(), 1.0);
            assert_eq!(rb.angular_damping(), 2.0);
            assert_eq!(rb.gravity_scale(), 3.0);
            assert!(rb.is_ccd_enabled());
            assert_eq!(rb.soft_ccd_prediction(), 4.0);
            assert_eq!(rb.dominance_group(), 5);
            assert_eq!(rb.locked_axes(), RapierLockedAxes::ROTATION_LOCKED);
            assert!(rb.mass_properties().additional_local_mprops.is_some());
            assert_eq!(rb.additional_solver_iterations(), 7);
            assert_eq!(rb.additional_pgs_iterations(), 8);
            assert!(rb.is_fast_rotation_allowed());
            assert_eq!(rb.activation().normalized_linear_threshold, -1.0);
            assert_eq!(rb.activation().time_until_sleep, 9.0);
            #[cfg(feature = "dim3")]
            assert!(!rb.gyroscopic_forces_enabled());
        });

        app.world_mut().entity_mut(entity).remove::<(
            ExternalForce,
            Damping,
            GravityScale,
            Ccd,
            SoftCcd,
            Dominance,
            LockedAxes,
            AdditionalMassProperties,
            AdditionalSolverIterations,
            AdditionalPgsIterations,
            AllowFastRotation,
            Sleeping,
        )>();
        #[cfg(feature = "dim3")]
        app.world_mut()
            .entity_mut(entity)
            .remove::<GyroscopicForces>();
        app.update();

        let default_sleeping = Sleeping::default();
        with_body(&mut app, entity, |_, rb| {
            assert_eq!(rb.user_force(), Vect::ZERO);
            assert_eq!(rb.linear_damping(), 0.0);
            assert_eq!(rb.angular_damping(), 0.0);
            assert_eq!(rb.gravity_scale(), 1.0);
            assert!(!rb.is_ccd_enabled());
            assert_eq!(rb.soft_ccd_prediction(), 0.0);
            assert_eq!(rb.dominance_group(), 0);
            assert_eq!(rb.locked_axes(), RapierLockedAxes::empty());
            assert_eq!(rb.mass(), 0.0);
            assert_eq!(rb.additional_solver_iterations(), 0);
            assert_eq!(rb.additional_pgs_iterations(), 0);
            assert!(!rb.is_fast_rotation_allowed());
            assert_eq!(
                rb.activation().normalized_linear_threshold,
                default_sleeping.normalized_linear_threshold
            );
            assert_eq!(
                rb.activation().angular_threshold,
                default_sleeping.angular_threshold
            );
            assert_eq!(
                rb.activation().time_until_sleep,
                default_sleeping.time_until_sleep
            );
            #[cfg(feature = "dim3")]
            assert!(rb.gyroscopic_forces_enabled());
        });
    }

    #[test]
    fn component_reinserted_in_same_frame_is_not_reset() {
        let mut app = test_app();
        app.update();

        let entity = app
            .world_mut()
            .spawn((Transform::default(), RigidBody::Dynamic, GravityScale(2.0)))
            .id();
        app.update();

        app.world_mut()
            .entity_mut(entity)
            .remove::<GravityScale>()
            .insert(GravityScale(3.0));
        app.update();

        with_body(&mut app, entity, |_, rb| {
            assert_eq!(rb.gravity_scale(), 3.0)
        });
    }

    #[test]
    fn world_mass_properties_are_written_back() {
        let mut app = test_app();
        app.update();

        let translation = Vec3::new(1.0, 2.0, 0.0);
        let entity = app
            .world_mut()
            .spawn((
                Transform::from_translation(translation),
                RigidBody::Dynamic,
                Collider::ball(0.5),
                LockedAxes::TRANSLATION_LOCKED_X,
                GravityScale(0.0),
                ReadWorldMassProperties::default(),
            ))
            .id();
        app.update();

        let props = *app.world().get::<ReadWorldMassProperties>(entity).unwrap();
        #[cfg(feature = "dim2")]
        let expected_com = translation.truncate();
        #[cfg(feature = "dim3")]
        let expected_com = translation;
        approx::assert_relative_eq!(props.center_of_mass, expected_com, epsilon = 1.0e-5);
        assert_eq!(props.effective_inv_mass.x, 0.0);
        assert!(props.effective_inv_mass.y > 0.0);

        with_body(&mut app, entity, |set, rb| {
            assert_eq!(set.center_of_mass(entity), Some(props.center_of_mass));
            assert_eq!(set.mass(entity), Some(rb.mass()));
            assert_eq!(set.is_moving(entity), Some(false));
            assert_eq!(set.effective_dominance_group(entity), Some(0));
            assert_eq!(set.center_of_mass(Entity::PLACEHOLDER), None);
        });
    }

    #[test]
    fn waking_up_a_body_wakes_up_its_island() {
        let mut app = test_app();
        app.update();

        let spawn_body = |app: &mut App, x: f32| {
            app.world_mut()
                .spawn((
                    Transform::from_xyz(x, 0.0, 0.0),
                    RigidBody::Dynamic,
                    Collider::ball(0.5),
                    GravityScale(0.0),
                    Sleeping::default(),
                ))
                .id()
        };
        let body1 = spawn_body(&mut app, 0.0);
        let body2 = spawn_body(&mut app, 2.0);
        app.world_mut().entity_mut(body2).insert(ImpulseJoint::new(
            body1,
            FixedJointBuilder::new().local_anchor1(Vect::X * 2.0),
        ));

        for _ in 0..120 {
            app.update();
        }
        assert!(app.world().get::<Sleeping>(body1).unwrap().sleeping);
        assert!(app.world().get::<Sleeping>(body2).unwrap().sleeping);

        app.world_mut().get_mut::<Sleeping>(body2).unwrap().sleeping = false;
        app.update();

        with_body(&mut app, body1, |_, rb| assert!(!rb.is_sleeping()));
        with_body(&mut app, body2, |_, rb| assert!(!rb.is_sleeping()));
    }
}
