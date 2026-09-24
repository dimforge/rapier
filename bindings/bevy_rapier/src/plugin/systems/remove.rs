use crate::dynamics::ImpulseJoint;
use crate::dynamics::ImpulseJointDisabled;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::dynamics::RapierRigidBodyHandle;
use crate::dynamics::RigidBody;
use crate::dynamics::SoftBody;
use crate::dynamics::SoftBodyCluster;
use crate::geometry::Collider;
use crate::geometry::ColliderDisabled;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};
#[cfg(feature = "dim3")]
use crate::prelude::GyroscopicForces;
use crate::prelude::{
    ActiveCollisionTypes, ActiveEvents, ActiveHooks, AdditionalMassProperties,
    AdditionalPgsIterations, AdditionalSolverIterations, AllowFastRotation, Ccd,
    ColliderMassProperties, CollisionGroups, ContactForceEventThreshold, ContactSkin, Damping,
    Dominance, ExternalForce, Friction, GravityScale, LockedAxes, MassModifiedEvent,
    MassProperties, ReadColliderMassProperties, Restitution, RigidBodyDisabled, Sensor, Sleeping,
    SoftCcd, SolverGroups,
};
use bevy::ecs::query::IterQueryData;
use bevy::prelude::*;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context_writer: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    // Sometimes a Remove immediately followed by Add happens. These `q_has_*` queries prevent that immediate Add
    // from being removed by this system by verifying it's still removed.
    (
        q_has_rigidbody_handle,
        q_has_collider_handle,
        q_has_multibody_joint_handle,
        q_has_impulse_joint_handle,
    ): (
        Query<(), With<RapierRigidBodyHandle>>,
        Query<(), With<RapierColliderHandle>>,
        Query<(), With<RapierMultibodyJointHandle>>,
        Query<(), With<RapierImpulseJointHandle>>,
    ),
    mut removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    mut removed_colliders: RemovedComponents<RapierColliderHandle>,
    mut removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    mut removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<
        Entity,
        (
            With<RapierRigidBodyHandle>,
            Without<RigidBody>,
            Without<SoftBodyCluster>,
        ),
    >,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,

    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,
    (mut removed_impulse_joints_disabled, mut impulse_joints): (
        RemovedComponents<ImpulseJointDisabled>,
        Query<&mut ImpulseJoint, Without<ImpulseJointDisabled>>,
    ),

    mut mass_modified: MessageWriter<MassModifiedEvent>,
) {
    /*
     * Rigid-bodies removal detection.
     */
    for entity in removed_bodies
        .read()
        .filter(|e| !q_has_rigidbody_handle.contains(*e))
    {
        let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        else {
            continue;
        };
        let context = &mut *context;
        let joints = &mut *joints;
        let rigidbody_set = &mut *rigidbody_set;

        let _ = rigidbody_set.last_body_transform_set.remove(&handle);
        rigidbody_set.bodies.remove(
            handle,
            &mut context.islands,
            &mut context_colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
            &mut rigidbody_set.soft_bodies,
            false,
        );
    }

    for entity in orphan_bodies.iter() {
        if let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        {
            let context = &mut *context;
            let joints = &mut *joints;
            let rigidbody_set = &mut *rigidbody_set;
            let _ = rigidbody_set.last_body_transform_set.remove(&handle);
            rigidbody_set.bodies.remove(
                handle,
                &mut context.islands,
                &mut context_colliders.colliders,
                &mut joints.impulse_joints,
                &mut joints.multibody_joints,
                &mut rigidbody_set.soft_bodies,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders
        .read()
        .filter(|e| !q_has_collider_handle.contains(*e))
    {
        let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        else {
            continue;
        };
        let context = &mut *context;
        let rigidbody_set = &mut *rigidbody_set;
        if let Some(parent) = context_colliders.collider_parent(rigidbody_set, entity) {
            mass_modified.write(parent.into());
        }

        context_colliders.colliders.remove(
            handle,
            &mut context.islands,
            &mut rigidbody_set.bodies,
            &mut rigidbody_set.soft_bodies,
            true,
        );
        context.deleted_colliders.insert(handle, entity);
    }

    for entity in orphan_colliders.iter() {
        if let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        {
            let context = &mut *context;
            let context_colliders = &mut *context_colliders;
            let rigidbody_set = &mut *rigidbody_set;
            if let Some(parent) = context_colliders.collider_parent(rigidbody_set, entity) {
                mass_modified.write(parent.into());
            }

            context_colliders.colliders.remove(
                handle,
                &mut context.islands,
                &mut rigidbody_set.bodies,
                &mut rigidbody_set.soft_bodies,
                true,
            );
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints
        .read()
        .filter(|e| !q_has_impulse_joint_handle.contains(*e))
    {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.impulse_joints.remove(handle, true);
    }

    for entity in orphan_impulse_joints.iter() {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) {
            joints.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints
        .read()
        .filter(|e| !q_has_multibody_joint_handle.contains(*e))
    {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.multibody_joints.remove(handle, true);
    }

    for entity in orphan_multibody_joints.iter() {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) {
            joints.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors.read() {
        for (_, mut colliders, _, rigidbody_set) in context_writer.iter_mut() {
            // A soft body entity stands for all its surface colliders.
            let handles = rigidbody_set
                .soft_body_colliders(&colliders.colliders, entity)
                .or_else(|| colliders.entity2collider.get(&entity).map(|h| vec![*h]));
            if let Some(handles) = handles {
                for handle in handles {
                    if let Some(co) = colliders.colliders.get_mut(handle) {
                        co.set_sensor(false);
                    }
                }
                break;
            }
        }
    }

    for entity in removed_colliders_disabled.read() {
        if let Some((mut context, handle)) = find_context(&mut context_writer, |context| {
            context.1.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_enabled(true);
            }
        }
    }

    // Flag the joint as changed so `apply_joint_user_changes` enables it again.
    for entity in removed_impulse_joints_disabled.read() {
        if let Ok(mut joint) = impulse_joints.get_mut(entity) {
            joint.set_changed();
        }
    }

    for entity in removed_rigid_body_disabled.read() {
        if let Some(((_, _, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.3.entity2body.get(&entity).copied()
            })
        {
            if let Some(rb) = rigidbody_set.bodies.get_mut(handle) {
                rb.set_enabled(true);
            }
        }
    }
}

/// Removal detection for a single component: the entities it was removed from, paired with a
/// query checking if it was re-inserted since.
type RemovedComponent<'w, 's, T> = (RemovedComponents<'w, 's, T>, Query<'w, 's, (), With<T>>);

/// System responsible for resetting the Rapier rigid-body properties to their default values
/// when the corresponding `bevy_rapier` component is removed by the user.
pub fn reset_removed_rigid_body_components(
    mut rigid_body_sets: Query<&mut RapierRigidBodySet>,
    (mut removed_forces, mut removed_damping, mut removed_gravity_scale, mut removed_dominance): (
        RemovedComponent<ExternalForce>,
        RemovedComponent<Damping>,
        RemovedComponent<GravityScale>,
        RemovedComponent<Dominance>,
    ),
    (mut removed_ccd, mut removed_soft_ccd, mut removed_fast_rotation): (
        RemovedComponent<Ccd>,
        RemovedComponent<SoftCcd>,
        RemovedComponent<AllowFastRotation>,
    ),
    (mut removed_locked_axes, mut removed_mass_props, mut removed_sleeping): (
        RemovedComponent<LockedAxes>,
        RemovedComponent<AdditionalMassProperties>,
        RemovedComponent<Sleeping>,
    ),
    (mut removed_solver_iters, mut removed_pgs_iters): (
        RemovedComponent<AdditionalSolverIterations>,
        RemovedComponent<AdditionalPgsIterations>,
    ),
    #[cfg(feature = "dim3")] mut removed_gyroscopic_forces: RemovedComponent<GyroscopicForces>,
    mut mass_modified: MessageWriter<MassModifiedEvent>,
) {
    let sets = &mut rigid_body_sets;
    reset_removed(&mut removed_forces, sets, |_, rb| {
        rb.reset_forces(true);
        rb.reset_torques(true);
    });
    reset_removed(&mut removed_damping, sets, |_, rb| {
        let default = Damping::default();
        rb.set_linear_damping(default.linear_damping);
        rb.set_angular_damping(default.angular_damping);
    });
    reset_removed(&mut removed_gravity_scale, sets, |_, rb| {
        rb.set_gravity_scale(GravityScale::default().0, true);
    });
    reset_removed(&mut removed_dominance, sets, |_, rb| {
        rb.set_dominance_group(Dominance::default().groups);
    });
    reset_removed(&mut removed_ccd, sets, |_, rb| {
        rb.enable_ccd(Ccd::default().enabled);
    });
    reset_removed(&mut removed_soft_ccd, sets, |_, rb| {
        rb.set_soft_ccd_prediction(SoftCcd::default().prediction);
    });
    reset_removed(&mut removed_fast_rotation, sets, |_, rb| {
        rb.set_allow_fast_rotation(false);
    });
    reset_removed(&mut removed_locked_axes, sets, |_, rb| {
        rb.set_locked_axes(LockedAxes::default().into(), true);
    });
    reset_removed(&mut removed_mass_props, sets, |entity, rb| {
        rb.set_additional_mass_properties(MassProperties::default().into_rapier(), true);
        mass_modified.write(entity.into());
    });
    reset_removed(&mut removed_sleeping, sets, |_, rb| {
        let default = Sleeping::default();
        let activation = rb.activation_mut();
        activation.normalized_linear_threshold = default.normalized_linear_threshold;
        activation.angular_threshold = default.angular_threshold;
        activation.time_until_sleep = default.time_until_sleep;
    });
    reset_removed(&mut removed_solver_iters, sets, |_, rb| {
        rb.set_additional_solver_iterations(AdditionalSolverIterations::default().0);
    });
    reset_removed(&mut removed_pgs_iters, sets, |_, rb| {
        rb.set_additional_pgs_iterations(AdditionalPgsIterations::default().0);
    });
    #[cfg(feature = "dim3")]
    reset_removed(&mut removed_gyroscopic_forces, sets, |_, rb| {
        rb.enable_gyroscopic_forces(GyroscopicForces::default().enabled);
    });
}

/// Calls `reset` on the Rapier rigid-body of each entity `T` was removed from, unless `T` was
/// inserted back since.
fn reset_removed<T: Component>(
    (removed, still_present): &mut RemovedComponent<T>,
    rigid_body_sets: &mut Query<&mut RapierRigidBodySet>,
    mut reset: impl FnMut(Entity, &mut rapier::dynamics::RigidBody),
) {
    for entity in removed.read() {
        if still_present.contains(entity) {
            continue;
        }

        for mut rigid_body_set in rigid_body_sets.iter_mut() {
            if let Some(handle) = rigid_body_set.entity2body.get(&entity).copied() {
                if let Some(rb) = rigid_body_set.bodies.get_mut(handle) {
                    reset(entity, rb);
                }
                break;
            }
        }
    }
}

/// System responsible for resetting the Rapier collider properties to their default values when
/// the corresponding `bevy_rapier` component is removed by the user.
///
/// The default values are the ones a collider gets when inserted without that component.
pub fn reset_removed_collider_components(
    mut contexts: Query<(&mut RapierContextColliders, &RapierRigidBodySet)>,
    soft_bodies: Query<&SoftBody>,
    (mut removed_friction, mut removed_restitution, mut removed_contact_skin): (
        RemovedComponent<Friction>,
        RemovedComponent<Restitution>,
        RemovedComponent<ContactSkin>,
    ),
    (mut removed_collision_groups, mut removed_solver_groups): (
        RemovedComponent<CollisionGroups>,
        RemovedComponent<SolverGroups>,
    ),
    (mut removed_events, mut removed_hooks, mut removed_collision_types): (
        RemovedComponent<ActiveEvents>,
        RemovedComponent<ActiveHooks>,
        RemovedComponent<ActiveCollisionTypes>,
    ),
    (mut removed_force_threshold, mut removed_mass_props): (
        RemovedComponent<ContactForceEventThreshold>,
        RemovedComponent<ColliderMassProperties>,
    ),
    mut read_mass_props: Query<&mut ReadColliderMassProperties>,
    mut mass_modified: MessageWriter<MassModifiedEvent>,
) {
    let ctxts = &mut contexts;
    let sbs = &soft_bodies;
    reset_removed_collider(&mut removed_friction, ctxts, sbs, |_, co, _, defaults| {
        co.set_friction(defaults.friction);
        co.set_friction_combine_rule(defaults.friction_combine_rule);
    });
    reset_removed_collider(
        &mut removed_restitution,
        ctxts,
        sbs,
        |_, co, _, defaults| {
            co.set_restitution(defaults.restitution);
            co.set_restitution_combine_rule(defaults.restitution_combine_rule);
        },
    );
    reset_removed_collider(
        &mut removed_contact_skin,
        ctxts,
        sbs,
        |entity, co, rigid_body_set, defaults| {
            // The surface colliders of a soft body are as thick as its particles.
            let skin = rigid_body_set
                .soft_body(entity)
                .map_or(defaults.contact_skin, |sb| sb.particle_radius());
            co.set_contact_skin(skin);
        },
    );
    reset_removed_collider(
        &mut removed_collision_groups,
        ctxts,
        sbs,
        |_, co, _, defaults| {
            co.set_collision_groups(defaults.collision_groups);
        },
    );
    reset_removed_collider(
        &mut removed_solver_groups,
        ctxts,
        sbs,
        |_, co, _, defaults| {
            co.set_solver_groups(defaults.solver_groups);
        },
    );
    reset_removed_collider(&mut removed_events, ctxts, sbs, |_, co, _, defaults| {
        co.set_active_events(defaults.active_events);
    });
    reset_removed_collider(&mut removed_hooks, ctxts, sbs, |_, co, _, defaults| {
        co.set_active_hooks(defaults.active_hooks);
    });
    reset_removed_collider(
        &mut removed_collision_types,
        ctxts,
        sbs,
        |_, co, _, defaults| {
            co.set_active_collision_types(defaults.active_collision_types);
        },
    );
    reset_removed_collider(
        &mut removed_force_threshold,
        ctxts,
        sbs,
        |_, co, _, defaults| {
            co.set_contact_force_event_threshold(defaults.contact_force_event_threshold);
        },
    );
    reset_removed_collider(
        &mut removed_mass_props,
        ctxts,
        sbs,
        |entity, co, rigid_body_set, _| {
            // The surface colliders of a soft body have no mass.
            if rigid_body_set.soft_body(entity).is_some() {
                return;
            }
            co.set_density(rapier::geometry::ColliderBuilder::default_density());

            if let Ok(mut read_mass_props) = read_mass_props.get_mut(entity) {
                read_mass_props.set_if_neq(ReadColliderMassProperties::from_rapier(co));
            }

            if let Some(body_entity) = co
                .parent()
                .and_then(|h| rigid_body_set.rigid_body_entity(h))
            {
                mass_modified.write(body_entity.into());
            }
        },
    );
}

/// Calls `reset` on the Rapier collider of each entity `T` was removed from, unless `T` was
/// inserted back since.
///
/// For a soft body entity, `reset` is called on each of its surface colliders (see
/// [`RapierRigidBodySet::soft_body_colliders`]), with the collider template of its builder as
/// defaults.
fn reset_removed_collider<T: Component>(
    (removed, still_present): &mut RemovedComponent<T>,
    contexts: &mut Query<(&mut RapierContextColliders, &RapierRigidBodySet)>,
    soft_bodies: &Query<&SoftBody>,
    mut reset: impl FnMut(
        Entity,
        &mut rapier::geometry::Collider,
        &RapierRigidBodySet,
        &rapier::geometry::ColliderBuilder,
    ),
) {
    let defaults = rapier::geometry::ColliderBuilder::default();
    for entity in removed.read() {
        if still_present.contains(entity) {
            continue;
        }

        for (mut context_colliders, rigid_body_set) in contexts.iter_mut() {
            if let Some(handles) =
                rigid_body_set.soft_body_colliders(&context_colliders.colliders, entity)
            {
                let template = soft_bodies
                    .get(entity)
                    .ok()
                    .and_then(|sb| sb.builder.collider_template.as_ref())
                    .unwrap_or(&defaults);
                for handle in handles {
                    if let Some(co) = context_colliders.colliders.get_mut(handle) {
                        reset(entity, co, rigid_body_set, template);
                    }
                }
                break;
            }
            if let Some(handle) = context_colliders.entity2collider.get(&entity).copied() {
                if let Some(co) = context_colliders.colliders.get_mut(handle) {
                    reset(entity, co, rigid_body_set, &defaults);
                }
                break;
            }
        }
    }
}

fn find_context<'a, TReturn, TQueryParams: IterQueryData>(
    context_writer: &'a mut Query<TQueryParams>,
    item_finder: impl Fn(&mut TQueryParams::Item<'_, '_>) -> Option<TReturn>,
) -> Option<(TQueryParams::Item<'a, 'a>, TReturn)> {
    let ret: Option<(TQueryParams::Item<'_, '_>, TReturn)> = context_writer
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)));
    ret
}
