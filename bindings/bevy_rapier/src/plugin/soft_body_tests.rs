//! Tests of the soft-body integration.

use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::time::{TimePlugin, TimeUpdateStrategy};

use crate::math::Real;
use crate::plugin::context::DefaultRapierContext;
use crate::prelude::*;

/// An app running the physics at a fixed 60Hz timestep.
fn test_app() -> App {
    test_app_with(|_| {})
}

/// Same as [`test_app`], calling `setup` before finishing the app.
fn test_app_with(setup: impl FnOnce(&mut App)) -> App {
    let mut app = App::new();
    setup(&mut app);
    app.add_plugins((
        TransformPlugin,
        TimePlugin,
        RapierPhysicsPlugin::<NoUserData>::default(),
    ));
    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1.0 / 60.0),
    ));
    app.finish();
    // Create the default context.
    app.update();
    app
}

fn run(app: &mut App, frames: usize) {
    for _ in 0..frames {
        app.update();
    }
}

fn context_entity(app: &mut App) -> Entity {
    app.world_mut()
        .query_filtered::<Entity, With<DefaultRapierContext>>()
        .single(app.world())
        .unwrap()
}

fn rigid_body_set(app: &mut App) -> &RapierRigidBodySet {
    let context = context_entity(app);
    app.world().get::<RapierRigidBodySet>(context).unwrap()
}

fn colliders(app: &mut App) -> &RapierContextColliders {
    let context = context_entity(app);
    app.world().get::<RapierContextColliders>(context).unwrap()
}

#[cfg(feature = "dim3")]
fn v(x: Real, y: Real) -> Vect {
    Vect::new(x, y, 0.0)
}
#[cfg(feature = "dim2")]
fn v(x: Real, y: Real) -> Vect {
    Vect::new(x, y)
}

#[cfg(feature = "dim3")]
fn ground() -> Collider {
    Collider::cuboid(10.0, 0.5, 10.0)
}
#[cfg(feature = "dim2")]
fn ground() -> Collider {
    Collider::cuboid(10.0, 0.5)
}

/// A small volumetric soft body centered at the entity's origin.
#[cfg(feature = "dim3")]
fn soft_box() -> SoftBody {
    SoftBody::cuboid(Vect::splat(0.5), 3, 3, 3)
}
#[cfg(feature = "dim2")]
fn soft_box() -> SoftBody {
    SoftBody::grid(Vect::splat(0.5), 4, 4)
}

#[derive(Resource)]
struct Received<M: Message> {
    messages: Vec<M>,
}

impl<M: Message> Default for Received<M> {
    fn default() -> Self {
        Self { messages: vec![] }
    }
}

fn collect<M: Message + Clone>(app: &mut App) {
    app.init_resource::<Received<M>>().add_systems(
        PostUpdate,
        (|mut reader: MessageReader<M>, mut received: ResMut<Received<M>>| {
            received.messages.extend(reader.read().cloned());
        })
        .after(PhysicsSet::Writeback),
    );
}

/// Tears the soft body of `body` along its edge `edge` from a system.
fn tear(app: &mut App, body: Entity, edge: u32) -> SoftBodyTearResult {
    app.world_mut()
        .run_system_once(
            move |mut context: WriteRapierContext, mut commands: Commands| {
                context
                    .single_mut()
                    .unwrap()
                    .tear_soft_body(&mut commands, body, &[edge], &[])
            },
        )
        .unwrap()
        .expect("the soft body tore")
}

/// A rope of 9 particles along `x`, pinned at its first particle.
fn pinned_rope() -> SoftBody {
    SoftBody::rope(v(0.0, 0.0), v(4.0, 0.0), 9).map(|b| b.pinned_particles([0]).can_sleep(false))
}

#[test]
fn soft_body_falls_and_rests_on_ground() {
    let mut app = test_app();
    app.world_mut()
        .spawn((Transform::from_xyz(0.0, -0.5, 0.0), ground()));
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), soft_box()))
        .id();
    app.update();

    // The particles were placed by the entity's transform.
    let com = rigid_body_set(&mut app)
        .soft_body_center_of_mass(body)
        .unwrap();
    assert!((com.y - 3.0).abs() < 0.1, "{com}");
    assert!(app.world().get::<RapierSoftBodyHandle>(body).is_some());

    run(&mut app, 30);
    let state = *app.world().get::<SoftBodyState>(body).unwrap();
    assert!(state.center_of_mass.y < 2.9, "{state:?}");

    run(&mut app, 300);
    let state = *app.world().get::<SoftBodyState>(body).unwrap();
    // Resting on the ground (its top is at `y = 0`): half-height 0.5, plus the contact skin.
    assert!(
        state.center_of_mass.y > 0.3 && state.center_of_mass.y < 1.0,
        "{state:?}"
    );
    let set = rigid_body_set(&mut app);
    let max_speed = set
        .soft_body_particle_velocities(body)
        .unwrap()
        .map(|vel| vel.length())
        .fold(0.0, Real::max);
    assert!(max_speed < 0.1, "{max_speed}");

    // The entity's transform follows the whole-body proxy, centered on the free particles.
    let proxy = set.soft_body_whole_proxy(body).unwrap();
    let expected = crate::utils::iso_to_transform(set.bodies[proxy].position());
    let transform = *app.world().get::<Transform>(body).unwrap();
    assert!((transform.translation.x - state.center_of_mass.x).abs() < 1.0e-5);
    assert!((transform.translation.y - state.center_of_mass.y).abs() < 1.0e-5);
    assert!((transform.translation - expected.translation).length() < 1.0e-5);
    assert!(transform.rotation.angle_between(expected.rotation) < 1.0e-5);
}

#[test]
fn soft_body_children_follow_the_whole_body_frame() {
    let mut app = test_app();
    let mut child = Entity::PLACEHOLDER;
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box().map(|b| b.can_sleep(false).gravity_scale(0.0)),
        ))
        .with_children(|builder| {
            child = builder
                .spawn((Transform::from_xyz(1.0, 0.0, 0.0), Collider::ball(0.2)))
                .id();
        })
        .id();
    app.update();

    // Spin the soft body around `z` while moving it (Rapier keeps the pose of a 2D proxy
    // rotating in place counterclockwise).
    let impulses = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        let center = sb.center_of_mass();
        sb.particles()
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let r = p.position() - center;
                (i as u32, v(-r.y, r.x) * 2.0 * p.mass())
            })
            .collect()
    };
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyExternalImpulse {
            velocity_change: v(0.5, 0.0),
            particle_impulses: impulses,
        });
    run(&mut app, 20);

    let transform = *app.world().get::<Transform>(body).unwrap();
    let (_, angle) = transform.rotation.to_axis_angle();
    assert!(angle > 0.2, "{angle}");
    // The child entity is where its collider is attached on the whole-body proxy.
    let handle = app.world().get::<RapierColliderHandle>(child).unwrap().0;
    let attachment = {
        let set = rigid_body_set(&mut app);
        let proxy = set.soft_body_whole_proxy(body).unwrap();
        let proxy_pose = *set.bodies[proxy].position();
        let co = &colliders(&mut app).colliders[handle];
        assert_eq!(co.parent(), Some(proxy));
        proxy_pose * co.position_wrt_parent().copied().unwrap()
    };
    let collider_pose = crate::utils::iso_to_transform(&attachment);
    let child_pose = app
        .world()
        .get::<GlobalTransform>(child)
        .unwrap()
        .compute_transform();
    #[cfg(feature = "dim2")]
    let child_translation = child_pose.translation.truncate().extend(0.0);
    #[cfg(feature = "dim3")]
    let child_translation = child_pose.translation;
    assert!(
        (child_translation - collider_pose.translation).length() < 1.0e-3,
        "{child_translation} != {}",
        collider_pose.translation
    );
    assert!(child_pose.rotation.angle_between(collider_pose.rotation) < 1.0e-3);
}

#[test]
fn soft_body_collides_as_its_entity() {
    let mut app = test_app();
    collect::<CollisionEvent>(&mut app);
    let ground = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, -0.5, 0.0), ground()))
        .id();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.0, 0.0),
            soft_box(),
            ActiveEvents::COLLISION_EVENTS,
        ))
        .id();
    run(&mut app, 120);

    let events = &app.world().resource::<Received<CollisionEvent>>().messages;
    assert!(events.iter().any(|event| matches!(
        event,
        CollisionEvent::Started(a, b, _) if (*a, *b) == (body, ground) || (*a, *b) == (ground, body)
    )));
    // The soft body entity maps to its surface collider.
    let collider = colliders(&mut app).entity2collider()[&body];
    assert_eq!(colliders(&mut app).collider_entity(collider), Some(body));
}

#[test]
fn despawn_removes_soft_body() {
    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), soft_box()))
        .id();
    let other = app
        .world_mut()
        .spawn((Transform::from_xyz(5.0, 3.0, 0.0), soft_box()))
        .id();
    run(&mut app, 5);
    assert_eq!(rigid_body_set(&mut app).soft_bodies.len(), 2);
    assert!(!colliders(&mut app).colliders.is_empty());

    app.world_mut().despawn(body);
    run(&mut app, 2);
    let set = rigid_body_set(&mut app);
    assert_eq!(set.soft_bodies.len(), 1);
    assert!(set.soft_body(body).is_none());
    assert!(!set.entity2soft_body().contains_key(&body));
    assert!(!colliders(&mut app).entity2collider().contains_key(&body));

    // Removing the component also removes the soft body.
    app.world_mut().entity_mut(other).remove::<SoftBody>();
    run(&mut app, 2);
    let set = rigid_body_set(&mut app);
    assert!(set.soft_bodies.is_empty());
    assert!(set.bodies.is_empty(), "the proxies weren't removed");
    assert!(colliders(&mut app).colliders.is_empty());
    assert!(app.world().get::<RapierSoftBodyHandle>(other).is_none());
}

#[test]
fn pinned_particles_stay_put() {
    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), pinned_rope()))
        .id();
    run(&mut app, 60);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!((sb.particle_position(0) - v(0.0, 3.0)).length() < 1.0e-5);
    assert!(sb.particle_position(8).y < 2.0);

    // Pin the last particle too, then move it with a kinematic target.
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyPinnedParticles(vec![0, 8]));
    app.update();
    let held = rigid_body_set(&mut app)
        .soft_body(body)
        .unwrap()
        .particle_position(8);
    run(&mut app, 30);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!(sb.particles()[8].is_pinned());
    assert!((sb.particle_position(8) - held).length() < 1.0e-5);

    let target = v(4.0, 3.0);
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyKinematicTargets(vec![(8, target)]));
    run(&mut app, 3);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!((sb.particle_position(8) - target).length() < 1.0e-4);

    // Removing the component restores the builder's pinned particles.
    app.world_mut()
        .entity_mut(body)
        .remove::<SoftBodyPinnedParticles>();
    run(&mut app, 30);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!(sb.particles()[0].is_pinned());
    assert!(!sb.particles()[8].is_pinned());
    assert!(sb.particle_position(8).y < 2.9);
}

#[test]
fn soft_body_components_are_applied_and_reset() {
    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box(),
            SoftBodyMaterial::default(),
        ))
        .id();
    run(&mut app, 2);

    let mut material = SoftBodyMaterial::uniform(12.0, 0.5);
    material.tear_strain = Some(0.5);
    app.world_mut().entity_mut(body).insert(material);
    app.world_mut()
        .entity_mut(body)
        .insert((SoftBodyVolumeFactor(1.5), SoftBodyDisabled));
    app.update();
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert_eq!(*sb.material(), material.0);
    assert_eq!(sb.volume_factor(), 1.5);
    assert!(!sb.is_enabled());
    assert!(!app.world().get::<SoftBodyState>(body).unwrap().is_enabled);

    app.world_mut()
        .entity_mut(body)
        .remove::<(SoftBodyMaterial, SoftBodyVolumeFactor, SoftBodyDisabled)>();
    app.update();
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert_eq!(*sb.material(), soft_box().builder.material);
    assert_eq!(sb.volume_factor(), 1.0);
    assert!(sb.is_enabled());

    // Impulses are applied once, then reset.
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyExternalImpulse {
            velocity_change: v(3.0, 0.0),
            particle_impulses: vec![],
        });
    app.update();
    assert_eq!(
        *app.world().get::<SoftBodyExternalImpulse>(body).unwrap(),
        SoftBodyExternalImpulse::default()
    );
    let vel = rigid_body_set(&mut app)
        .soft_body_particle_velocities(body)
        .unwrap()
        .next()
        .unwrap();
    assert!(vel.x > 2.0, "{vel}");
}

#[test]
fn soft_body_collider_components_are_applied_and_reset() {
    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box(),
            Friction::coefficient(0.3),
        ))
        .id();
    app.update();

    // The friction, sensor flag, contact skin and collision groups of the surface colliders.
    let surface = |app: &mut App| {
        let context = context_entity(app);
        let world = app.world();
        let set = world.get::<RapierRigidBodySet>(context).unwrap();
        let colliders = &world
            .get::<RapierContextColliders>(context)
            .unwrap()
            .colliders;
        let handles = set.soft_body_colliders(colliders, body).unwrap();
        assert!(!handles.is_empty());
        let radius = set.soft_body(body).unwrap().particle_radius();
        let states: Vec<_> = handles
            .iter()
            .map(|h| {
                let co = &colliders[*h];
                (
                    co.friction(),
                    co.is_sensor(),
                    co.contact_skin(),
                    co.collision_groups(),
                )
            })
            .collect();
        (states, radius)
    };
    let (states, _) = surface(&mut app);
    assert!(states.iter().all(|s| s.0 == 0.3), "{states:?}");

    let groups = CollisionGroups::new(Group::GROUP_2, Group::GROUP_3);
    app.world_mut().entity_mut(body).insert((
        Friction::coefficient(0.9),
        Sensor,
        ContactSkin(0.05),
        groups,
    ));
    app.update();
    let (states, _) = surface(&mut app);
    for state in &states {
        assert_eq!(state.0, 0.9);
        assert!(state.1);
        assert_eq!(state.2, 0.05);
        assert_eq!(state.3, groups.into());
    }

    app.world_mut()
        .entity_mut(body)
        .remove::<(Friction, Sensor, ContactSkin, CollisionGroups)>();
    app.update();
    let (states, radius) = surface(&mut app);
    let defaults = rapier::geometry::ColliderBuilder::default();
    for state in &states {
        assert_eq!(state.0, defaults.friction);
        assert!(!state.1);
        assert_eq!(state.2, radius);
        assert_eq!(state.3, defaults.collision_groups);
    }
}

#[test]
fn non_finite_soft_body_is_quarantined() {
    let mut app = test_app();
    collect::<PhysicsQuarantineEvent>(&mut app);
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), soft_box()))
        .id();
    run(&mut app, 2);

    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyExternalImpulse {
            velocity_change: Vect::splat(Real::NAN),
            particle_impulses: vec![],
        });
    run(&mut app, 2);

    let context = context_entity(&mut app);
    let events = &app
        .world()
        .resource::<Received<PhysicsQuarantineEvent>>()
        .messages;
    assert_eq!(
        events,
        &[PhysicsQuarantineEvent {
            context,
            bodies: vec![],
            colliders: vec![],
            soft_bodies: vec![body],
        }]
    );
    assert!(app.world().get::<SoftBodyDisabled>(body).is_some());
    assert!(!rigid_body_set(&mut app)
        .soft_body(body)
        .unwrap()
        .is_enabled());

    // Fix the particles, then remove the component to release the soft body.
    app.world_mut()
        .run_system_once(move |mut context: WriteRapierContext| {
            let mut context = context.single_mut().unwrap();
            let sb = context.soft_body_mut(body).unwrap();
            for i in 0..sb.num_particles() {
                let rest = sb.particles()[i].rest_position();
                sb.set_particle_position(i, rest + v(0.0, 3.0));
            }
        })
        .unwrap();
    app.world_mut()
        .entity_mut(body)
        .remove::<SoftBodyDisabled>();
    run(&mut app, 2);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!(sb.is_enabled());
    assert!(sb.particle_positions().all(|p| p.is_finite()));
}

#[test]
fn particle_attachment_holds_the_soft_body() {
    let mut app = test_app();
    let anchor = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), RigidBody::Fixed))
        .id();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            SoftBody::rope(v(0.0, 0.0), v(2.0, 0.0), 5).map(|b| b.can_sleep(false)),
            SoftBodyAttachments(vec![SoftBodyAttachment {
                particle: 0,
                body: anchor,
            }]),
        ))
        .id();
    run(&mut app, 120);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert_eq!(sb.particle_attachments().len(), 1);
    assert!((sb.particle_position(0) - v(0.0, 3.0)).length() < 0.1);

    app.world_mut()
        .entity_mut(body)
        .remove::<SoftBodyAttachments>();
    run(&mut app, 60);
    let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
    assert!(sb.particle_attachments().is_empty());
    assert!(sb.particle_position(0).y < 2.0);
}

#[test]
fn tear_spawns_piece_entities_and_sends_messages() {
    let mut app = test_app();
    collect::<SoftBodyTearEvent>(&mut app);
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            pinned_rope(),
            Name::new("rope"),
        ))
        .id();
    app.update();

    let context = context_entity(&mut app);
    let torn = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        sb.edges()
            .iter()
            .position(|e| e.vertices == [4, 5] && e.kind == SoftBodyEdgeKind::Structural)
            .unwrap() as u32
    };
    let result = tear(&mut app, body, torn);
    assert_eq!(result.raw.pieces.len(), 2);
    assert_eq!(result.pieces.len(), 2);
    assert_eq!(result.pieces[0], body);
    // The piece's entity is known and mapped right away.
    let set = rigid_body_set(&mut app);
    let piece_handle = set.entity2soft_body()[&result.pieces[1]];
    assert_eq!(piece_handle, result.raw.pieces[1].soft_body);
    assert_eq!(set.soft_body_entity(piece_handle), Some(result.pieces[1]));
    run(&mut app, 60);

    let events = &app
        .world()
        .resource::<Received<SoftBodyTearEvent>>()
        .messages;
    assert_eq!(events.len(), 1);
    let event = &events[0];
    assert_eq!(event.soft_body, body);
    assert_eq!(event.context, context);
    assert_eq!(event.pieces.len(), 2);
    assert_eq!(event.pieces[0].soft_body, body);
    let piece = event.pieces[1].soft_body;
    assert_ne!(piece, body);
    assert_eq!(piece, result.pieces[1]);

    // The piece entity is a soft body entity, cloned from the torn one.
    let world = app.world();
    assert!(world.get::<RapierSoftBodyHandle>(piece).is_some());
    assert!(world.get::<SoftBody>(piece).is_some());
    assert_eq!(world.get::<Name>(piece).unwrap().as_str(), "rope");
    assert_eq!(
        world.get::<RapierContextEntityLink>(piece),
        Some(&RapierContextEntityLink(context))
    );
    let piece_state = *world.get::<SoftBodyState>(piece).unwrap();
    let body_state = *world.get::<SoftBodyState>(body).unwrap();
    assert_eq!(piece_state.num_particles, 5);
    // The loose piece fell away from the pinned one.
    assert!(piece_state.center_of_mass.y < body_state.center_of_mass.y - 1.0);

    let set = rigid_body_set(&mut app);
    assert_eq!(set.soft_bodies.len(), 2);
    let handle = set.entity2soft_body()[&piece];
    assert_eq!(set.soft_body_entity(handle), Some(piece));
    let proxy = set.soft_bodies[handle].root_body();
    assert_eq!(set.rigid_body_entity(proxy), Some(piece));

    // Despawning the piece removes it.
    app.world_mut().despawn(piece);
    app.update();
    assert_eq!(rigid_body_set(&mut app).soft_bodies.len(), 1);
}

#[test]
fn tear_by_force_during_the_step_sends_messages() {
    let mut app = test_app();
    collect::<SoftBodyTearEvent>(&mut app);
    // A vertical rope whose middle edge bears three particles, past its tear force.
    let positions = (0..7).map(|i| v(0.0, 3.0 - i as Real)).collect();
    let edges = (0..6).map(|i| [i, i + 1]).collect();
    let builder = SoftBodyBuilder::new(positions)
        .edges(edges)
        .pinned_particles([0])
        .particle_mass(1.0)
        .softness(rapier::dynamics::SpringCoefficients::new(120.0, 1.0))
        .no_surface_collider()
        .can_sleep(false)
        .edge_tear_resistance([(0, 4.0), (1, 4.0), (2, 4.0)])
        .tear_force(24.0);
    let body = app
        .world_mut()
        .spawn((Transform::default(), SoftBody::new(builder)))
        .id();
    run(&mut app, 60);

    let events = &app
        .world()
        .resource::<Received<SoftBodyTearEvent>>()
        .messages;
    assert_eq!(events.len(), 1, "{events:?}");
    assert_eq!(events[0].soft_body, body);
    assert_eq!(events[0].raw.torn_edges, vec![[3, 4]]);
    let piece = events[0].pieces[1].soft_body;
    assert!(app.world().get::<RapierSoftBodyHandle>(piece).is_some());
    assert_eq!(rigid_body_set(&mut app).soft_bodies.len(), 2);
}

#[test]
fn cluster_entity_gets_a_rigid_body_and_joints() {
    let mut app = test_app();
    let anchor = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), RigidBody::Fixed))
        .id();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box().map(|b| b.can_sleep(false)),
        ))
        .id();
    let num_particles = soft_box().builder.positions.len() as u32;
    #[cfg(feature = "dim3")]
    let joint = SphericalJointBuilder::new();
    #[cfg(feature = "dim2")]
    let joint = RevoluteJointBuilder::new();
    let cluster = app
        .world_mut()
        .spawn((
            Transform::default(),
            SoftBodyCluster::new(body, 0..num_particles),
            ImpulseJoint::new(anchor, joint),
        ))
        .id();
    run(&mut app, 2);

    let handle = app
        .world()
        .get::<RapierRigidBodyHandle>(cluster)
        .expect("the cluster has no rigid-body")
        .0;
    assert!(app
        .world()
        .get::<RapierImpulseJointHandle>(cluster)
        .is_some());
    let set = rigid_body_set(&mut app);
    assert!(set.bodies[handle].is_soft_frame());
    assert_eq!(set.entity2body()[&cluster], handle);
    assert_eq!(set.rigid_body_entity(handle), Some(cluster));
    // The soft body entity is mapped to its whole-body proxy.
    let whole = set.soft_body(body).unwrap().root_body();
    assert_eq!(set.entity2body()[&body], whole);
    assert_eq!(set.soft_body_whole_proxy(body), Some(whole));
    assert_eq!(
        set.soft_body_cluster_index(body),
        Some((set.entity2soft_body()[&body], 0))
    );
    let cluster_index = set.soft_body_cluster_index(cluster).unwrap();
    assert_eq!(
        set.soft_bodies[cluster_index.0].cluster_proxy(cluster_index.1),
        Some(handle)
    );
    #[cfg(feature = "to-bevy-mesh")]
    assert!(app.world().get::<Visibility>(cluster).is_some());

    // The joint holds the soft body where it would otherwise have fallen.
    run(&mut app, 120);
    let com = rigid_body_set(&mut app)
        .soft_body_center_of_mass(body)
        .unwrap();
    assert!((com - v(0.0, 3.0)).length() < 0.5, "{com}");
    // The cluster's pose is written back to its transform.
    let transform = *app.world().get::<Transform>(cluster).unwrap();
    assert!((transform.translation.y - 3.0).abs() < 0.5, "{transform:?}");

    // Despawning the cluster removes its proxy and the joint.
    app.world_mut().despawn(cluster);
    run(&mut app, 120);
    let set = rigid_body_set(&mut app);
    assert!(!set.bodies.contains(handle));
    assert!(set.soft_body_center_of_mass(body).unwrap().y < 0.0);
}

#[test]
fn deformable_collider_follows_its_soft_body() {
    #[cfg(feature = "dim2")]
    use rapier::parry::shape::PolylineFlags;
    #[cfg(feature = "dim3")]
    use rapier::parry::shape::TriMeshFlags;

    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box().map(|b| b.no_surface_collider()),
        ))
        .id();
    app.update();
    // A collision mesh made of the soft body's surface.
    let (vertices, surface) = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        (
            sb.particle_positions().collect::<Vec<_>>(),
            sb.boundary().to_vec(),
        )
    };
    #[cfg(feature = "dim3")]
    let collider =
        Collider::trimesh_with_flags(vertices, surface, TriMeshFlags::DEFORMABLE).unwrap();
    #[cfg(feature = "dim2")]
    let collider =
        Collider::polyline_with_flags(vertices, Some(surface), PolylineFlags::DEFORMABLE);
    let deformable = app
        .world_mut()
        .spawn((
            Transform::default(),
            collider,
            DeformableCollider::new(body, SoftMeshBinding::direct_by_position(1.0e-3)),
        ))
        .id();
    app.update();
    assert!(app
        .world()
        .get::<DeformableColliderError>(deformable)
        .is_none());
    let handle = app
        .world()
        .get::<RapierColliderHandle>(deformable)
        .expect("the deformable collider wasn't created")
        .0;
    let y0 = colliders(&mut app).colliders[handle].compute_aabb().mins.y;
    run(&mut app, 30);
    let co = &colliders(&mut app).colliders[handle];
    assert!(co.deformable_mesh_ref().is_some());
    assert!(co.compute_aabb().mins.y < y0 - 0.5);
    assert_eq!(
        colliders(&mut app).collider_entity(handle),
        Some(deformable)
    );
}

#[cfg(feature = "to-bevy-mesh")]
#[test]
fn mesh_sync_updates_vertex_positions() {
    #[cfg(feature = "dim2")]
    use bevy::mesh::Mesh2d as MeshComponent;
    #[cfg(feature = "dim3")]
    use bevy::mesh::Mesh3d as MeshComponent;
    use bevy::mesh::{Mesh, VertexAttributeValues};

    let mut app = test_app_with(|app| {
        app.add_plugins(AssetPlugin::default()).init_asset::<Mesh>();
    });
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(1.0, 3.0, 0.0),
            soft_box(),
            SoftBodyMeshSync::default(),
        ))
        .id();

    let check = |app: &mut App| {
        let context = context_entity(app);
        let mesh_handle = app.world().get::<MeshComponent>(body).unwrap().0.clone();
        let transform = *app.world().get::<Transform>(body).unwrap();
        let meshes = app.world().resource::<Assets<Mesh>>();
        let mesh = meshes.get(&mesh_handle).unwrap();
        let Some(VertexAttributeValues::Float32x3(positions)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            panic!("no positions");
        };
        let set = app.world().get::<RapierRigidBodySet>(context).unwrap();
        let particles: Vec<Vect> = set.soft_body_particle_positions(body).unwrap().collect();
        assert_eq!(positions.len(), particles.len());
        for (p, particle) in positions.iter().zip(particles.iter()) {
            let world = transform.transform_point(Vec3::from_array(*p));
            #[cfg(feature = "dim2")]
            let world = world.truncate();
            assert!(
                (world - *particle).length() < 1.0e-4,
                "{world} != {particle}"
            );
        }
        #[cfg(feature = "dim3")]
        assert!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_some());
    };

    run(&mut app, 2);
    check(&mut app);
    run(&mut app, 30);
    check(&mut app);
    assert!(
        app.world()
            .get::<SoftBodyState>(body)
            .unwrap()
            .center_of_mass
            .y
            < 2.9
    );
}

#[cfg(all(feature = "to-bevy-mesh", feature = "dim3"))]
#[test]
fn mesh_sync_ignores_deformable_colliders() {
    use bevy::mesh::{Mesh, Mesh3d};
    use rapier::parry::shape::TriMeshFlags;

    let mut app = test_app_with(|app| {
        app.add_plugins(AssetPlugin::default()).init_asset::<Mesh>();
    });
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box().map(|b| b.gravity_scale(0.0)),
        ))
        .id();
    app.update();
    // A deformable collider made of two triangles of the soft body's surface.
    let (vertices, boundary) = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        (
            sb.particle_positions().collect::<Vec<_>>(),
            sb.boundary().to_vec(),
        )
    };
    let collider =
        Collider::trimesh_with_flags(vertices, boundary[..2].to_vec(), TriMeshFlags::DEFORMABLE)
            .unwrap();
    app.world_mut().spawn((
        Transform::default(),
        collider,
        DeformableCollider::new(body, SoftMeshBinding::direct_by_position(1.0e-3)),
    ));
    app.update();
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyMeshSync::default());
    run(&mut app, 2);

    let mesh_handle = app.world().get::<Mesh3d>(body).unwrap().0.clone();
    let mesh = app
        .world()
        .resource::<Assets<Mesh>>()
        .get(&mesh_handle)
        .unwrap();
    assert_eq!(mesh.indices().unwrap().len(), boundary.len() * 3);
}

#[test]
fn soft_contacts_and_hook_views_are_exposed() {
    let mut app = test_app();
    let a = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 0.5, 0.0), soft_box()))
        .id();
    let b = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 2.5, 0.0), soft_box()))
        .id();
    app.world_mut()
        .spawn((Transform::from_xyz(0.0, -0.5, 0.0), ground()));
    run(&mut app, 120);

    let context = context_entity(&mut app);
    let world = app.world();
    let simulation = world.get::<RapierContextSimulation>(context).unwrap();
    let colliders = world.get::<RapierContextColliders>(context).unwrap();
    let set = world.get::<RapierRigidBodySet>(context).unwrap();
    let pair = simulation
        .contact_pair(colliders, set, a, b)
        .expect("the soft bodies don't touch");
    assert!(pair.is_soft());
    assert!(pair.soft_contacts().is_some());
    assert!(matches!(pair.contacts(), PairContacts::Soft { .. }));
    assert!(pair.soft_touching().is_some());
}

#[cfg(feature = "serde-serialize")]
#[test]
fn soft_body_settings_wrappers_serde_round_trip() {
    use crate::reflect::{SoftBodiesSettingsWrapper, SoftRecoverySettingsWrapper};

    let mut settings = SoftBodiesSettings {
        max_extra_substeps: 7,
        ..Default::default()
    };
    settings.recovery.recovery_pace = 0.25;
    settings.recovery.overlap_patch_constraints = SoftPatchConstraints::StandDown;
    let json = serde_json::to_string(&SoftBodiesSettingsWrapper(settings)).unwrap();
    let back: SoftBodiesSettingsWrapper = serde_json::from_str(&json).unwrap();
    assert_eq!(back.0, settings);

    let json = serde_json::to_string(&SoftRecoverySettingsWrapper(settings.recovery)).unwrap();
    let back: SoftRecoverySettingsWrapper = serde_json::from_str(&json).unwrap();
    assert_eq!(back.0, settings.recovery);
}

#[test]
fn soft_body_settings_are_reflected() {
    use crate::reflect::IntegrationParametersWrapper;
    use bevy::reflect::{GetPath, Reflect};

    let mut params =
        IntegrationParametersWrapper(rapier::dynamics::IntegrationParameters::default());
    *params
        .path_mut::<Real>("soft_bodies.recovery.recovery_pace")
        .unwrap() = 0.125;
    *params
        .path_mut::<usize>("soft_bodies.max_extra_substeps")
        .unwrap() = 2;
    assert_eq!(params.0.soft_bodies.recovery.recovery_pace, 0.125);
    assert_eq!(params.0.soft_bodies.max_extra_substeps, 2);
    assert!(params.as_reflect().reflect_ref().as_struct().is_ok());

    let mut material = SoftBodyMaterial::default();
    *material.path_mut::<Real>("0.young_modulus").unwrap() = 5.0;
    assert_eq!(material.young_modulus, 5.0);
}

#[test]
fn contact_modification_hook_sees_soft_pairs() {
    use bevy::ecs::system::SystemParam;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    #[derive(Resource, Default)]
    struct SoftCalls(Arc<AtomicUsize>);

    #[derive(SystemParam)]
    struct SoftHooks<'w> {
        calls: Res<'w, SoftCalls>,
    }

    impl BevyPhysicsHooks for SoftHooks<'_> {
        fn modify_solver_contacts(&self, mut context: ContactModificationContextView) {
            if context.is_soft() {
                assert!(context.soft().is_some());
                assert!(context.manifold().is_none());
                // Dropping every candidate lets the soft bodies pass through each other.
                context.soft_mut().unwrap().disable_all();
                self.calls.0.fetch_add(1, Ordering::SeqCst);
            }
        }
    }

    let mut app = App::new();
    app.add_plugins((
        TransformPlugin,
        TimePlugin,
        RapierPhysicsPlugin::<SoftHooks>::default(),
    ))
    .init_resource::<SoftCalls>()
    .insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    })
    .insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1.0 / 60.0),
    ));
    app.finish();
    app.update();
    app.world_mut()
        .spawn((Transform::from_xyz(0.0, -0.5, 0.0), ground()));
    app.world_mut().spawn((
        Transform::from_xyz(0.0, 0.5, 0.0),
        soft_box(),
        ActiveHooks::MODIFY_SOLVER_CONTACTS,
    ));
    let top = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 2.5, 0.0),
            soft_box(),
            ActiveHooks::MODIFY_SOLVER_CONTACTS,
        ))
        .id();
    run(&mut app, 120);

    assert!(app.world().resource::<SoftCalls>().0.load(Ordering::SeqCst) > 0);
    // Without soft contacts, the top body sank into the bottom one.
    let com = app
        .world()
        .get::<SoftBodyState>(top)
        .unwrap()
        .center_of_mass;
    assert!(com.y < 1.5, "{com}");
}

#[test]
fn tear_through_a_cluster_spawns_a_cluster_entity() {
    let mut app = test_app();
    collect::<SoftBodyTearEvent>(&mut app);
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), pinned_rope()))
        .id();
    let cluster = app
        .world_mut()
        .spawn((Transform::default(), SoftBodyCluster::new(body, 0..9)))
        .id();
    run(&mut app, 2);

    let torn = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        sb.edges()
            .iter()
            .position(|e| e.vertices == [4, 5] && e.kind == SoftBodyEdgeKind::Structural)
            .unwrap() as u32
    };
    tear(&mut app, body, torn);
    run(&mut app, 2);

    let events = app
        .world()
        .resource::<Received<SoftBodyTearEvent>>()
        .messages
        .clone();
    assert_eq!(events.len(), 1);
    let splits = &events[0].cluster_splits;
    assert!(!splits.is_empty(), "{events:?}");
    // Cluster 0 is the whole-body cluster, cluster 1 the one of the cluster entity.
    let fresh = splits
        .iter()
        .find(|split| split.source_cluster == 1 && !split.keeps_proxy)
        .expect("no fresh proxy");
    let fresh_entity = fresh.proxy.unwrap();
    assert_ne!(fresh_entity, cluster);
    let fresh_cluster = app.world().get::<SoftBodyCluster>(fresh_entity).unwrap();
    assert_eq!(fresh_cluster.soft_body, fresh.soft_body);
    let handle = app
        .world()
        .get::<RapierRigidBodyHandle>(fresh_entity)
        .unwrap()
        .0;
    let set = rigid_body_set(&mut app);
    assert_eq!(set.rigid_body_entity(handle), Some(fresh_entity));
    assert!(set.bodies[handle].is_soft_frame());
    // The fresh whole-body proxy of the new piece maps to the piece entity.
    let whole = splits
        .iter()
        .find(|split| split.source_cluster == 0 && !split.keeps_proxy)
        .unwrap();
    assert_eq!(whole.proxy, Some(whole.soft_body));
    assert_eq!(events[0].pieces[1].soft_body, whole.soft_body);
    // The kept proxy still maps to the original cluster entity.
    let kept = splits
        .iter()
        .find(|split| split.source_cluster == 1 && split.keeps_proxy)
        .unwrap();
    assert_eq!(kept.proxy, Some(cluster));
    assert_eq!(
        app.world()
            .get::<SoftBodyCluster>(cluster)
            .unwrap()
            .soft_body,
        kept.soft_body
    );
}

#[test]
fn split_clusters_inherit_the_cluster_components() {
    let mut app = test_app();
    collect::<SoftBodyTearEvent>(&mut app);
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), pinned_rope()))
        .id();
    // The cluster holds the whole rope where it was created (its rest centroid is at `x = 2`).
    let material = SoftBodyClusterMaterial {
        tear_resistance: 2.0,
        ..default()
    };
    let shape_matching = SoftBodyClusterShapeMatching {
        target: Some(Transform::from_xyz(0.0, 3.0, 0.0)),
    };
    let target = SoftBodyClusterKinematicTarget(Transform::from_xyz(2.0, 3.0, 0.0));
    let cluster = app
        .world_mut()
        .spawn((
            Transform::default(),
            SoftBodyCluster::new(body, 0..9),
            target,
            shape_matching,
            material,
        ))
        .id();
    run(&mut app, 5);
    // An edge between two pinned particles can't tear: release the cluster first (its target is
    // kept, but no longer applied).
    app.world_mut()
        .entity_mut(cluster)
        .remove::<SoftBodyClusterPinned>();
    app.update();

    let torn = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        sb.edges()
            .iter()
            .position(|e| e.vertices == [4, 5] && e.kind == SoftBodyEdgeKind::Structural)
            .unwrap() as u32
    };
    tear(&mut app, body, torn);
    run(&mut app, 30);

    let events = app
        .world()
        .resource::<Received<SoftBodyTearEvent>>()
        .messages
        .clone();
    let fresh = events[0]
        .cluster_splits
        .iter()
        .find(|split| split.source_cluster == 1 && !split.keeps_proxy)
        .expect("no fresh cluster");
    let fresh_entity = fresh.proxy.unwrap();
    assert_ne!(fresh_entity, cluster);
    let world = app.world();
    assert!(world.get::<SoftBodyClusterPinned>(fresh_entity).is_some());
    assert_eq!(
        world.get::<SoftBodyClusterShapeMatching>(fresh_entity),
        Some(&shape_matching)
    );
    assert_eq!(
        world.get::<SoftBodyClusterMaterial>(fresh_entity),
        Some(&material)
    );
    // The target of the fresh cluster was shifted to its own rest centroid.
    let fresh_target = world
        .get::<SoftBodyClusterKinematicTarget>(fresh_entity)
        .unwrap();
    assert!(fresh_target.0.translation.x > 2.5, "{fresh_target:?}");

    // The inherited target places the particles of the fresh cluster where the original target
    // placed them.
    let set = rigid_body_set(&mut app);
    let (handle, index) = set.soft_body_cluster_index(fresh_entity).unwrap();
    let sb = &set.soft_bodies[handle];
    let fresh_cluster = sb.cluster(index).unwrap();
    assert!(fresh_cluster.shape_matching_enabled());
    for &particle in fresh_cluster.particles() {
        let p = &sb.particles()[particle as usize];
        assert!(p.is_pinned());
        // The rest positions are relative to the rest center of mass of the rope, at `(2, 3)`.
        let expected = p.rest_position() + v(2.0, 3.0);
        assert!(
            (p.position() - expected).length() < 1.0e-3,
            "{} != {expected}",
            p.position(),
        );
    }
}

#[test]
fn zero_dt_frame_does_not_disturb_a_welded_cluster() {
    // Spawn everything before the first frame, whose delta time is zero.
    let mut app = App::new();
    app.add_plugins((
        TransformPlugin,
        TimePlugin,
        RapierPhysicsPlugin::<NoUserData>::default(),
    ));
    app.insert_resource(TimestepMode::default());
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1.0 / 60.0),
    ));
    app.finish();

    app.world_mut()
        .spawn((Transform::from_xyz(0.0, -0.5, 0.0), ground()));
    #[cfg(feature = "dim3")]
    let jelly = soft_box();
    #[cfg(feature = "dim2")]
    let jelly = SoftBody::grid(Vect::splat(0.6), 5, 5);
    let jelly = jelly.map(|b| {
        b.cell_model(SoftBodyCellModel::Corotational)
            .particle_mass(0.08)
            .particle_radius(0.05)
            .can_sleep(false)
    });
    let top_y = jelly
        .builder
        .particle_positions()
        .iter()
        .map(|p| p.y)
        .fold(Real::MIN, Real::max);
    let top_edge: Vec<u32> = (0..jelly.builder.particle_positions().len() as u32)
        .filter(|i| jelly.builder.particle_positions()[*i as usize].y > top_y - 1.0e-3)
        .collect();
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 1.0, 0.0), jelly))
        .id();
    #[cfg(feature = "dim3")]
    let plate_shape = Collider::cuboid(0.7, 0.06, 0.7);
    #[cfg(feature = "dim2")]
    let plate_shape = Collider::cuboid(0.7, 0.06);
    let plate = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.0 + top_y + 0.2, 0.0),
            RigidBody::Dynamic,
            plate_shape,
        ))
        .id();
    app.world_mut().spawn((
        Transform::default(),
        SoftBodyCluster::new(body, top_edge),
        ImpulseJoint::new(plate, FixedJointBuilder::new().local_anchor1(v(0.0, -0.2))),
    ));

    // The first frame has a zero delta time: it must not inject any velocity (a zero-length step
    // used to launch the plate and the jelly's particles).
    let max_speeds = |app: &mut App| {
        let set = rigid_body_set(app);
        let particles = set
            .soft_body_particle_velocities(body)
            .unwrap()
            .map(|v| v.length())
            .fold(0.0, Real::max);
        let plate = &set.bodies[set.entity2body()[&plate]];
        (particles, plate.linvel().length())
    };
    run(&mut app, 1);
    assert_eq!(max_speeds(&mut app), (0.0, 0.0));
    run(&mut app, 1);
    let (particles, plate_speed) = max_speeds(&mut app);
    assert!(
        particles < 0.5 && plate_speed < 0.5,
        "{particles} {plate_speed}"
    );

    run(&mut app, 60);
    let set = rigid_body_set(&mut app);
    assert!(set
        .soft_body_particle_positions(body)
        .unwrap()
        .all(|p| p.is_finite()));
    assert!(set.bodies[set.entity2body()[&plate]]
        .translation()
        .is_finite());
    let plate_transform = app.world().get::<Transform>(plate).unwrap();
    assert!(plate_transform.translation.is_finite());
}

#[test]
fn soft_body_entity_is_a_joint_target() {
    let mut app = test_app();
    let anchor = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), RigidBody::Fixed))
        .id();
    #[cfg(feature = "dim3")]
    let joint = SphericalJointBuilder::new();
    #[cfg(feature = "dim2")]
    let joint = RevoluteJointBuilder::new();
    // A joint on the soft body entity itself, and a ball hanging from the soft body entity.
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            soft_box().map(|b| b.can_sleep(false)),
            ImpulseJoint::new(anchor, joint),
        ))
        .id();
    let ball = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 1.5, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.2),
            ImpulseJoint::new(body, RopeJointBuilder::new(1.5)),
        ))
        .id();
    run(&mut app, 2);
    assert!(app.world().get::<RapierImpulseJointHandle>(body).is_some());
    assert!(app.world().get::<RapierImpulseJointHandle>(ball).is_some());

    // The soft body is held where it would otherwise have fallen, and holds the ball.
    run(&mut app, 120);
    let set = rigid_body_set(&mut app);
    let com = set.soft_body_center_of_mass(body).unwrap();
    assert!((com - v(0.0, 3.0)).length() < 0.5, "{com}");
    let ball_rb = &set.bodies[set.entity2body()[&ball]];
    assert!(ball_rb.translation().y > 0.5, "{}", ball_rb.translation());

    // Despawning the soft body unmaps it and removes its joints.
    app.world_mut().despawn(body);
    run(&mut app, 2);
    let set = rigid_body_set(&mut app);
    assert!(!set.entity2body().contains_key(&body));
    let context = context_entity(&mut app);
    let joints = app.world().get::<RapierContextJoints>(context).unwrap();
    assert!(joints.impulse_joints.is_empty());
}

#[test]
fn cluster_components_are_applied_and_reset() {
    let mut app = test_app();
    let body = app
        .world_mut()
        .spawn((Transform::from_xyz(0.0, 3.0, 0.0), pinned_rope()))
        .id();
    let cluster = app
        .world_mut()
        .spawn(SoftBodyCluster::new(body, [7, 8]))
        .id();
    run(&mut app, 2);
    let (handle, index) = rigid_body_set(&mut app)
        .soft_body_cluster_index(cluster)
        .unwrap();
    assert_eq!(index, 1);

    // Pin the cluster and move it with a kinematic target.
    let target = Transform::from_xyz(5.0, 3.0, 0.0);
    app.world_mut()
        .entity_mut(cluster)
        .insert(SoftBodyClusterKinematicTarget(target));
    assert!(app.world().get::<SoftBodyClusterPinned>(cluster).is_some());
    run(&mut app, 3);
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(sb.particles()[7].is_pinned() && sb.particles()[8].is_pinned());
    let center = (sb.particle_position(7) + sb.particle_position(8)) / 2.0;
    assert!((center - v(5.0, 3.0)).length() < 1.0e-3, "{center}");

    // Changing the soft body's pinned particles keeps the cluster pinned.
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyPinnedParticles(vec![0, 8]));
    app.update();
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(sb.particles()[7].is_pinned() && sb.particles()[8].is_pinned());

    // Unpinning the cluster keeps the particles pinned by the soft body.
    app.world_mut()
        .entity_mut(cluster)
        .remove::<(SoftBodyClusterKinematicTarget, SoftBodyClusterPinned)>();
    app.update();
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(!sb.particles()[7].is_pinned());
    assert!(sb.particles()[8].is_pinned());
    assert!(sb.particles()[0].is_pinned());

    // Shape matching and regional material.
    app.world_mut().entity_mut(cluster).insert((
        SoftBodyClusterShapeMatching {
            target: Some(Transform::from_xyz(1.0, 2.0, 0.0)),
        },
        SoftBodyClusterMaterial {
            tear_resistance: 3.0,
            ..default()
        },
    ));
    app.update();
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(sb.cluster(index).unwrap().shape_matching_enabled());
    let edge = |sb: &RapierSoftBody| {
        sb.edges()
            .iter()
            .find(|e| e.vertices == [7, 8])
            .unwrap()
            .tear_resistance
    };
    assert_eq!(edge(sb), 3.0);

    app.world_mut()
        .entity_mut(cluster)
        .remove::<(SoftBodyClusterShapeMatching, SoftBodyClusterMaterial)>();
    app.update();
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(!sb.cluster(index).unwrap().shape_matching_enabled());
    assert_eq!(edge(sb), 1.0);

    // The components also drive the whole-body cluster of a soft body entity.
    app.world_mut()
        .entity_mut(body)
        .insert(SoftBodyClusterPinned);
    app.update();
    let sb = &rigid_body_set(&mut app).soft_bodies[handle];
    assert!(sb.particles().iter().all(|p| p.is_pinned()));
}

#[test]
fn tear_remaps_the_particle_indices_of_the_components() {
    let mut app = test_app();
    collect::<SoftBodyTearEvent>(&mut app);
    let anchor = app
        .world_mut()
        .spawn((Transform::from_xyz(4.0, 3.0, 0.0), RigidBody::Fixed))
        .id();
    // A rope pinned at its first particle, and held by its last one.
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            pinned_rope(),
            SoftBodyPinnedParticles(vec![0, 7]),
            SoftBodyKinematicTargets(vec![(7, v(3.5, 3.0))]),
            SoftBodyAttachments(vec![SoftBodyAttachment {
                particle: 8,
                body: anchor,
            }]),
            SoftBodyExternalForce {
                force: v(0.0, 1.0),
                particle_forces: vec![(6, v(1.0, 0.0))],
            },
        ))
        .id();
    run(&mut app, 2);
    let (torn, held) = {
        let sb = rigid_body_set(&mut app).soft_body(body).unwrap();
        let torn = sb
            .edges()
            .iter()
            .position(|e| e.vertices == [5, 6] && e.kind == SoftBodyEdgeKind::Structural)
            .unwrap() as u32;
        (torn, sb.particle_position(7))
    };
    // Impulses written between the step and the writeback are only applied on the next frame,
    // after the tear remapped them.
    #[derive(Resource)]
    struct PendingImpulse(Option<Entity>);
    app.insert_resource(PendingImpulse(Some(body)));
    app.add_systems(
        PostUpdate,
        (|mut pending: ResMut<PendingImpulse>, mut commands: Commands| {
            if let Some(entity) = pending.0.take() {
                commands.entity(entity).insert(SoftBodyExternalImpulse {
                    velocity_change: v(0.0, 1.0),
                    particle_impulses: vec![(6, v(1.0, 0.0)), (1, v(2.0, 0.0))],
                });
            }
        })
        .after(PhysicsSet::StepSimulation)
        .before(PhysicsSet::Writeback),
    );

    let result = tear(&mut app, body, torn);
    let raw = result.raw;
    assert_eq!(raw.pieces.len(), 2);
    let dest = |particle: u32| raw.particle_destination(particle).unwrap();
    // The particles 6 to 8 went to the other piece.
    let (piece_handle, i7) = dest(7);
    let (_, i6) = dest(6);
    let (_, i8) = dest(8);
    let (_, i0) = dest(0);
    let (_, i1) = dest(1);
    assert_ne!(piece_handle, raw.soft_body);
    app.update();

    let piece = result.pieces[1];
    let world = app.world();
    let impulse = world.get::<SoftBodyExternalImpulse>(piece).unwrap();
    assert_eq!(impulse.velocity_change, v(0.0, 1.0));
    assert_eq!(impulse.particle_impulses, [(i6, v(1.0, 0.0))]);
    let impulse = world.get::<SoftBodyExternalImpulse>(body).unwrap();
    assert_eq!(impulse.particle_impulses, [(i1, v(2.0, 0.0))]);
    // The pinned particles of the builders were remapped too.
    assert_eq!(world.get::<SoftBody>(body).unwrap().builder.pinned, [i0]);
    assert!(world
        .get::<SoftBody>(piece)
        .unwrap()
        .builder
        .pinned
        .is_empty());
    app.update();

    let events = &app
        .world()
        .resource::<Received<SoftBodyTearEvent>>()
        .messages;
    let piece = events[0].pieces[1].soft_body;
    let world = app.world();
    assert_eq!(world.get::<SoftBodyPinnedParticles>(body).unwrap().0, [0]);
    assert!(world
        .get::<SoftBodyKinematicTargets>(body)
        .unwrap()
        .0
        .is_empty());
    assert!(world.get::<SoftBodyAttachments>(body).unwrap().0.is_empty());
    assert_eq!(world.get::<SoftBodyPinnedParticles>(piece).unwrap().0, [i7]);
    assert_eq!(
        world.get::<SoftBodyKinematicTargets>(piece).unwrap().0,
        [(i7, v(3.5, 3.0))]
    );
    assert_eq!(
        world.get::<SoftBodyAttachments>(piece).unwrap().0,
        [SoftBodyAttachment {
            particle: i8,
            body: anchor
        }]
    );
    let force = world.get::<SoftBodyExternalForce>(piece).unwrap();
    assert_eq!(force.force, v(0.0, 1.0));
    assert_eq!(force.particle_forces, [(i6, v(1.0, 0.0))]);
    // The piece's whole-body proxy is mapped to its entity.
    let set = rigid_body_set(&mut app);
    assert_eq!(
        set.entity2body().get(&piece),
        set.soft_body_whole_proxy(piece).as_ref()
    );
    assert!(set.entity2body().contains_key(&piece));

    // The pinned particle stays pinned, even after the components are applied again.
    app.world_mut()
        .get_mut::<SoftBodyPinnedParticles>(piece)
        .unwrap()
        .set_changed();
    run(&mut app, 60);
    let sb = rigid_body_set(&mut app).soft_body(piece).unwrap();
    assert!(sb.particles()[i7 as usize].is_pinned());
    assert!((sb.particle_position(i7 as usize) - held).length() < 1.0e-4);
    assert_eq!(sb.particle_attachments().len(), 1);
    assert_eq!(sb.particle_attachments()[0].particle, i8);

    // Removing the pinned particles restores the (remapped) pins of the builder.
    for entity in [body, piece] {
        app.world_mut()
            .entity_mut(entity)
            .remove::<(SoftBodyPinnedParticles, SoftBodyKinematicTargets)>();
    }
    app.update();
    let set = rigid_body_set(&mut app);
    let pinned = |entity| -> Vec<usize> {
        let sb = set.soft_body(entity).unwrap();
        (0..sb.num_particles())
            .filter(|i| sb.particles()[*i].is_pinned())
            .collect()
    };
    assert_eq!(pinned(body), [i0 as usize]);
    assert!(pinned(piece).is_empty());
}
