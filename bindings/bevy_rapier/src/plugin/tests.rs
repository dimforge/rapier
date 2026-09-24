//! Tests of the simulation pipeline configuration, diagnostics and serialization.

use bevy::prelude::*;
use bevy::time::{TimePlugin, TimeUpdateStrategy};

use crate::plugin::context::DefaultRapierContext;
use crate::prelude::*;

/// An app running the physics at a fixed 60Hz timestep.
fn test_app(plugin: RapierPhysicsPlugin<NoUserData>) -> App {
    let mut app = App::new();
    app.add_plugins((TransformPlugin, TimePlugin, plugin));
    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1.0 / 60.0),
    ));
    app.finish();
    app
}

fn context_entity(app: &mut App) -> Entity {
    app.world_mut()
        .query_filtered::<Entity, With<DefaultRapierContext>>()
        .single(app.world())
        .unwrap()
}

fn configure(app: &mut App, f: impl FnOnce(&mut RapierConfiguration)) {
    let context = context_entity(app);
    let mut config = app
        .world_mut()
        .get_mut::<RapierConfiguration>(context)
        .unwrap();
    f(&mut config);
}

fn body_translation(app: &mut App, entity: Entity) -> Vect {
    let context = context_entity(app);
    let bodies = app.world().get::<RapierRigidBodySet>(context).unwrap();
    let handle = bodies.entity2body()[&entity];
    #[allow(clippy::useless_conversion)] // Needed in 3D but not 2D.
    bodies.bodies[handle].translation().into()
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

#[test]
fn quarantine_event_is_sent_for_nan_velocity() {
    let mut app = test_app(RapierPhysicsPlugin::default());
    collect::<PhysicsQuarantineEvent>(&mut app);
    app.update();

    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
            Velocity::zero(),
        ))
        .id();
    let healthy = app
        .world_mut()
        .spawn((
            Transform::from_xyz(5.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
        ))
        .id();
    for _ in 0..3 {
        app.update();
    }
    assert!(app
        .world()
        .resource::<Received<PhysicsQuarantineEvent>>()
        .messages
        .is_empty());

    app.world_mut().get_mut::<Velocity>(body).unwrap().linear = Vect::splat(crate::math::Real::NAN);
    app.update();

    let context = context_entity(&mut app);
    let received = &app
        .world()
        .resource::<Received<PhysicsQuarantineEvent>>()
        .messages;
    assert_eq!(
        received,
        &[PhysicsQuarantineEvent {
            context,
            bodies: vec![body],
            colliders: vec![],
            soft_bodies: vec![],
        }]
    );

    let bodies = app.world().get::<RapierRigidBodySet>(context).unwrap();
    assert!(!bodies.bodies[bodies.entity2body()[&body]].is_enabled());
    assert!(bodies.bodies[bodies.entity2body()[&healthy]].is_enabled());
    assert!(app.world().get::<RigidBodyDisabled>(body).is_some());
    assert!(app.world().get::<RigidBodyDisabled>(healthy).is_none());

    // Fixing the velocity and removing the component releases the body from the quarantine.
    *app.world_mut().get_mut::<Velocity>(body).unwrap() = Velocity::linear(Vect::X);
    app.world_mut()
        .entity_mut(body)
        .remove::<RigidBodyDisabled>();
    app.update();
    app.update();
    let bodies = app.world().get::<RapierRigidBodySet>(context).unwrap();
    let rb = &bodies.bodies[bodies.entity2body()[&body]];
    assert!(rb.is_enabled());
    assert!(rb.translation().x > 0.0 && rb.translation().is_finite());
    assert_eq!(
        app.world()
            .resource::<Received<PhysicsQuarantineEvent>>()
            .messages
            .len(),
        1
    );
}

#[test]
fn mass_read_backs_are_inserted_automatically() {
    let mut app = test_app(RapierPhysicsPlugin::default());
    app.update();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
            ColliderMassProperties::Mass(2.0),
        ))
        .id();
    app.update();
    app.update();

    let world = app.world();
    assert_eq!(world.get::<ReadMassProperties>(body).unwrap().mass, 2.0);
    assert_eq!(
        world.get::<ReadColliderMassProperties>(body).unwrap().mass,
        2.0
    );
    // The world-space mass properties are opt-in.
    assert!(world.get::<ReadWorldMassProperties>(body).is_none());
}

/// Spawns two overlapping dynamic bodies at `y = 3`, and a kinematic and a fixed body far away.
fn spawn_collision_only_scene(app: &mut App, sensors: bool) -> [Entity; 4] {
    let collider = |app: &mut App, entity: Entity| {
        if sensors {
            app.world_mut().entity_mut(entity).insert(Sensor);
        }
    };
    let body1 = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
            ActiveEvents::COLLISION_EVENTS,
        ))
        .id();
    collider(app, body1);
    let body2 = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.5, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
        ))
        .id();
    let kinematic = app
        .world_mut()
        .spawn((
            Transform::from_xyz(10.0, 0.0, 0.0),
            RigidBody::KinematicPositionBased,
            Collider::ball(0.5),
            ActiveEvents::COLLISION_EVENTS,
            ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_STATIC,
        ))
        .id();
    collider(app, kinematic);
    let fixed = app
        .world_mut()
        .spawn((
            Transform::from_xyz(-10.0, 0.0, 0.0),
            RigidBody::Fixed,
            Collider::ball(0.5),
        ))
        .id();
    [body1, body2, kinematic, fixed]
}

fn check_collision_only_mode(sensors: bool) {
    let mut app = test_app(RapierPhysicsPlugin::default());
    collect::<CollisionEvent>(&mut app);
    app.update();
    configure(&mut app, |config| {
        config.simulation_mode = SimulationMode::CollisionOnly;
    });
    let [body1, body2, kinematic, fixed] = spawn_collision_only_scene(&mut app, sensors);

    for _ in 0..10 {
        app.update();
    }

    // The overlapping bodies are neither pushed apart nor falling.
    approx::assert_relative_eq!(body_translation(&mut app, body1), Vect::Y * 3.0);
    approx::assert_relative_eq!(
        body_translation(&mut app, body2),
        Vect::X * 0.5 + Vect::Y * 3.0
    );
    assert_eq!(
        app.world().get::<Transform>(body1).unwrap().translation.y,
        3.0
    );

    let received = &app.world().resource::<Received<CollisionEvent>>().messages;
    assert_eq!(received.len(), 1);
    assert!(matches!(
        received[0],
        CollisionEvent::Started(a, b, _) if (a, b) == (body1, body2) || (a, b) == (body2, body1)
    ));

    // Moving a kinematic body through its transform still moves its rigid-body and collider.
    app.world_mut()
        .get_mut::<Transform>(kinematic)
        .unwrap()
        .translation = Vec3::new(0.0, 3.5, 0.0);
    app.update();
    approx::assert_relative_eq!(body_translation(&mut app, kinematic), Vect::Y * 3.5);
    assert_eq!(
        app.world()
            .resource::<Received<CollisionEvent>>()
            .messages
            .iter()
            .filter(|e| matches!(e, CollisionEvent::Started(..)))
            .count(),
        3
    );

    // The kinematic body also reports its contacts with fixed bodies.
    app.world_mut()
        .get_mut::<Transform>(kinematic)
        .unwrap()
        .translation = Vec3::new(-10.0, 0.5, 0.0);
    app.update();
    approx::assert_relative_eq!(body_translation(&mut app, fixed), Vect::X * -10.0);
    let received = &app.world().resource::<Received<CollisionEvent>>().messages;
    assert!(matches!(
        received.last(),
        Some(CollisionEvent::Started(a, b, _)) if (*a, *b) == (kinematic, fixed) || (*a, *b) == (fixed, kinematic)
    ));
}

#[test]
fn collision_only_mode_detects_intersections_without_dynamics() {
    check_collision_only_mode(true);
}

#[test]
fn collision_only_mode_detects_contacts_without_dynamics() {
    check_collision_only_mode(false);
}

fn num_steps(app: &mut App) -> usize {
    let context = context_entity(app);
    app.world()
        .get::<RapierContextSimulation>(context)
        .unwrap()
        .step_stats()
        .num_steps
}

#[test]
fn zero_dt_frames_do_not_step() {
    let mut app = test_app(RapierPhysicsPlugin::default());
    app.insert_resource(TimestepMode::default());
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 3.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
        ))
        .id();

    // The first frame has a zero delta time: nothing is stepped but the body is still created.
    app.update();
    assert_eq!(num_steps(&mut app), 0);
    assert_eq!(body_translation(&mut app, body).y, 3.0);

    app.update();
    assert_eq!(num_steps(&mut app), 1);
    assert!(body_translation(&mut app, body).y < 3.0);

    // Same with a zero fixed dt.
    app.insert_resource(TimestepMode::Fixed {
        dt: 0.0,
        substeps: 1,
    });
    let y = body_translation(&mut app, body).y;
    app.update();
    assert_eq!(num_steps(&mut app), 0);
    assert_eq!(body_translation(&mut app, body).y, y);
}

#[test]
fn diagnostics_are_measured_after_a_step() {
    let mut app = App::new();
    app.add_plugins((
        TransformPlugin,
        TimePlugin,
        RapierPhysicsPlugin::<NoUserData>::default(),
        RapierDiagnosticsPlugin::default(),
    ));
    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 2,
    });
    app.finish();
    app.update();

    app.world_mut().spawn((
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Fixed,
        Collider::ball(1.0),
    ));
    app.world_mut().spawn((
        Transform::from_xyz(0.0, 1.5, 0.0),
        RigidBody::Dynamic,
        Collider::ball(1.0),
    ));
    app.update();
    app.update();

    let store = app.world().resource::<bevy::diagnostic::DiagnosticsStore>();
    let value = |path| store.get(&path).unwrap().value();
    assert!(value(RapierDiagnosticsPlugin::STEP_TIME).unwrap() > 0.0);
    assert_eq!(value(RapierDiagnosticsPlugin::STEPS), Some(2.0));
    assert_eq!(value(RapierDiagnosticsPlugin::RIGID_BODIES), Some(2.0));
    assert_eq!(value(RapierDiagnosticsPlugin::COLLIDERS), Some(2.0));
    assert_eq!(value(RapierDiagnosticsPlugin::CONTACT_PAIRS), Some(1.0));
    assert_eq!(
        value(RapierDiagnosticsPlugin::ACTIVE_CONTACT_PAIRS),
        Some(1.0)
    );
    assert!(value(RapierDiagnosticsPlugin::CONTACT_CONSTRAINTS).unwrap() >= 1.0);
    assert!(value(RapierDiagnosticsPlugin::SOLVER_CONSTRAINTS).unwrap() >= 1.0);
    assert_eq!(value(RapierDiagnosticsPlugin::CCD_SUBSTEPS), Some(0.0));
    #[cfg(feature = "profiler")]
    assert!(value(RapierDiagnosticsPlugin::SOLVER_TIME).is_some());

    let context = context_entity(&mut app);
    let stats = app
        .world()
        .get::<RapierContextSimulation>(context)
        .unwrap()
        .step_stats();
    assert_eq!(stats.num_steps, 2);
    assert_eq!(stats.num_contact_pairs, 1);
    // Each step solves the contact manifold between the two balls.
    assert_eq!(stats.num_solver_constraints, 2);
}

#[test]
fn diagnostics_sum_every_context_and_measure_each_one() {
    let mut app = App::new();
    app.add_plugins((
        TransformPlugin,
        TimePlugin,
        RapierPhysicsPlugin::<NoUserData>::default(),
        RapierDiagnosticsPlugin::default().with_per_context_diagnostics(true),
    ));
    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });
    app.finish();
    app.update();

    let default_context = context_entity(&mut app);
    let other_context = app
        .world_mut()
        .spawn(RapierContextSimulation::default())
        .id();
    for (context, num_bodies) in [(default_context, 1), (other_context, 2)] {
        for i in 0..num_bodies {
            app.world_mut().spawn((
                Transform::from_xyz(i as f32 * 5.0, 0.0, 0.0),
                RigidBody::Dynamic,
                Collider::ball(1.0),
                RapierContextEntityLink(context),
            ));
        }
    }
    // The per-context diagnostics are registered on the first measurement.
    for _ in 0..3 {
        app.update();
    }

    let store = app.world().resource::<bevy::diagnostic::DiagnosticsStore>();
    let value = |path| store.get(&path).and_then(|d| d.value());
    let context_value =
        |path, context| value(RapierDiagnosticsPlugin::context_path(&path, context));
    assert_eq!(value(RapierDiagnosticsPlugin::RIGID_BODIES), Some(3.0));
    assert_eq!(value(RapierDiagnosticsPlugin::STEPS), Some(2.0));
    assert_eq!(
        context_value(RapierDiagnosticsPlugin::RIGID_BODIES, default_context),
        Some(1.0)
    );
    assert_eq!(
        context_value(RapierDiagnosticsPlugin::RIGID_BODIES, other_context),
        Some(2.0)
    );
    assert_eq!(
        context_value(RapierDiagnosticsPlugin::STEPS, other_context),
        Some(1.0)
    );
    assert_eq!(
        RapierDiagnosticsPlugin::context_path(&RapierDiagnosticsPlugin::STEP_TIME, other_context)
            .as_str(),
        format!("rapier/{other_context}/step_time")
    );
}

#[cfg(feature = "serde-serialize")]
#[test]
fn broad_phase_optimization_strategy_is_applied() {
    for strategy in [
        BroadPhaseOptimizationStrategy::None,
        BroadPhaseOptimizationStrategy::SubtreeOptimizer,
    ] {
        let RapierContextInitialization::InitializeDefaultRapierContext {
            integration_parameters,
            rapier_configuration,
            ..
        } = RapierContextInitialization::default()
        else {
            unreachable!()
        };
        let mut app = test_app(RapierPhysicsPlugin::default().with_custom_initialization(
            RapierContextInitialization::InitializeDefaultRapierContext {
                integration_parameters,
                rapier_configuration,
                broad_phase_optimization_strategy: strategy,
            },
        ));
        app.update();

        // Rapier doesn't expose the strategy, but serializes it.
        let context = context_entity(&mut app);
        let simulation = app.world().get::<RapierContextSimulation>(context).unwrap();
        let json = serde_json::to_value(&simulation.broad_phase).unwrap();
        let expected = match strategy {
            BroadPhaseOptimizationStrategy::None => "None",
            BroadPhaseOptimizationStrategy::SubtreeOptimizer => "SubtreeOptimizer",
        };
        assert_eq!(json["optimization_strategy"], expected);
    }
}

#[cfg(feature = "serde-serialize")]
#[test]
fn configuration_serialization_round_trip() {
    let config = RapierConfiguration {
        simulation_mode: SimulationMode::CollisionOnly,
        num_threads: Some(3),
        ..RapierConfiguration::new(2.0)
    };
    let json = serde_json::to_string(&config).unwrap();
    let restored: RapierConfiguration = serde_json::from_str(&json).unwrap();
    assert_eq!(restored.gravity, config.gravity);
    assert_eq!(restored.simulation_mode, config.simulation_mode);
    assert_eq!(restored.num_threads, config.num_threads);

    let timestep = TimestepMode::Interpolated {
        dt: 0.5,
        time_scale: 2.0,
        substeps: 3,
    };
    let json = serde_json::to_string(&timestep).unwrap();
    assert_eq!(
        serde_json::from_str::<TimestepMode>(&json).unwrap(),
        timestep
    );

    let json = serde_json::to_string(&RapierContextInitialization::default()).unwrap();
    let RapierContextInitialization::InitializeDefaultRapierContext {
        integration_parameters,
        ..
    } = serde_json::from_str(&json).unwrap()
    else {
        panic!("unexpected initialization");
    };
    assert_eq!(integration_parameters.length_unit, 1.0);
}

#[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
#[test]
fn thread_pool_configuration_is_applied() {
    let mut app = test_app(RapierPhysicsPlugin::default());
    app.update();
    let context = context_entity(&mut app);
    let num_threads = |app: &App| {
        app.world()
            .get::<RapierContextSimulation>(context)
            .unwrap()
            .pipeline
            .num_threads()
    };
    assert_eq!(num_threads(&app), None);

    configure(&mut app, |config| config.num_threads = Some(2));
    app.update();
    assert_eq!(num_threads(&app), Some(2));

    configure(&mut app, |config| config.num_threads = None);
    app.update();
    assert_eq!(num_threads(&app), None);
}

#[cfg(feature = "serde-serialize")]
#[test]
fn serialization_round_trip_restores_entity_maps() {
    use crate::plugin::context::{RapierContextColliders, RapierContextJoints};

    let mut app = test_app(RapierPhysicsPlugin::default());
    app.update();

    let ground = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, -1.0, 0.0),
            RigidBody::Fixed,
            Collider::ball(1.0),
        ))
        .id();
    let body = app
        .world_mut()
        .spawn((
            Transform::from_xyz(0.0, 5.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
        ))
        .id();
    let jointed = app
        .world_mut()
        .spawn((
            Transform::from_xyz(2.0, 5.0, 0.0),
            RigidBody::Dynamic,
            Collider::ball(0.5),
            ImpulseJoint::new(body, FixedJointBuilder::new().local_anchor2(Vect::X * -2.0)),
        ))
        .id();
    for _ in 0..5 {
        app.update();
    }

    let context = context_entity(&mut app);
    let world = app.world();
    let simulation = world.get::<RapierContextSimulation>(context).unwrap();
    let colliders = world.get::<RapierContextColliders>(context).unwrap();
    let bodies = world.get::<RapierRigidBodySet>(context).unwrap();
    let joints = world.get::<RapierContextJoints>(context).unwrap();

    let simulation_json = serde_json::to_string(simulation).unwrap();
    let colliders_json = serde_json::to_string(colliders).unwrap();
    let bodies_json = serde_json::to_string(bodies).unwrap();
    let joints_json = serde_json::to_string(joints).unwrap();

    let restored_simulation: RapierContextSimulation =
        serde_json::from_str(&simulation_json).unwrap();
    let restored_colliders: RapierContextColliders = serde_json::from_str(&colliders_json).unwrap();
    let restored_bodies: RapierRigidBodySet = serde_json::from_str(&bodies_json).unwrap();
    let restored_joints: RapierContextJoints = serde_json::from_str(&joints_json).unwrap();

    assert_eq!(restored_bodies.entity2body(), bodies.entity2body());
    assert_eq!(
        restored_colliders.entity2collider(),
        colliders.entity2collider()
    );
    assert_eq!(
        restored_joints.entity2impulse_joint(),
        joints.entity2impulse_joint()
    );
    assert_eq!(restored_joints.entity2impulse_joint().len(), 1);
    for entity in [ground, body, jointed] {
        let handle = restored_bodies.entity2body()[&entity];
        assert_eq!(restored_bodies.rigid_body_entity(handle), Some(entity));
        let collider = restored_colliders.entity2collider()[&entity];
        assert_eq!(restored_colliders.collider_entity(collider), Some(entity));
    }

    // Restore the snapshot and check that the simulation continues.
    let before = body_translation(&mut app, body);
    app.world_mut().entity_mut(context).insert((
        restored_simulation,
        restored_colliders,
        restored_bodies,
        restored_joints,
    ));
    for _ in 0..5 {
        app.update();
    }
    let after = body_translation(&mut app, body);
    assert!(after.y < before.y, "{after} should be below {before}");

    let world = app.world();
    let bodies = world.get::<RapierRigidBodySet>(context).unwrap();
    let handle = bodies.entity2body()[&body];
    assert_eq!(bodies.rigid_body_entity(handle), Some(body));
}
