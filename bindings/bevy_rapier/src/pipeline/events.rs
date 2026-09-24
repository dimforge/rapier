use crate::math::{Real, Vect};
use bevy::ecs::message::Message;
use bevy::prelude::Entity;
use rapier::dynamics::{RigidBodySet, SoftBodySet, SoftBodyTearEvent as RapierSoftBodyTearEvent};
use rapier::geometry::{
    ColliderHandle, ColliderSet, CollisionEvent as RapierCollisionEvent, CollisionEventFlags,
    ContactForceEvent as RapierContactForceEvent, ContactPair,
};
use rapier::pipeline::EventHandler;
use std::collections::HashMap;
use std::sync::RwLock;

#[cfg(doc)]
use crate::prelude::{
    ActiveEvents, ContactForceEventThreshold, RapierContext, RapierContextMut,
    RapierContextSimulation, SoftBodyAttachments, SoftBodyExternalForce, SoftBodyExternalImpulse,
    SoftBodyKinematicTargets, SoftBodyPinnedParticles,
};

/// Events occurring when two colliders start or stop colliding
///
/// This will only get triggered if the entity has the
/// [`ActiveEvents::COLLISION_EVENTS`] flag enabled.
///
/// This event only identifies the colliders involved. The detailed contact geometry
/// (manifolds, contact points, normals, impulses) of a non-sensor pair can be read from the
/// narrow-phase with [`RapierContext::contact_pair`] (or
/// [`RapierContextSimulation::contact_pair`]) while the pair still exists, i.e., for
/// [`CollisionEvent::Started`], and for [`CollisionEvent::Stopped`] unless one of the colliders
/// was removed. For sensors, use [`RapierContext::intersection_pair`] instead. To access the raw
/// Rapier contact pair at the exact time the event is emitted, install a custom
/// [`EventHandler`] with [`RapierContextSimulation::set_event_handler`].
#[derive(Message, Copy, Clone, Debug, PartialEq, Eq)]
pub enum CollisionEvent {
    /// Event occurring when two colliders start colliding
    Started(Entity, Entity, CollisionEventFlags),
    /// Event occurring when two colliders stop colliding
    Stopped(Entity, Entity, CollisionEventFlags),
}

/// Event emitted after a simulation step during which Rapier detected non-finite (NaN or
/// infinite) state and neutralized it.
///
/// Each reported rigid-body was rolled back to its last valid pose (when known), had its
/// velocities and forces zeroed, and was disabled inside of Rapier; each reported collider was
/// disabled inside of Rapier; each reported soft body was disabled and had its velocities zeroed
/// (its non-finite particle positions are left as is). The plugin mirrors this by inserting
/// [`RigidBodyDisabled`], [`ColliderDisabled`] or [`SoftBodyDisabled`] on the quarantined
/// entities. Once the offending state is fixed (e.g. by writing a finite [`Velocity`],
/// [`Transform`] or [`Collider`], or by moving the particles with
/// [`RapierSoftBody::set_particle_position`]), remove that component to re-enable the entity. A
/// warning is also logged for each event.
///
/// [`RigidBodyDisabled`]: crate::dynamics::RigidBodyDisabled
/// [`ColliderDisabled`]: crate::geometry::ColliderDisabled
/// [`SoftBodyDisabled`]: crate::dynamics::SoftBodyDisabled
/// [`Velocity`]: crate::dynamics::Velocity
/// [`Collider`]: crate::geometry::Collider
/// [`Transform`]: bevy::prelude::Transform
/// [`RapierSoftBody::set_particle_position`]: crate::dynamics::RapierSoftBody::set_particle_position
#[derive(Message, Clone, Debug, PartialEq, Eq)]
pub struct PhysicsQuarantineEvent {
    /// The entity of the Rapier context in which the non-finite state was detected.
    pub context: Entity,
    /// The rigid-body entities quarantined during the step.
    pub bodies: Vec<Entity>,
    /// The collider entities quarantined during the step (because their own geometry or
    /// position became non-finite).
    pub colliders: Vec<Entity>,
    /// The soft-body entities quarantined during the step (because a particle's position or
    /// velocity became non-finite).
    pub soft_bodies: Vec<Entity>,
}

/// Event occurring when the sum of the magnitudes of the contact forces
/// between two colliders exceed a threshold ([`ContactForceEventThreshold`]).
///
/// This will only get triggered if the entity has the
/// [`ActiveEvents::CONTACT_FORCE_EVENTS`] flag enabled.
#[derive(Message, Copy, Clone, Debug, PartialEq)]
pub struct ContactForceEvent {
    /// The first collider involved in the contact.
    pub collider1: Entity,
    /// The second collider involved in the contact.
    pub collider2: Entity,
    /// The sum of all the forces between the two colliders.
    pub total_force: Vect,
    /// The sum of the magnitudes of each force between the two colliders.
    ///
    /// Note that this is **not** the same as the magnitude of `self.total_force`.
    /// Here we are summing the magnitude of all the forces, instead of taking
    /// the magnitude of their sum.
    pub total_force_magnitude: Real,
    /// The world-space (unit) direction of the force with strongest magnitude.
    pub max_force_direction: Vect,
    /// The magnitude of the largest force at a contact point of this contact pair.
    pub max_force_magnitude: Real,
    /// Is this the first step the total force of this pair exceeded its threshold?
    ///
    /// This is `true` on the step the force crosses the [`ContactForceEventThreshold`]
    /// from below (or from not touching), and `false` while it stays above it on subsequent
    /// steps. It becomes `true` again after the force drops below the threshold or the
    /// colliders separate.
    pub started: bool,
}

/// Message sent after a soft body tore (an element loaded past its tear threshold, or a tear or
/// cut requested through [`RapierContextMut::tear_soft_body`] or
/// [`RapierContextMut::cut_soft_body`]).
///
/// When a tear splits the soft body into disconnected pieces, the largest piece keeps the torn
/// soft body and its entity; each other piece becomes a new soft body for which the plugin spawns
/// a new entity (or uses the one already returned by the manual tear, see
/// [`SoftBodyTearResult`]). That entity is a clone of the torn soft body's entity (its material,
/// mesh synchronization, render components, user components...), except for the components tied
/// to the torn body's particle indices ([`SoftBodyPinnedParticles`], [`SoftBodyKinematicTargets`],
/// [`SoftBodyAttachments`], [`SoftBodyExternalForce`], [`SoftBodyExternalImpulse`]), which are
/// remapped to the particles of each piece, and its mesh (a new one is generated if
/// [`SoftBodyMeshSync`] is present).
///
/// [`SoftBodyMeshSync`]: crate::dynamics::SoftBodyMeshSync
#[derive(Message, Clone, Debug)]
pub struct SoftBodyTearEvent {
    /// The entity of the Rapier context the soft body belongs to.
    pub context: Entity,
    /// The entity of the soft body that tore.
    pub soft_body: Entity,
    /// The soft bodies the torn body came apart into (empty if nothing was split off).
    ///
    /// The first piece is the torn soft body itself (it keeps [`Self::soft_body`] as entity), the
    /// others are the entities spawned for the new soft bodies. The particle indices follow those
    /// of [`Self::raw`].
    pub pieces: Vec<SoftBodyTearPiece>,
    /// The pieces of every soft-body cluster the tear split (see [`SoftBodyClusterSplit`]).
    pub cluster_splits: Vec<SoftBodyClusterSplit>,
    /// The impulse joints moved from a cluster proxy to another by a cluster split.
    pub moved_joints: Vec<SoftBodyJointMove>,
    /// The raw event from Rapier, with every detail of the topology change in terms of Rapier
    /// handles and particle indices.
    pub raw: RapierSoftBodyTearEvent,
}

/// The result of a tear or cut requested through [`RapierContextMut::tear_soft_body`] or
/// [`RapierContextMut::cut_soft_body`].
///
/// The entities of the pieces are known right away: the soft bodies of the pieces are mapped to
/// them immediately (so the helpers of [`RapierRigidBodySet`] find them), and the entities are
/// spawned by the given `Commands`. They receive their components (cloned from the torn entity,
/// see [`SoftBodyTearEvent`]) during the next [`PhysicsSet::Writeback`], when the
/// [`SoftBodyTearEvent`] message of this tear is sent. The entities of the clusters split by the
/// tear are only spawned then (see [`SoftBodyTearEvent::cluster_splits`]).
///
/// [`RapierRigidBodySet`]: crate::plugin::context::RapierRigidBodySet
/// [`PhysicsSet::Writeback`]: crate::plugin::PhysicsSet::Writeback
#[derive(Clone, Debug)]
pub struct SoftBodyTearResult {
    /// The entities of the soft bodies the torn body came apart into, in the order of the pieces
    /// of [`Self::raw`] (empty if nothing was split off). The first one is the torn soft body's
    /// entity.
    pub pieces: Vec<Entity>,
    /// The raw event from Rapier.
    pub raw: RapierSoftBodyTearEvent,
}

/// A soft body resulting from a tear (see [`SoftBodyTearEvent::pieces`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SoftBodyTearPiece {
    /// The entity of the soft body holding the piece.
    pub soft_body: Entity,
    /// The particles of the piece: `particles[i]` is the index, in the torn body after the tear,
    /// of the piece's `i`-th particle.
    pub particles: Vec<u32>,
}

/// A piece of a soft-body cluster split by a tear (see [`SoftBodyTearEvent::cluster_splits`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SoftBodyClusterSplit {
    /// The index of the split cluster in the torn body, before the tear.
    pub source_cluster: u32,
    /// The entity of the soft body holding the piece.
    pub soft_body: Entity,
    /// The piece's cluster index in that soft body.
    pub cluster: u32,
    /// The entity the piece's proxy rigid-body maps to: the [`SoftBodyCluster`] entity of the
    /// cluster (a new one is spawned for a piece given a fresh proxy), or the soft body entity for
    /// a cluster without entity.
    ///
    /// [`SoftBodyCluster`]: crate::dynamics::SoftBodyCluster
    pub proxy: Option<Entity>,
    /// Whether this piece kept the proxy of the source cluster.
    pub keeps_proxy: bool,
}

/// An impulse joint moved to another cluster proxy by a tear (see
/// [`SoftBodyTearEvent::moved_joints`]).
///
/// The [`ImpulseJoint`](crate::dynamics::ImpulseJoint) component of the joint entity is not
/// modified: its `parent` may no longer match the Rapier joint.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SoftBodyJointMove {
    /// The entity of the joint.
    pub joint: Option<Entity>,
    /// The entity of the proxy the joint was attached to.
    pub from: Option<Entity>,
    /// The entity of the proxy the joint is attached to now.
    pub to: Option<Entity>,
}

/// Forwards every event emitted by Rapier to Bevy's event queue and to the user-provided
/// event handler (see [`RapierContextSimulation::set_event_handler`]), if any.
pub(crate) struct EventHandlerFanOut<'a> {
    pub queue: Option<&'a EventQueue<'a>>,
    pub user: Option<&'a (dyn EventHandler + Send + Sync)>,
    /// Collects the soft-body tear events, processed after the step to spawn the piece entities.
    pub soft_body_tears: &'a RwLock<Vec<RapierSoftBodyTearEvent>>,
}

impl EventHandlerFanOut<'_> {
    fn handlers(&self) -> impl Iterator<Item = &dyn EventHandler> {
        self.queue
            .map(|q| q as &dyn EventHandler)
            .into_iter()
            .chain(self.user.map(|h| h as &dyn EventHandler))
    }
}

impl EventHandler for EventHandlerFanOut<'_> {
    fn handle_collision_event(
        &self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: RapierCollisionEvent,
        contact_pair: Option<&ContactPair>,
    ) {
        for handler in self.handlers() {
            handler.handle_collision_event(bodies, colliders, event, contact_pair);
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        for handler in self.handlers() {
            handler.handle_contact_force_event(
                dt,
                bodies,
                colliders,
                contact_pair,
                total_force_magnitude,
            );
        }
    }

    fn handle_soft_body_tear_event(
        &self,
        soft_bodies: &SoftBodySet,
        event: &RapierSoftBodyTearEvent,
    ) {
        if let Ok(mut tears) = self.soft_body_tears.write() {
            tears.push(event.clone());
        }
        for handler in self.handlers() {
            handler.handle_soft_body_tear_event(soft_bodies, event);
        }
    }
}

// TODO: it may be more efficient to use crossbeam channel.
// However crossbeam channels cause a Segfault (I have not
// investigated how to reproduce this exactly to open an
// issue).
/// A set of queues collecting events emitted by the physics engine.
pub(crate) struct EventQueue<'a> {
    // Used to retrieve the entity of colliders that have been removed from the simulation
    // since the last physics step.
    pub deleted_colliders: &'a HashMap<ColliderHandle, Entity>,
    pub collision_events: RwLock<Vec<CollisionEvent>>,
    pub contact_force_events: RwLock<Vec<ContactForceEvent>>,
}

impl EventQueue<'_> {
    fn collider2entity(&self, colliders: &ColliderSet, handle: ColliderHandle) -> Option<Entity> {
        let entity = colliders
            .get(handle)
            .and_then(|co| Entity::try_from_bits(co.user_data as u64))
            .or_else(|| self.deleted_colliders.get(&handle).copied());
        if entity.is_none() {
            // This happens for colliders Rapier removed by itself (e.g. the colliders of a torn
            // soft body rebuilt by the tear) before the event was emitted.
            log::debug!("No entity found for collider {handle:?}; its event is dropped.");
        }
        entity
    }
}

impl EventHandler for EventQueue<'_> {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: RapierCollisionEvent,
        _: Option<&ContactPair>,
    ) {
        let event = match event {
            RapierCollisionEvent::Started(h1, h2, flags) => {
                let (Some(e1), Some(e2)) = (
                    self.collider2entity(colliders, h1),
                    self.collider2entity(colliders, h2),
                ) else {
                    return;
                };
                CollisionEvent::Started(e1, e2, flags)
            }
            RapierCollisionEvent::Stopped(h1, h2, flags) => {
                let (Some(e1), Some(e2)) = (
                    self.collider2entity(colliders, h1),
                    self.collider2entity(colliders, h2),
                ) else {
                    return;
                };
                CollisionEvent::Stopped(e1, e2, flags)
            }
        };

        if let Ok(mut events) = self.collision_events.write() {
            events.push(event);
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let rapier_event =
            RapierContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        let (Some(collider1), Some(collider2)) = (
            self.collider2entity(colliders, rapier_event.collider1),
            self.collider2entity(colliders, rapier_event.collider2),
        ) else {
            return;
        };
        let event = ContactForceEvent {
            collider1,
            collider2,
            total_force: rapier_event.total_force,
            total_force_magnitude: rapier_event.total_force_magnitude,
            max_force_direction: rapier_event.max_force_direction,
            max_force_magnitude: rapier_event.max_force_magnitude,
            started: rapier_event.started,
        };

        if let Ok(mut events) = self.contact_force_events.write() {
            events.push(event);
        }
    }
    // Tear events are collected by `EventHandlerFanOut` and turned into Bevy messages after the
    // step, once the entities of the new pieces are spawned.
    fn handle_soft_body_tear_event(
        &self,
        _soft_bodies: &SoftBodySet,
        _event: &RapierSoftBodyTearEvent,
    ) {
    }
}

#[cfg(test)]
mod test {
    use bevy::{
        app::{App, Startup, Update},
        prelude::{Commands, Component, Entity, Query, With},
        time::{TimePlugin, TimeUpdateStrategy},
        transform::{components::Transform, TransformPlugin},
        MinimalPlugins,
    };

    use crate::{plugin::*, prelude::*};

    #[cfg(feature = "dim3")]
    fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
        Collider::cuboid(hx, hy, hz)
    }
    #[cfg(feature = "dim2")]
    fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
        Collider::cuboid(hx, hy)
    }

    #[test]
    pub fn events_received() {
        return main();

        use bevy::prelude::*;

        #[derive(Resource, Reflect)]
        pub struct EventsSaver<E: Message> {
            pub events: Vec<E>,
        }
        impl<E: Message> Default for EventsSaver<E> {
            fn default() -> Self {
                Self {
                    events: Default::default(),
                }
            }
        }
        pub fn save_events<E: Message + Clone>(
            mut events: MessageReader<E>,
            mut saver: ResMut<EventsSaver<E>>,
        ) {
            for event in events.read() {
                saver.events.push(event.clone());
            }
        }
        fn run_test(app: &mut App) {
            app.add_systems(PostUpdate, save_events::<CollisionEvent>)
                .add_systems(PostUpdate, save_events::<ContactForceEvent>)
                .init_resource::<EventsSaver<CollisionEvent>>()
                .init_resource::<EventsSaver<ContactForceEvent>>();

            // Simulates 60 updates per seconds
            app.insert_resource(TimeUpdateStrategy::ManualDuration(
                std::time::Duration::from_secs_f32(1f32 / 60f32),
            ));
            app.finish();
            // 2 seconds should be plenty of time for the cube to fall on the
            // lowest collider.
            for _ in 0..120 {
                app.update();
            }
            let saved_collisions = app
                .world()
                .get_resource::<EventsSaver<CollisionEvent>>()
                .unwrap();
            assert!(!saved_collisions.events.is_empty());
            let saved_contact_forces = app
                .world()
                .get_resource::<EventsSaver<CollisionEvent>>()
                .unwrap();
            assert!(!saved_contact_forces.events.is_empty());
        }

        /// Adapted from events example
        fn main() {
            let mut app = App::new();
            app.add_plugins((
                TransformPlugin,
                TimePlugin,
                RapierPhysicsPlugin::<NoUserData>::default(),
            ))
            .add_systems(Startup, setup_physics);
            run_test(&mut app);
        }

        pub fn setup_physics(mut commands: Commands) {
            /*
             * Ground
             */
            commands.spawn((Transform::from_xyz(0.0, -1.2, 0.0), cuboid(4.0, 1.0, 1.0)));

            commands.spawn((
                Transform::from_xyz(0.0, 5.0, 0.0),
                cuboid(4.0, 1.5, 1.0),
                Sensor,
            ));

            commands.spawn((
                Transform::from_xyz(0.0, 13.0, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                ActiveEvents::COLLISION_EVENTS,
                ContactForceEventThreshold(30.0),
            ));
        }
    }

    fn falling_box_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            TransformPlugin,
            TimePlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ))
        .add_systems(Startup, |mut commands: Commands| {
            commands.spawn((Transform::from_xyz(0.0, -1.0, 0.0), cuboid(4.0, 1.0, 4.0)));
            commands.spawn((
                Transform::from_xyz(0.0, 0.6, 0.0),
                RigidBody::Dynamic,
                cuboid(0.5, 0.5, 0.5),
                ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS,
                ContactForceEventThreshold(0.0),
            ));
        });
        app
    }

    #[test]
    pub fn contact_force_event_started_flag() {
        use bevy::prelude::*;

        #[derive(Resource, Default)]
        struct Saved(Vec<ContactForceEvent>);

        let mut app = falling_box_app();
        app.init_resource::<Saved>().add_systems(
            PostUpdate,
            |mut events: MessageReader<ContactForceEvent>, mut saved: ResMut<Saved>| {
                saved.0.extend(events.read().copied());
            },
        );
        app.finish();
        for _ in 0..120 {
            app.update();
        }

        let events = &app.world().resource::<Saved>().0;
        assert!(events.len() > 2);
        // The first event is emitted when the force first crosses the threshold.
        assert!(events[0].started);
        // The box then rests on the ground, keeping the force above the threshold.
        assert!(!events.last().unwrap().started);
        assert!(events.iter().filter(|e| e.started).count() < events.len());
    }

    #[test]
    pub fn custom_event_handler_receives_events() {
        use bevy::prelude::*;
        use rapier::dynamics::{RigidBodySet, SoftBodySet, SoftBodyTearEvent};
        use rapier::geometry::{ColliderSet, CollisionEvent as RapierCollisionEvent, ContactPair};
        use rapier::pipeline::EventHandler;
        use std::sync::{
            atomic::{AtomicUsize, Ordering},
            Arc,
        };

        #[derive(Default, Clone)]
        struct Counters {
            collisions: Arc<AtomicUsize>,
            collisions_with_pair: Arc<AtomicUsize>,
            forces: Arc<AtomicUsize>,
        }

        impl EventHandler for Counters {
            fn handle_collision_event(
                &self,
                _bodies: &RigidBodySet,
                _colliders: &ColliderSet,
                _event: RapierCollisionEvent,
                contact_pair: Option<&ContactPair>,
            ) {
                self.collisions.fetch_add(1, Ordering::SeqCst);
                if contact_pair.is_some() {
                    self.collisions_with_pair.fetch_add(1, Ordering::SeqCst);
                }
            }

            fn handle_contact_force_event(
                &self,
                _dt: crate::math::Real,
                _bodies: &RigidBodySet,
                _colliders: &ColliderSet,
                _contact_pair: &ContactPair,
                _total_force_magnitude: crate::math::Real,
            ) {
                self.forces.fetch_add(1, Ordering::SeqCst);
            }

            fn handle_soft_body_tear_event(
                &self,
                _soft_bodies: &SoftBodySet,
                _event: &SoftBodyTearEvent,
            ) {
            }
        }

        #[derive(Resource, Default)]
        struct BevyCounts {
            collisions: usize,
            forces: usize,
        }

        let counters = Counters::default();
        let handler = counters.clone();
        let mut app = falling_box_app();
        app.init_resource::<BevyCounts>()
            .add_systems(
                Startup,
                move |mut contexts: Query<&mut RapierContextSimulation>| {
                    for mut context in contexts.iter_mut() {
                        context.set_event_handler(handler.clone());
                    }
                },
            )
            .add_systems(
                PostUpdate,
                |mut collisions: MessageReader<CollisionEvent>,
                 mut forces: MessageReader<ContactForceEvent>,
                 mut counts: ResMut<BevyCounts>| {
                    counts.collisions += collisions.read().count();
                    counts.forces += forces.read().count();
                },
            );
        app.finish();
        for _ in 0..60 {
            app.update();
        }

        let bevy_counts = app.world().resource::<BevyCounts>();
        let collisions = counters.collisions.load(Ordering::SeqCst);
        let forces = counters.forces.load(Ordering::SeqCst);
        assert!(collisions > 0);
        assert!(counters.collisions_with_pair.load(Ordering::SeqCst) > 0);
        assert!(forces > 0);
        // Bevy messages are still sent alongside the custom handler.
        assert_eq!(bevy_counts.collisions, collisions);
        assert_eq!(bevy_counts.forces, forces);
    }

    #[test]
    pub fn spam_remove_rapier_entity_interpolated() {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
        ))
        .insert_resource(TimestepMode::Interpolated {
            dt: 1.0 / 30.0,
            time_scale: 1.0,
            substeps: 2,
        })
        .add_systems(Startup, setup_physics)
        .add_systems(Update, remove_collider);
        // Simulates 60 updates per seconds
        app.insert_resource(TimeUpdateStrategy::ManualDuration(
            std::time::Duration::from_secs_f32(1f32 / 60f32),
        ));

        app.finish();

        for _ in 0..100 {
            app.update();
        }
        return;

        #[derive(Component)]
        pub struct ToRemove;

        #[cfg(feature = "dim3")]
        fn cuboid(hx: Real, hy: Real, hz: Real) -> Collider {
            Collider::cuboid(hx, hy, hz)
        }
        #[cfg(feature = "dim2")]
        fn cuboid(hx: Real, hy: Real, _hz: Real) -> Collider {
            Collider::cuboid(hx, hy)
        }
        pub fn setup_physics(mut commands: Commands) {
            for _i in 0..100 {
                commands.spawn((
                    Transform::from_xyz(0.0, 0.0, 0.0),
                    RigidBody::Dynamic,
                    cuboid(0.5, 0.5, 0.5),
                    ActiveEvents::all(),
                    ToRemove,
                ));
            }
            /*
             * Ground
             */
            let ground_size = 5.1;
            let ground_height = 0.1;
            let starting_y = -0.5 - ground_height;

            commands.spawn((
                Transform::from_xyz(0.0, starting_y, 0.0),
                cuboid(ground_size, ground_height, ground_size),
            ));
        }

        fn remove_collider(mut commands: Commands, query: Query<Entity, With<ToRemove>>) {
            let Some(entity) = query.iter().next() else {
                return;
            };
            commands.entity(entity).despawn();
        }
    }
}
