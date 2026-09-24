use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, setup_event_handler)
        .add_systems(FixedUpdate, display_events)
        .add_systems(FixedUpdate, display_contact_info)
        .add_systems(FixedUpdate, display_contact_info_all_from_1_entity)
        .add_systems(FixedUpdate, display_intersection_info)
        .add_systems(FixedUpdate, display_intersection_info_all_from_1_entity)
        .run();
}

#[derive(Resource)]
struct CustomInfo {
    pub entity1: Entity,
    pub entity2: Entity,
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(Camera2d::default());
}

fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    let ground = commands
        .spawn(Collider::cuboid(500.0, 50.0))
        .insert(Transform::from_xyz(0.0, -100.0, 0.0))
        .id();
    /* Create the bouncing ball. */
    let ball1 = commands
        .spawn(RigidBody::Dynamic)
        .insert(ActiveEvents::CONTACT_FORCE_EVENTS)
        .insert(Collider::ball(50.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(-55.0, 400.0, 0.0))
        .id();
    commands.insert_resource(CustomInfo {
        entity1: ground,
        entity2: ball1,
    });

    /* Create the bouncing ball. */
    commands
        .spawn(RigidBody::Dynamic)
        .insert(ActiveEvents::COLLISION_EVENTS)
        .insert(Collider::ball(50.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(55.0, 300.0, 0.0));
}

// DOCUSAURUS: Events start
/* A system that displays the events. */
fn display_events(
    mut collision_events: MessageReader<CollisionEvent>,
    mut contact_force_events: MessageReader<ContactForceEvent>,
) {
    for collision_event in collision_events.read() {
        println!("Received collision event: {:?}", collision_event);
    }

    for contact_force_event in contact_force_events.read() {
        // `started` is `true` only during the first step the contact force exceeds
        // the threshold, and `false` during the next steps where it remains above it.
        if contact_force_event.started {
            println!("Received contact force event: {:?}", contact_force_event);
        }
    }
}
// DOCUSAURUS: Events stop

// DOCUSAURUS: EventHandler start
use bevy_rapier2d::rapier::dynamics::{RigidBodySet, SoftBodySet, SoftBodyTearEvent};
use bevy_rapier2d::rapier::geometry::{
    ColliderSet, CollisionEvent as RapierCollisionEvent, ContactPair,
};
use bevy_rapier2d::rapier::pipeline::EventHandler;

/* An event handler counting the contact points at the time the collisions start. */
#[derive(Clone, Default)]
struct ContactCounter {
    num_contacts: Arc<AtomicUsize>,
}

impl EventHandler for ContactCounter {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        event: RapierCollisionEvent,
        contact_pair: Option<&ContactPair>,
    ) {
        if event.started() {
            // The colliders are identified by their Rapier handles. Their entity can be
            // retrieved with `RapierContextColliders::entity_from_collider`.
            let entity1 =
                RapierContextColliders::entity_from_collider(&colliders[event.collider1()]);
            // The contact pair is `None` if one of the colliders is a sensor.
            if let Some(contact_pair) = contact_pair {
                let num_points: usize = contact_pair
                    .manifolds()
                    .iter()
                    .map(|m| m.points.len())
                    .sum();
                self.num_contacts.fetch_add(num_points, Ordering::Relaxed);
                println!(
                    "Entity {:?} started touching with {} contact points.",
                    entity1, num_points
                );
            }
        }
    }

    fn handle_contact_force_event(
        &self,
        _dt: f32,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: f32,
    ) {
    }

    fn handle_soft_body_tear_event(&self, _soft_bodies: &SoftBodySet, _event: &SoftBodyTearEvent) {}
}

fn setup_event_handler(mut rapier_context: WriteRapierContext) {
    let mut rapier_context = rapier_context.single_mut().unwrap();
    // The handler is called in addition to (not instead of) the Bevy messages.
    rapier_context
        .simulation
        .set_event_handler(ContactCounter::default());
}
// DOCUSAURUS: EventHandler stop

// DOCUSAURUS: ContactGraph1 start
fn display_contact_info(rapier_context: ReadRapierContext, custom_info: Res<CustomInfo>) {
    let rapier_context = rapier_context.single().unwrap();
    let entity1 = custom_info.entity1; // A first entity with a collider attached.
    let entity2 = custom_info.entity2; // A second entity with a collider attached.

    /* Find the contact pair, if it exists, between two colliders. */
    if let Some(contact_pair) = rapier_context.contact_pair(entity1, entity2) {
        // The contact pair exists meaning that the broad-phase identified a potential contact.
        if contact_pair.has_any_active_contact() {
            // The contact pair has active contacts, meaning that it
            // contains contacts for which contact forces were computed.
        }

        // The sum of the contact impulses applied to the first collider during the last timestep.
        println!("Total contact impulse: {}", contact_pair.total_impulse());

        // We may also read the contact manifolds to access the contact geometry.
        for manifold in contact_pair.manifolds() {
            println!("Local-space contact normal: {}", manifold.local_n1());
            println!("Local-space contact normal: {}", manifold.local_n2());
            println!("World-space contact normal: {}", manifold.normal());

            // Read the geometric contacts.
            for contact_point in manifold.points() {
                // Keep in mind that all the geometric contact data are expressed in the local-space of the colliders.
                println!(
                    "Found local contact point 1: {:?}",
                    contact_point.local_p1()
                );
                println!("Found contact distance: {:?}", contact_point.dist()); // Negative if there is a penetration.
                println!("Found contact impulse: {}", contact_point.impulse());
                println!(
                    "Found friction impulse: {}",
                    contact_point.tangent_impulse()
                );
            }

            // Read the solver contacts.
            for solver_contact in manifold.solver_contacts() {
                // The world-space contact points on each body's surface.
                let (point1, point2) =
                    (solver_contact.world_point1(), solver_contact.world_point2());
                println!("Found solver contact points: {point1:?}, {point2:?}");
                // The solver contact distance is negative if there is a penetration.
                println!("Found solver contact distance: {:?}", solver_contact.dist());
            }
        }
    }
}
// DOCUSAURUS: ContactGraph1 stop

// DOCUSAURUS: ContactGraph2 start
fn display_contact_info_all_from_1_entity(
    rapier_context: ReadRapierContext,
    custom_info: Res<CustomInfo>,
) {
    let rapier_context = rapier_context.single().unwrap();
    let entity = custom_info.entity2; // An entity with a collider attached.

    /* Iterate through all the contact pairs involving a specific collider. */
    for contact_pair in rapier_context.contact_pairs_with(entity) {
        let other_collider = if contact_pair.collider1() == Some(entity) {
            contact_pair.collider2()
        } else {
            contact_pair.collider1()
        };

        // Process the contact pair in a way similar to what we did in
        // the previous example.
    }
}
// DOCUSAURUS: ContactGraph2 stop

// DOCUSAURUS: IntersectionGraph1 start
fn display_intersection_info(rapier_context: ReadRapierContext, custom_info: Res<CustomInfo>) {
    let rapier_context = rapier_context.single().unwrap();
    let entity1 = custom_info.entity1; // A first entity with a collider attached.
    let entity2 = custom_info.entity2; // A second entity with a collider attached.

    /* Find the intersection pair, if it exists, between two colliders. */
    if rapier_context.intersection_pair(entity1, entity2) == Some(true) {
        println!(
            "The entities {:?} and {:?} have intersecting colliders!",
            entity1, entity2
        );
    }
}
// DOCUSAURUS: IntersectionGraph1 stop

// DOCUSAURUS: IntersectionGraph2 start
fn display_intersection_info_all_from_1_entity(
    rapier_context: ReadRapierContext,
    custom_info: Res<CustomInfo>,
) {
    let rapier_context = rapier_context.single().unwrap();
    let entity = custom_info.entity2; // An entity with a collider attached.

    /* Iterate through all the intersection pairs involving a specific collider. */
    for (collider1, collider2, intersecting) in rapier_context.intersection_pairs_with(entity) {
        if intersecting {
            println!(
                "The entities {:?} and {:?} have intersecting colliders!",
                collider1, collider2
            );
        }
    }
}
// DOCUSAURUS: IntersectionGraph2 stop
