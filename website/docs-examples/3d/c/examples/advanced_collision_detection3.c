#include "snippets.h"

static int same_collider(R3ColliderHandle a, R3ColliderHandle b) {
    return a.world == b.world && a.index == b.index && a.generation == b.generation;
}

// DOCUSAURUS: PhysicsHooks start
// This is a silly example of contact pair filter that:
// - Enables contact and force computation if both colliders have even user-data.
// - Enables contact computation but not force computation if both colliders have equal user-data.
// - Disables contact computation otherwise.
static int32_t RAPIER_CALL filter_contact_pair(void *user_data, const R3ReadContext *read,
                                               R3ColliderHandle collider1, R3ColliderHandle collider2,
                                               R3RigidBodyHandle body1, R3RigidBodyHandle body2) {
    (void)user_data;
    (void)body1;
    (void)body2;
    uint64_t user_data1 = r3ReadCollider_UserData(read, collider1).low;
    uint64_t user_data2 = r3ReadCollider_UserData(read, collider2).low;

    if (user_data1 % 2 == 0 && user_data2 % 2 == 0) {
        return 1; // Compute the contacts and the contact forces.
    } else if (user_data1 == user_data2) {
        return 0; // Compute the contacts, but not the contact forces.
    } else {
        return -1; // Don't compute any contact.
    }
}

// This is a silly example of intersection pair filter that
// enables the intersection test if both colliders have odd
// user-data.
static int32_t RAPIER_CALL filter_intersection_pair(void *user_data, const R3ReadContext *read,
                                                    R3ColliderHandle collider1, R3ColliderHandle collider2,
                                                    R3RigidBodyHandle body1, R3RigidBodyHandle body2) {
    (void)user_data;
    (void)body1;
    (void)body2;
    uint64_t user_data1 = r3ReadCollider_UserData(read, collider1).low;
    uint64_t user_data2 = r3ReadCollider_UserData(read, collider2).low;

    return user_data1 % 2 == 1 && user_data2 % 2 == 1;
}

static void step_with_pair_filters(R3World *world) {
    // NULL callbacks keep the default behavior.
    R3PhysicsHooks hooks = {0};
    hooks.user_data = NULL; // Given to every callback as its first argument.
    hooks.filter_contact_pair = filter_contact_pair;
    hooks.filter_intersection_pair = filter_intersection_pair;

    r3Step(world, &hooks, NULL);
}
// DOCUSAURUS: PhysicsHooks stop

// DOCUSAURUS: ContactModification start
// This is a silly example of contact modifier that does silly things
// for illustration purpose:
// - Flip all the contact normals.
// - Set the friction coefficient to 0.3
// - Set the restitution coefficient to 0.4
// - Set the tangent velocities to X * 10.0
// The contacts of two soft surfaces are candidates rather than a manifold:
// only the manifolds of rigid pairs are given to this callback.
static void RAPIER_CALL modify_manifold(void *user_data, const R3ReadContext *read,
                                        R3ColliderHandle collider1, R3ColliderHandle collider2,
                                        R3ContactModification *manifold) {
    (void)user_data;
    (void)read;
    (void)collider1;
    (void)collider2;
    manifold->normal = r3VectorScale(manifold->normal, -1.0);

    // Friction and restitution are combined once per manifold, so they are set
    // for the whole manifold rather than per solver contact.
    manifold->friction = 0.3;
    manifold->restitution = 0.4;

    // Use the persistent user-data to count the number of times
    // contact modification was called for this contact manifold
    // since its creation.
    manifold->user_data += 1;
    printf("Contact manifold has been modified %u times since its creation.\n", manifold->user_data);
}

// Called right after `modify_manifold`, for the same manifold.
static void RAPIER_CALL modify_solver_contacts(void *user_data, const R3ReadContext *read,
                                               R3ColliderHandle collider1, R3ColliderHandle collider2,
                                               R3ContactModificationContext *context) {
    (void)user_data;
    (void)read;
    (void)collider1;
    (void)collider2;
    r3ContactModificationContext_SetTangentVelocity(context, r3Vector(10.0, 0.0, 0.0));
}

static void step_with_contact_modification(R3World *world) {
    R3PhysicsHooks hooks = {0};
    hooks.modify_solver_contacts = modify_manifold;
    hooks.modify_solver_contacts_context = modify_solver_contacts;

    r3Step(world, &hooks, NULL);
}
// DOCUSAURUS: ContactModification stop

// DOCUSAURUS: ContactModificationPerContact start
// Modifies the solver contacts one by one:
// - Delete the first contact.
// - Set the tangent velocities to X * 10.0
static void RAPIER_CALL modify_each_solver_contact(void *user_data, const R3ReadContext *read,
                                                   R3ColliderHandle collider1, R3ColliderHandle collider2,
                                                   R3ContactModificationContext *context) {
    (void)user_data;
    (void)read;
    (void)collider1;
    (void)collider2;
    // The contacts of two soft surfaces are candidates rather than a manifold.
    if (r3ContactModificationContext_IsSoft(context)) {
        return;
    }

    // The last solver contact takes the place of the removed one.
    if (r3ContactModificationContext_SolverContactCount(context) > 0) {
        r3ContactModificationContext_RemoveSolverContact(context, 0);
    }

    size_t count = r3ContactModificationContext_SolverContactCount(context);
    for (size_t i = 0; i < count; i++) {
        R3SolverContact solver_contact = r3ContactModificationContext_SolverContact(context, i);
        solver_contact.tangent_velocity.x = 10.0;
        r3ContactModificationContext_SetSolverContact(context, i, &solver_contact);
    }
}
// DOCUSAURUS: ContactModificationPerContact stop

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();

    /* Create the ground. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    R3ColliderHandle collider_handle1 = r3InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle ball_body_handle = r3InsertRigidBody(world, &ball_body);
    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    ball.restitution = 0.7;
    ball.activeEvents = R3_COLLISION_EVENTS | R3_CONTACT_FORCE_EVENTS;
    R3ColliderHandle collider_handle2 = r3InsertCollider(ball_body_handle, &ball);

    /* A sensor intersecting the ground (both are fixed). */
    R3ColliderDesc sensor = r3BallColliderDesc(0.5);
    sensor.position.translation = r3Vector(5.0, 0.0, 0.0);
    sensor.isSensor = 1;
    sensor.activeCollisionTypes = R3_COLLISION_TYPES_DEFAULT | R3_COLLISION_TYPES_FIXED_FIXED;
    r3InsertColliderWithoutParent(world, &sensor);

    // DOCUSAURUS: Events start
    // Initialize the event collector.
    R3EventCollector *events = r3NewEventCollector();

    r3Step(world, NULL, events);

    // Get the number of collision events, then copy them.
    size_t count = r3EventCollector_CollisionEvents(events, NULL, 0);
    R3CollisionEvent *collision_events = malloc(count * sizeof(*collision_events));
    count = r3EventCollector_CollisionEvents(events, collision_events, count);
    for (size_t i = 0; i < count; i++) {
        // Handle the collision event.
        R3CollisionEvent event = collision_events[i];
        printf("Received collision event: colliders %u and %u, started: %u, flags: %u\n",
               event.collider1.index, event.collider2.index, event.started, event.flags);
    }
    free(collision_events);

    count = r3EventCollector_ContactForceEvents(events, NULL, 0);
    R3ContactForceEvent *contact_force_events = malloc(count * sizeof(*contact_force_events));
    count = r3EventCollector_ContactForceEvents(events, contact_force_events, count);
    for (size_t i = 0; i < count; i++) {
        // Handle the contact force event.
        R3ContactForceEvent event = contact_force_events[i];
        printf("Received contact force event: colliders %u and %u, force magnitude: %f\n",
               event.collider1.index, event.collider2.index, (double)event.total_force_magnitude);
    }
    free(contact_force_events);

    count = r3EventCollector_TearEventCount(events);
    for (size_t i = 0; i < count; i++) {
        // Handle the soft-body tear event. It is a copy that must be freed.
        R3SoftBodyTearEvent *tear_event = r3EventCollector_TearEvent(events, i);
        printf("Received soft-body tear event: soft-body %u\n",
               r3SoftBodyTearEvent_SoftBody(tear_event).index);
        r3FreeSoftBodyTearEvent(tear_event);
    }

    // The events accumulate until the collector is cleared.
    r3EventCollector_Clear(events);
    // DOCUSAURUS: Events stop

    /* Let the ball fall on the ground. */
    for (int i = 0; i < 200; i++) {
        r3Step(world, NULL, events);
    }
    size_t collision_count = r3EventCollector_CollisionEvents(events, NULL, 0);
    size_t force_count = r3EventCollector_ContactForceEvents(events, NULL, 0);
    printf("%zu collision events and %zu contact force events\n", collision_count, force_count);
    if (collision_count == 0 || force_count == 0) {
        return EXIT_FAILURE;
    }
    r3FreeEventCollector(events);

    // DOCUSAURUS: ContactGraph1 start
    /* Find the contact pair, if it exists, between two colliders. */
    R3OptionalContactPair contact_pair = r3TryContactPair(collider_handle1, collider_handle2);
    if (contact_pair.found) {
        // The contact pair exists meaning that the broad-phase identified a potential contact.
        if (contact_pair.pair.has_any_active_contact) {
            // The contact pair has active contacts, meaning that it
            // contains contacts for which contact forces were computed.
        }

        // We may also read the contact manifolds to access the contact geometry.
        size_t num_manifolds = r3ContactManifolds(collider_handle1, collider_handle2, NULL, 0);
        R3ContactManifold *manifolds = malloc(num_manifolds * sizeof(*manifolds));
        num_manifolds = r3ContactManifolds(collider_handle1, collider_handle2, manifolds, num_manifolds);
        // The geometric contacts of all the manifolds, each with the index of its manifold.
        size_t num_points = r3ContactPoints(collider_handle1, collider_handle2, NULL, 0);
        R3ContactPoint *points = malloc(num_points * sizeof(*points));
        num_points = r3ContactPoints(collider_handle1, collider_handle2, points, num_points);

        for (size_t i = 0; i < num_manifolds; i++) {
            R3ContactManifold manifold = manifolds[i];
            printf("Local-space contact normal: (%f, %f, %f)\n", (double)manifold.local_n1.x, (double)manifold.local_n1.y,
                   (double)manifold.local_n1.z);
            printf("Local-space contact normal: (%f, %f, %f)\n", (double)manifold.local_n2.x, (double)manifold.local_n2.y,
                   (double)manifold.local_n2.z);
            printf("World-space contact normal: (%f, %f, %f)\n", (double)manifold.normal.x, (double)manifold.normal.y,
                   (double)manifold.normal.z);

            // Read the geometric contacts.
            for (size_t j = 0; j < num_points; j++) {
                if (points[j].manifold_index != i) {
                    continue;
                }
                // Keep in mind that all the geometric contact data are expressed in the local-space of the colliders.
                R3ContactPoint contact_point = points[j];
                printf("Found local contact point 1: (%f, %f, %f)\n", (double)contact_point.local_p1.x,
                       (double)contact_point.local_p1.y, (double)contact_point.local_p1.z);
                printf("Found contact distance: %f\n", (double)contact_point.distance); // Negative if there is a penetration.
                printf("Found contact impulse: %f\n", (double)contact_point.impulse);
                printf("Found friction impulse: (%f, %f)\n", (double)contact_point.tangent_impulse[0],
                       (double)contact_point.tangent_impulse[1]);
            }

            // Read the solver contacts.
            size_t num_solver_contacts = r3SolverContacts(collider_handle1, collider_handle2, i, NULL, 0);
            R3SolverContact *solver_contacts = malloc(num_solver_contacts * sizeof(*solver_contacts));
            num_solver_contacts =
                r3SolverContacts(collider_handle1, collider_handle2, i, solver_contacts, num_solver_contacts);
            for (size_t j = 0; j < num_solver_contacts; j++) {
                // Solver contacts are anchored in the local-space of the body they touch, so
                // they ride rigidly with it. `r3SolverContacts` resolves them through the bodies'
                // current poses to give the world-space contact point on each body's surface.
                R3SolverContact solver_contact = solver_contacts[j];
                printf("Found solver contact points: (%f, %f, %f), (%f, %f, %f)\n",
                       (double)solver_contact.point1.x, (double)solver_contact.point1.y,
                       (double)solver_contact.point1.z, (double)solver_contact.point2.x,
                       (double)solver_contact.point2.y, (double)solver_contact.point2.z);
                // The solver contact distance is negative if there is a penetration.
                printf("Found solver contact distance: %f\n", (double)solver_contact.distance);
            }
            free(solver_contacts);
        }
        free(points);
        free(manifolds);
    }
    // DOCUSAURUS: ContactGraph1 stop

    // DOCUSAURUS: ContactGraph2 start
    /* Iterate through all the contact pairs involving a specific collider. */
    size_t num_pairs = r3Collider_ContactPairs(collider_handle1, NULL, 0);
    R3ContactPair *pairs = malloc(num_pairs * sizeof(*pairs));
    num_pairs = r3Collider_ContactPairs(collider_handle1, pairs, num_pairs);
    for (size_t i = 0; i < num_pairs; i++) {
        R3ColliderHandle other_collider =
            same_collider(pairs[i].collider1, collider_handle1) ? pairs[i].collider2 : pairs[i].collider1;

        // Process the contact pair in a way similar to what we did in
        // the previous example.
        (void)other_collider;
    }
    free(pairs);
    // DOCUSAURUS: ContactGraph2 stop

    // DOCUSAURUS: IntersectionGraph1 start
    /* Find the intersection pair, if it exists, between two colliders. */
    R3OptionalIntersectionPair intersection_pair = r3TryIntersectionPair(collider_handle1, collider_handle2);
    if (intersection_pair.found && intersection_pair.pair.intersecting) {
        printf("The colliders %u and %u are intersecting!\n", collider_handle1.index, collider_handle2.index);
    }
    // DOCUSAURUS: IntersectionGraph1 stop

    // DOCUSAURUS: IntersectionGraph2 start
    /* Iterate through all the intersection pairs involving a specific collider. */
    size_t num_intersections = r3Collider_IntersectionPairs(collider_handle1, NULL, 0);
    R3IntersectionPair *intersections = malloc(num_intersections * sizeof(*intersections));
    num_intersections = r3Collider_IntersectionPairs(collider_handle1, intersections, num_intersections);
    for (size_t i = 0; i < num_intersections; i++) {
        if (intersections[i].intersecting) {
            printf("The colliders %u and %u are intersecting!\n", intersections[i].collider1.index,
                   intersections[i].collider2.index);
        }
    }
    free(intersections);
    // DOCUSAURUS: IntersectionGraph2 stop

    /* Run the physics hooks. */
    R3UserData even = {2, 0};
    r3Collider_SetUserData(collider_handle1, even);
    r3Collider_SetUserData(collider_handle2, even);
    r3Collider_SetActiveHooks(collider_handle2, R3_FILTER_CONTACT_PAIRS | R3_MODIFY_SOLVER_CONTACTS);
    r3RigidBody_WakeUp(ball_body_handle, 1);
    for (int i = 0; i < 10; i++) {
        step_with_pair_filters(world);
    }
    for (int i = 0; i < 10; i++) {
        step_with_contact_modification(world);
    }
    R3PhysicsHooks per_contact_hooks = {0};
    per_contact_hooks.modify_solver_contacts_context = modify_each_solver_contact;
    r3Step(world, &per_contact_hooks, NULL);

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
