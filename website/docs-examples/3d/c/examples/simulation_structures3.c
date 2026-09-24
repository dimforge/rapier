#include "snippets.h"

/* Accepts every contact pair, i.e., behaves like the default filter. */
static int32_t RAPIER_CALL filter_contact_pair(void *user_data, const R3ReadContext *read,
                                               R3ColliderHandle collider1, R3ColliderHandle collider2,
                                               R3RigidBodyHandle body1, R3RigidBodyHandle body2) {
    (void)user_data;
    (void)read;
    (void)collider1;
    (void)collider2;
    (void)body1;
    (void)body2;
    return 1;
}

int main(void) {
    snippets_init();

    // DOCUSAURUS: World start
    /* The world owns every structure needed by the simulation. */
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0.0, -9.81, 0.0));
    r3SetTimeStep(world, 1.0 / 60.0);

    /* Create the ground: a collider without any parent rigid-body. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball: a rigid-body, then its collider attached to it. */
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle ball_handle = r3InsertRigidBody(world, &ball_body);
    R3ColliderDesc ball_collider = r3BallColliderDesc(0.5);
    ball_collider.restitution = 0.7;
    r3InsertCollider(ball_handle, &ball_collider);

    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        r3Step(world, NULL, NULL);
        printf("Ball altitude: %f\n", (double)r3RigidBody_Translation(ball_handle).y);
    }
    // DOCUSAURUS: World stop

    {
        // DOCUSAURUS: WorldQueries start
        /* The scene queries are functions of the world. */
        R3OptionalRayHit hit =
            r3TryCastRay(world, NULL, r3Vector(0.0, 10.0, 0.0), r3Vector(0.0, -1.0, 0.0), 100.0, 1);
        if (hit.found) {
            printf("Collider {%u, %u} hit at distance %f\n", hit.hit.collider.index,
                   hit.hit.collider.generation, (double)hit.hit.time_of_impact);
        }

        /* So is everything else the world contains, e.g., its contact pairs. */
        size_t num_pairs = r3ContactPairs(world, NULL, 0);
        printf("%zu contact pairs\n", num_pairs);
        // DOCUSAURUS: WorldQueries stop
    }

    {
        // DOCUSAURUS: Step start
        /* The events of every timestep it is given to are accumulated by the collector. */
        R3EventCollector *events = r3NewEventCollector();
        /* The unset callbacks (NULL) keep the default behavior. */
        R3PhysicsHooks hooks = {0};
        hooks.filter_contact_pair = filter_contact_pair;

        for (int i = 0; i < 10; i++) {
            r3Step(world, &hooks, events);
        }

        size_t num_events = r3EventCollector_CollisionEvents(events, NULL, 0);
        printf("%zu collision events during the last 10 steps\n", num_events);
        /* Discard the events once they are processed. */
        r3EventCollector_Clear(events);
        r3FreeEventCollector(events);
        // DOCUSAURUS: Step stop
    }

    {
        // DOCUSAURUS: DetectCollisions start
        /* Teleport the ball, then update the contacts and the scene queries without moving anything. */
        r3RigidBody_SetTranslation(ball_handle, r3Vector(0.0, 0.4, 0.0), 1);
        r3DetectCollisions(world, NULL, NULL);

        R3OptionalRayHit hit =
            r3TryCastRay(world, NULL, r3Vector(0.0, 10.0, 0.0), r3Vector(0.0, -1.0, 0.0), 100.0, 1);
        printf("The ray now hits the ball at distance %f\n", (double)hit.hit.time_of_impact);
        // DOCUSAURUS: DetectCollisions stop
    }

    {
        // DOCUSAURUS: Handles start
        printf("%zu rigid-bodies, %zu colliders, %zu soft-bodies, %zu impulse joints, %zu multibody joints\n",
               r3RigidBodyCount(world), r3ColliderCount(world), r3SoftBodyCount(world),
               r3ImpulseJointCount(world), r3MultibodyJointCount(world));

        /* Ask for the number of handles, then copy them into a large enough buffer. */
        size_t num_colliders = r3ColliderHandles(world, NULL, 0);
        R3ColliderHandle *colliders = malloc(num_colliders * sizeof(R3ColliderHandle));
        num_colliders = r3ColliderHandles(world, colliders, num_colliders);

        for (size_t i = 0; i < num_colliders; i++) {
            R3Vector translation = r3Collider_Translation(colliders[i]);
            printf("Collider {%u, %u} is at altitude %f\n", colliders[i].index, colliders[i].generation,
                   (double)translation.y);
        }
        free(colliders);
        // DOCUSAURUS: Handles stop
    }

    {
        // DOCUSAURUS: UserData start
        /* Any 128-bit value, e.g., the identifier of the game object owning the rigid-body. */
        R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
        body.userData.low = 42;
        R3RigidBodyHandle body_handle = r3InsertRigidBody(world, &body);

        /* It can be modified and read at any time. */
        R3UserData user_data = {43, 0};
        r3RigidBody_SetUserData(body_handle, user_data);
        printf("Game object: %llu\n", (unsigned long long)r3RigidBody_UserData(body_handle).low);
        // DOCUSAURUS: UserData stop
    }

    {
        // DOCUSAURUS: Removal start
        /* Remove the ball's rigid-body, together with its colliders (its joints are always removed). */
        r3RemoveRigidBody(ball_handle, 1);

        /* The handle of a removed object is stale: every function using it fails. */
        if (!r3RigidBody_Contains(ball_handle)) {
            printf("The ball is no longer part of the world.\n");
        }
        // DOCUSAURUS: Removal stop
    }

    {
        // DOCUSAURUS: ThreadPool start
        /* Only supported if the library was built with -DRAPIER_ENABLE_PARALLEL=ON. */
        if (r3BuildFeatures().parallel) {
            /* The world gets its own pool of 4 threads, used from the next timestep. */
            r3SetNumThreads(world, 4);
        }
        // DOCUSAURUS: ThreadPool stop
    }

    {
        // DOCUSAURUS: StepTime start
        /* Only supported if the library was built with the `profiler` feature. */
        if (r3BuildFeatures().profiling) {
            r3SetCountersEnabled(world, 1);
            r3Step(world, NULL, NULL);
            printf("The last timestep took %f ms\n", r3StepTimeMs(world));
        }
        // DOCUSAURUS: StepTime stop
    }

    // DOCUSAURUS: FreeWorld start
    /* Frees the world and everything it contains: every handle of this world becomes invalid. */
    r3FreeWorld(world);
    // DOCUSAURUS: FreeWorld stop
    return EXIT_SUCCESS;
}
