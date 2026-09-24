#include "snippets.h"

/* Accepts every contact pair, i.e., behaves like the default filter. */
static int32_t RAPIER_CALL filter_contact_pair(void *user_data, const R2ReadContext *read,
                                               R2ColliderHandle collider1, R2ColliderHandle collider2,
                                               R2RigidBodyHandle body1, R2RigidBodyHandle body2) {
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
    R2World *world = r2NewWorld();
    r2SetGravity(world, r2Vector(0.0, -9.81));
    r2SetTimeStep(world, 1.0 / 60.0);

    /* Create the ground: a collider without any parent rigid-body. */
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball: a rigid-body, then its collider attached to it. */
    R2RigidBodyDesc ball_body = r2DynamicRigidBodyDesc();
    ball_body.position.translation = r2Vector(0.0, 10.0);
    R2RigidBodyHandle ball_handle = r2InsertRigidBody(world, &ball_body);
    R2ColliderDesc ball_collider = r2BallColliderDesc(0.5);
    ball_collider.restitution = 0.7;
    r2InsertCollider(ball_handle, &ball_collider);

    /* Run the game loop, stepping the simulation once per frame. */
    for (int i = 0; i < 200; i++) {
        r2Step(world, NULL, NULL);
        printf("Ball altitude: %f\n", (double)r2RigidBody_Translation(ball_handle).y);
    }
    // DOCUSAURUS: World stop

    /* Throw the ball again, so it moves during the next steps. */
    r2RigidBody_SetLinvel(ball_handle, r2Vector(0.0, 5.0), 1);

    for (int i = 0; i < 10; i++) {
        r2Step(world, NULL, NULL);
        // DOCUSAURUS: IslandManager start
        /* Iterate on each rigid-body that moved (dynamic and kinematic). */
        size_t num_active = r2ActiveRigidBodies(world, NULL, 0);
        R2RigidBodyHandle *active = malloc(num_active * sizeof(R2RigidBodyHandle));
        num_active = r2ActiveRigidBodies(world, active, num_active);

        for (size_t k = 0; k < num_active; k++) {
            R2Pose position = r2RigidBody_Position(active[k]);
            printf("Rigid body {%u, %u} has a new position: (%f, %f), %f rad\n", active[k].index,
                   active[k].generation, (double)position.translation.x, (double)position.translation.y,
                   (double)position.rotation.angle);
        }
        free(active);
        // DOCUSAURUS: IslandManager stop
    }

    {
        // DOCUSAURUS: WorldQueries start
        /* The scene queries are functions of the world. */
        R2OptionalRayHit hit =
            r2TryCastRay(world, NULL, r2Vector(0.0, 10.0), r2Vector(0.0, -1.0), 100.0, 1);
        if (hit.found) {
            printf("Collider {%u, %u} hit at distance %f\n", hit.hit.collider.index,
                   hit.hit.collider.generation, (double)hit.hit.time_of_impact);
        }

        /* So is everything else the world contains, e.g., its contact pairs. */
        size_t num_pairs = r2ContactPairs(world, NULL, 0);
        printf("%zu contact pairs\n", num_pairs);
        // DOCUSAURUS: WorldQueries stop
    }

    {
        // DOCUSAURUS: Step start
        /* The events of every timestep it is given to are accumulated by the collector. */
        R2EventCollector *events = r2NewEventCollector();
        /* The unset callbacks (NULL) keep the default behavior. */
        R2PhysicsHooks hooks = {0};
        hooks.filter_contact_pair = filter_contact_pair;

        for (int i = 0; i < 10; i++) {
            r2Step(world, &hooks, events);
        }

        size_t num_events = r2EventCollector_CollisionEvents(events, NULL, 0);
        printf("%zu collision events during the last 10 steps\n", num_events);
        /* Discard the events once they are processed. */
        r2EventCollector_Clear(events);
        r2FreeEventCollector(events);
        // DOCUSAURUS: Step stop
    }

    {
        // DOCUSAURUS: DetectCollisions start
        /* Teleport the ball, then update the contacts and the scene queries without moving anything. */
        r2RigidBody_SetTranslation(ball_handle, r2Vector(0.0, 0.4), 1);
        r2DetectCollisions(world, NULL, NULL);

        R2OptionalRayHit hit =
            r2TryCastRay(world, NULL, r2Vector(0.0, 10.0), r2Vector(0.0, -1.0), 100.0, 1);
        printf("The ray now hits the ball at distance %f\n", (double)hit.hit.time_of_impact);
        // DOCUSAURUS: DetectCollisions stop
    }

    {
        // DOCUSAURUS: Handles start
        printf("%zu rigid-bodies, %zu colliders, %zu soft-bodies, %zu impulse joints, %zu multibody joints\n",
               r2RigidBodyCount(world), r2ColliderCount(world), r2SoftBodyCount(world),
               r2ImpulseJointCount(world), r2MultibodyJointCount(world));

        /* Ask for the number of handles, then copy them into a large enough buffer. */
        size_t num_colliders = r2ColliderHandles(world, NULL, 0);
        R2ColliderHandle *colliders = malloc(num_colliders * sizeof(R2ColliderHandle));
        num_colliders = r2ColliderHandles(world, colliders, num_colliders);

        for (size_t i = 0; i < num_colliders; i++) {
            R2Vector translation = r2Collider_Translation(colliders[i]);
            printf("Collider {%u, %u} is at altitude %f\n", colliders[i].index, colliders[i].generation,
                   (double)translation.y);
        }
        free(colliders);
        // DOCUSAURUS: Handles stop
    }

    {
        // DOCUSAURUS: UserData start
        /* Any 128-bit value, e.g., the identifier of the game object owning the rigid-body. */
        R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
        body.userData.low = 42;
        R2RigidBodyHandle body_handle = r2InsertRigidBody(world, &body);

        /* It can be modified and read at any time. */
        R2UserData user_data = {43, 0};
        r2RigidBody_SetUserData(body_handle, user_data);
        printf("Game object: %llu\n", (unsigned long long)r2RigidBody_UserData(body_handle).low);
        // DOCUSAURUS: UserData stop
    }

    {
        // DOCUSAURUS: Removal start
        /* Remove the ball's rigid-body, together with its colliders (its joints are always removed). */
        r2RemoveRigidBody(ball_handle, 1);

        /* The handle of a removed object is stale: every function using it fails. */
        if (!r2RigidBody_Contains(ball_handle)) {
            printf("The ball is no longer part of the world.\n");
        }
        // DOCUSAURUS: Removal stop
    }

    {
        // DOCUSAURUS: ThreadPool start
        /* Only supported if the library was built with -DRAPIER_ENABLE_PARALLEL=ON. */
        if (r2BuildFeatures().parallel) {
            /* The world gets its own pool of 4 threads, used from the next timestep. */
            r2SetNumThreads(world, 4);
        }
        // DOCUSAURUS: ThreadPool stop
    }

    {
        // DOCUSAURUS: StepTime start
        /* Only supported if the library was built with the `profiler` feature. */
        if (r2BuildFeatures().profiling) {
            r2SetCountersEnabled(world, 1);
            r2Step(world, NULL, NULL);
            printf("The last timestep took %f ms\n", r2StepTimeMs(world));
        }
        // DOCUSAURUS: StepTime stop
    }

    // DOCUSAURUS: FreeWorld start
    /* Frees the world and everything it contains: every handle of this world becomes invalid. */
    r2FreeWorld(world);
    // DOCUSAURUS: FreeWorld stop
    return EXIT_SUCCESS;
}
