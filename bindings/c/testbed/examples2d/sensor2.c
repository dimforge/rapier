/* Port of examples2d/sensor2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static int same(R2RigidBodyHandle handle, R2RigidBodyHandle handleB) {
    return handle.world == handleB.world && handle.index == handleB.index &&
           handle.generation == handleB.generation;
}

void tbSensor2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle ground = {0};
    R2RigidBodyHandle sensor = {0};
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.1);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(200.1, 0.1));
    rigidBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(ground, &boxCollider);

    for (int i = 0; i < 10; i++) {
        R2RigidBodyHandle handle;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(i * 0.4 - 2, 3);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
        rigidBody.canSleep = !testbed->noSleep;
        handle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(handle, &collider);

        tbBodyColor(testbed, handle, 0.5, 0.5, 1, 1);
    }
    R2RigidBodyDesc dynamicBody = r2DynamicRigidBodyDesc();
    dynamicBody.position.translation = r2Vector(0, 10);
    R2ColliderDesc sensorBodyCollider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
    dynamicBody.canSleep = !testbed->noSleep;
    sensor = r2InsertRigidBody(world, &dynamicBody);
    r2InsertCollider(sensor, &sensorBodyCollider);

    R2ColliderDesc collider = r2BallColliderDesc(1);
    collider.density = 0;
    collider.isSensor = 1;
    collider.activeEvents = R2_COLLISION_EVENTS;
    r2InsertCollider(sensor, &collider);
    tbBodyColor(testbed, sensor, 0.5, 1, 1, 1);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 1, 100);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;
    R2EventCollector *eventHandler = r2NewEventCollector();

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2EventCollector_Clear(eventHandler);
            r2Step(world, NULL, eventHandler);

            size_t n = r2EventCollector_CollisionEvents(eventHandler, NULL, 0);
            R2CollisionEvent *events = calloc(n ? n : 1, sizeof(*events));
            if (!events) {
                abort();
            }
            n = r2EventCollector_CollisionEvents(eventHandler, events, n);
            for (size_t i = 0; i < n; i++) {
                R2ColliderHandle colliderHandles[] = {events[i].collider1, events[i].collider2};
                for (size_t j = 0; j < 2; j++) {
                    R2RigidBodyHandle handle = r2Collider_Parent(colliderHandles[j]);
                    if (!same(handle, ground) && !same(handle, sensor)) {
                        tbBodyColor(testbed, handle, events[i].started ? 1 : 0.5f,
                                    events[i].started ? 1 : 0.5f, events[i].started ? 0 : 1, 1);
                    }
                }
            }
            free(events);
        }
    }
    r2FreeEventCollector(eventHandler);
    r2FreeWorld(world);
}
