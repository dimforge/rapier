/* Port of examples3d/sensor3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static int same(R3RigidBodyHandle handle, R3RigidBodyHandle handleB) {
    return handle.world == handleB.world && handle.index == handleB.index &&
           handle.generation == handleB.generation;
}

void tbSensor3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle ground = {0};
    R3RigidBodyHandle sensor = {0};
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(10.1, 0.1, 10.1));
    rigidBody.canSleep = !testbed->noSleep;
    ground = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(ground, &boxCollider);

    for (int i = 0; i < 10; i++) {
        for (int k = 0; k < 10; k++) {
            R3RigidBodyHandle handle;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(i * 0.4 - 2, 3, k * 0.4 - 2);
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
            rigidBody.canSleep = !testbed->noSleep;
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);

            tbBodyColor(testbed, handle, 0.5, 0.5, 1, 1);
        }
    }
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(0, 5, 0);
    R3ColliderDesc sensorBodyCollider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
    dynamicBody.canSleep = !testbed->noSleep;
    sensor = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(sensor, &sensorBodyCollider);

    R3ColliderDesc collider = r3BallColliderDesc(1);
    collider.density = 0;
    collider.isSensor = 1;
    collider.activeEvents = R3_COLLISION_EVENTS;
    r3InsertCollider(sensor, &collider);
    tbBodyColor(testbed, sensor, 0.5, 1, 1, 1);
    /* Set up the viewer. */
    tbCamera(testbed, 6, 4, 6, 0, 1, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;
    R3EventCollector *eventHandler = r3NewEventCollector();

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3EventCollector_Clear(eventHandler);
            r3Step(world, NULL, eventHandler);

            size_t n = r3EventCollector_CollisionEvents(eventHandler, NULL, 0);
            R3CollisionEvent *events = calloc(n ? n : 1, sizeof(*events));
            if (!events) {
                abort();
            }
            n = r3EventCollector_CollisionEvents(eventHandler, events, n);
            for (size_t i = 0; i < n; i++) {
                R3ColliderHandle colliderHandles[] = {events[i].collider1, events[i].collider2};
                for (size_t j = 0; j < 2; j++) {
                    R3RigidBodyHandle handle = r3Collider_Parent(colliderHandles[j]);
                    if (!same(handle, ground) && !same(handle, sensor)) {
                        tbBodyColor(testbed, handle, events[i].started ? 1 : 0.5f,
                                    events[i].started ? 1 : 0.5f, events[i].started ? 0 : 1, 1);
                    }
                }
            }
            free(events);
        }
    }
    r3FreeEventCollector(eventHandler);
    r3FreeWorld(world);
}
