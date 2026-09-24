/* Port of examples3d/ccd3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static int same(R3RigidBodyHandle handle, R3RigidBodyHandle handleB) {
    return handle.world == handleB.world && handle.index == handleB.index &&
           handle.generation == handleB.generation;
}

void tbCcd3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle ground = {0};
    R3RigidBodyHandle sensor = {0};
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(50, 0.1, 50));
    groundBody.canSleep = !testbed->noSleep;
    ground = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(ground, &boxCollider);

    for (int wall = 0; wall < 5; wall++) {
        for (int row = 0; row < 2; row++) {
            int k = 0;
            for (int i = 0; i < 8; i++) {
                for (int j = i; j < 8; j++) {
                    R3RigidBodyHandle handle;
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r3Vector(wall * 6, i + 0.6, i + (j - i) * 2 + row * 20 - 8);
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 1));
                    rigidBody.canSleep = !testbed->noSleep;
                    handle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(handle, &collider);

                    k++;
                    tbBodyColor(testbed, handle, k % 2 ? 131.0f / 255 : 1, k % 2 ? 1 : 131.0f / 255,
                                244.0f / 255, 1);
                }
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-20, 2.6, i * 20);
        rigidBody.linvel = r3Vector(1000, 0, 0);
        rigidBody.ccdEnabled = 1;
        R3ColliderDesc collider = r3BallColliderDesc(1);
        collider.density = 10;
        if (!i) {
            collider.isSensor = 1;
            collider.activeEvents = R3_COLLISION_EVENTS;
        }
        R3RigidBodyHandle handleH;
        rigidBody.canSleep = !testbed->noSleep;
        handleH = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handleH, &collider);
        if (!i) {
            sensor = handleH;
        } else {
            tbBodyColor(testbed, handleH, 0.2, 0.2, 1, 1);
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

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
