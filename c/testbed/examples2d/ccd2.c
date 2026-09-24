/* Port of examples2d/ccd2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static int same(R2RigidBodyHandle handle, R2RigidBodyHandle handleB) {
    return handle.world == handleB.world && handle.index == handleB.index &&
           handle.generation == handleB.generation;
}

void tbCcd2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle ground = {0};
    R2RigidBodyHandle sensor = {0};
    R2RigidBodyDesc floor = r2FixedRigidBodyDesc();
    floor.position.translation = r2Vector(0, 0);
    floor.ccdEnabled = 1;
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(25, 0.1));
    floor.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &floor);
    r2InsertCollider(ground, &boxCollider);

    sensor = ground;
    const R2Real x[] = {-3, 6, 2.5};
    for (int i = 0; i < 3; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 25));
        collider.position.translation = r2Vector(x[i], 0);
        if (i == 2) {
            collider.isSensor = 1;
            collider.activeEvents = R2_COLLISION_EVENTS;
        }
        r2InsertCollider(ground, &collider);
    }
    R2Pose poses[] = {r2TranslationPose(r2Vector(0, 0.35)), r2TranslationPose(r2Vector(-0.35, 0)),
                      r2TranslationPose(r2Vector(0.35, 0))};
    R2SharedShape *shapes[3] = {NULL};
    shapes[0] = r2CuboidSharedShape(r2Vector(0.4, 0.05));
    shapes[1] = r2CuboidSharedShape(r2Vector(0.05, 0.4));
    shapes[2] = r2CuboidSharedShape(r2Vector(0.05, 0.4));
    R2ColliderDesc shape;
    R2CompoundShapeDesc shapeParts[3];
    for (size_t part = 0; part < 3; ++part) {
        shapeParts[part].pose = poses[part];
        shapeParts[part].shape = r2DefaultShapeDesc();
        shapeParts[part].shape.kind = R2_SHAPE_DESC_SHARED;
        shapeParts[part].shape.sharedShape = ((const R2SharedShape *const *)shapes)[part];
    }
    shape = r2DefaultColliderDesc();
    shape.shape.kind = R2_SHAPE_DESC_COMPOUND;
    shape.shape.children = (R2CompoundShapeView){shapeParts, 3};

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 0.82 - 1.96, j * 0.82 + 4.41);
            rigidBody.linvel = r2Vector(100, -10);
            rigidBody.ccdEnabled = 1;
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &shape);
        }
    }
    for (size_t part = 0; part < 3; part++) {
        r2FreeSharedShape(shapes[part]);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

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
