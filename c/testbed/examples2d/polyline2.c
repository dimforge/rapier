/* Port of examples2d/polyline2.rs. */
#include "testbed.h"
#include "rapier_math.h"
#include "rapier_helpers.h"

void tbPolyline2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2Vector points[2000];
    R2Edge edges[1999];
    points[0] = r2Vector(-25, 40);
    points[1999] = r2Vector(25, 40);
    for (int i = 1; i < 1999; i++) {
        points[i] = r2Vector(-25 + i * 0.025, cos(i * 0.025) * 2);
    }
    for (uint32_t i = 0; i < 1999; i++) {
        edges[i] = (R2Edge){i, i + 1};
    }
    R2RigidBodyDesc ground = r2FixedRigidBodyDesc();
    ground.canSleep = !testbed->noSleep;
    R2ColliderDesc groundCollider = r2DefaultColliderDesc();
    r2ShapeDesc_SetPolyline(&groundCollider.shape, (R2VectorView){points, TB_COUNT(points)},
                           (R2EdgeView){edges, TB_COUNT(edges)}, 0);
    R2RigidBodyHandle groundHandle = r2InsertRigidBody(world, &ground);
    r2InsertCollider(groundHandle, &groundCollider);
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 20; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i - 10, j + 3.5);
            R2ColliderDesc collider;
            if (j % 2) {
                collider = r2BallColliderDesc(0.5);
            } else {
                collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            }
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 10);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
