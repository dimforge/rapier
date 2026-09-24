/* Port of examples3d/debug_trimesh3.rs. */
#include "testbed.h"
#include "rapier_math.h"
#include "rapier_helpers.h"

void tbDebugTrimesh3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3Vector vertices[] = {r3Vector(-0.5, 0, -0.5),    r3Vector(0.5, 0, -0.5),
                           r3Vector(0.5, 0, 0.5),      r3Vector(-0.5, 0, 0.5),
                           r3Vector(-0.5, -0.5, -0.5), r3Vector(0.5, -0.5, -0.5),
                           r3Vector(0.5, -0.5, 0.5),   r3Vector(-0.5, -0.5, 0.5)};
    R3Triangle triangles[] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 4, 7}, {0, 7, 3},
                              {1, 6, 5}, {1, 2, 6}, {3, 7, 2}, {2, 7, 6}, {0, 1, 5}, {0, 5, 4}};
    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 35, 0);
    rigidBody.canSleep = 0;
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 2, 1));
    if (testbed->noSleep) {
        rigidBody.canSleep = 0;
        rigidBody.sleeping = 0;
    }
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    R3RigidBodyHandle ground;
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.canSleep = !testbed->noSleep;
    R3ColliderDesc groundCollider = r3DefaultColliderDesc();
    r3ShapeDesc_SetTrimesh(&groundCollider.shape, (R3VectorView){vertices, TB_COUNT(vertices)},
                          (R3TriangleView){triangles, TB_COUNT(triangles)}, 0);
    ground = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(ground, &groundCollider);
    tbBodyColor(testbed, ground, 0.75, 0.75, 0.75, 1);
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
