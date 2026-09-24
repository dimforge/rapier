/* Port of examples3d/debug_thin_cube_on_mesh3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugThinCubeOnMesh3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    R3Real *heights = calloc(2 * 2, sizeof(*heights));
    if (!heights) {
        abort();
    }
    R3ColliderDesc heightfield = r3DefaultColliderDesc();
    heightfield.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    heightfield.shape.heights = (R3RealView){heights, (2) * (2)};
    heightfield.shape.rows = 2;
    heightfield.shape.columns = 2;
    heightfield.shape.scale = r3Vector(50, 1, 50);
    heightfield.shape.flags = R3_HEIGHTFIELD_FIX_INTERNAL_EDGES;
    r3InsertColliderWithoutParent(world, &heightfield);

    free(heights);
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 5, 0);
        rigidBody.position.rotation = r3RotationFromAxisAngle(r3Vector(.5, 0, .5), sqrt(.5));
        rigidBody.linvel = r3Vector(0, -100, 0);
        rigidBody.softCcdPrediction = 10;
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(5, 0.015, 5));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
