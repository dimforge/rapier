/* Port of examples3d/debug_internal_edges3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugInternalEdges3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    R3Real *heights = calloc(100 * 100, sizeof(*heights));
    if (!heights) {
        abort();
    }
    R3ColliderDesc heightfield = r3DefaultColliderDesc();
    heightfield.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    heightfield.shape.heights = (R3RealView){heights, (100) * (100)};
    heightfield.shape.rows = 100;
    heightfield.shape.columns = 100;
    heightfield.shape.scale = r3Vector(60, 1, 60);
    heightfield.shape.flags = R3_HEIGHTFIELD_FIX_INTERNAL_EDGES;
    r3InsertColliderWithoutParent(world, &heightfield);

    free(heights);
    /* Dynamic rigid bodies. */
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(4, 0.5, 0);
        rigidBody.linvel = r3Vector(0, -40, 20);
        rigidBody.canSleep = 0;
        R3ColliderDesc collider = r3BallColliderDesc(.5);
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-3, 5, 0);
        rigidBody.linvel = r3Vector(0, -4, 20);
        rigidBody.canSleep = 0;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(8, 0.2, 0);
        rigidBody.linvel = r3Vector(0, -4, 20);
        rigidBody.canSleep = 0;
        R3ColliderDesc collider = r3CylinderColliderDesc(.5, .2);
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI / 2);
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
