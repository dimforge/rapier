/* Port of examples2d/s2d_pyramid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dPyramid(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2Vector origin = r2Vector(0, 0);
    R2ColliderDesc floor = r2CuboidColliderDesc(r2Vector(100, 1));
    floor.friction = 0.6;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2VectorAdd(origin, r2Vector(0, -1));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);
    int base = (int)tbSetting(testbed, "# of basis cubes", 100, 2, 200, 1);
    R2Real shift = 0.5;
    for (int i = 0; i < base; i++) {
        for (int j = i; j < base; j++) {
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 0.5));
            collider.friction = 0.6;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r2VectorAdd(origin, r2Vector((i + 1) * shift + 2 * (j - i) * shift - 0.5 * base,
                                             (2 * i + 1) * shift + 0));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, (float)origin.x, (float)origin.y + 2.5f, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
