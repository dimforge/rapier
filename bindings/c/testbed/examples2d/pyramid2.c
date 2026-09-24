/* Port of examples2d/pyramid2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbPyramid2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    const R2Real groundThickness = 1;
    const R2Real rad = 0.5;
    const R2Real shift = rad * 2;
    const int num = 10;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(10, groundThickness));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
    const R2Real centerx = shift * num / 2;
    const R2Real centery = shift / 2 + groundThickness + rad * 1.5;
    for (int i = 0; i < num; i++) {
        for (int j = i; j < num; j++) {
            R2Real x = i * shift / 2 + (j - i) * shift - centerx;
            R2Real y = i * shift + centery;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, y);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(rad, rad));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
