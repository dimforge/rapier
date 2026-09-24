/* Port of examples2d/damping2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDamping2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, 0));
    const int num = 10;
    const R2Real subdiv = (R2Real)1 / num;
    for (int i = 0; i < num; i++) {
        R2Real x = (R2Real)sin(i * subdiv * R2_PI * 2);
        R2Real y = (R2Real)cos(i * subdiv * R2_PI * 2);
        R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
        body.position.translation = r2Vector(x, y);
        body.linvel = r2Vector(x * 10, y * 10);
        body.angvel = 100;
        body.linearDamping = (i + 1) * subdiv * 10;
        body.angularDamping = (num - i) * subdiv * 10;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.2, 0.2));
        body.canSleep = !testbed->noSleep;
        R2RigidBodyHandle bodyHandle = r2InsertRigidBody(world, &body);
        r2InsertCollider(bodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 3, 2, 50);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
