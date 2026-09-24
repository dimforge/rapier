/* Port of examples3d/damping3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDamping3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, 0, 0));
    const int num = 10;
    const R3Real subdiv = (R3Real)1 / num;
    for (int i = 0; i < num; i++) {
        R3Real x = (R3Real)sin(i * subdiv * R3_PI * 2);
        R3Real y = (R3Real)cos(i * subdiv * R3_PI * 2);
        R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
        body.position.translation = r3Vector(x, y, 0);
        body.linvel = r3Vector(x * 10, y * 10, 0);
        body.angvel = r3Vector(0, 0, 100);
        body.linearDamping = (i + 1) * subdiv * 10;
        body.angularDamping = (num - i) * subdiv * 10;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
        body.canSleep = !testbed->noSleep;
        R3RigidBodyHandle bodyHandle = r3InsertRigidBody(world, &body);
        r3InsertCollider(bodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 2, 2.5, 20, 2, 2.5, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
