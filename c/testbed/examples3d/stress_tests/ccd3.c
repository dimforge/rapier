/* Port of examples3d/stress_tests/ccd3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsCcd3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(100.1, 0.1, 100.1));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);

    R3Real offset = -6;
    for (int j = 0; j < 20; j++, offset -= 0.15) {
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 3 - 6 + offset, j * 3 + 4.5, k * 3 - 6 + offset);
                rigidBody.linvel = r3Vector(0, -1000, 0);
                rigidBody.ccdEnabled = 1;
                R3ColliderDesc collider;
                switch (j % 5) {
                case 0:
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    break;
                case 1:
                    collider = r3BallColliderDesc(1);
                    break;
                case 2:
                    collider = r3RoundCylinderColliderDesc(1, 1, 0.1);
                    break;
                case 3:
                    collider = r3ConeColliderDesc(1, 1);
                    break;
                default:
                    collider = r3CapsuleYColliderDesc(1, 1);
                    break;
                }
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
