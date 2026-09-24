/* Port of examples2d/soft_blobs2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSoftBlobs2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.5);
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(6, 0.5));
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
    for (int side = -1; side <= 1; side += 2) {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(side * 6, 6);
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.5, 6));
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    for (int j = 0; j < 15; j++) {
        for (int i = 0; i < 5; i++) {
            R2SoftBodyDesc softBody = r2DiskSoftBodyDesc(
                r2Vector(-4 + i * 2 + j % 2 * 0.5, 3 + j * 2), 0.45 + 0.1 * ((i + j) % 3), 20);
            softBody.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){20, 1});
            softBody.volumeFactor = 1.05;
            softBody.selfContacts = 1;
            softBody.particleMass = 0.05;
            if (testbed->noSleep) {
                softBody.canSleep = 0;
            }
            r2InsertSoftBody(world, &softBody);
        }
    }
    R2SoftBodyDesc strip = r2GridSoftBodyDesc(r2Vector(0, 16), r2Vector(3, 0.15), 40, 3);
    strip.material = r2UniformSoftBodyMaterial((R2SpringCoefficients){30, 1});
    strip.selfContacts = 1;
    strip.particleMass = 0.02;
    if (testbed->noSleep) {
        strip.canSleep = 0;
    }
    r2InsertSoftBody(world, &strip);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 6, 30);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
