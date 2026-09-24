/* Port of examples2d/s2d_card_house.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static void card(Testbed *testbed, R2World *world, R2Vector position, R2Real angle) {
    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
    rigidBody.position.translation = position;
    rigidBody.position = r2Pose(position, r2Rotation(angle));
    R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.01, 2));
    collider.friction = 0.7;
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &collider);
}

void tbS2dCardHouse(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2ColliderDesc floor = r2CuboidColliderDesc(r2Vector(40, 2));
    floor.friction = 0.7;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -2);
    rigidBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(rigidBodyHandle, &floor);
    R2Real z0 = 0;
    R2Real y = 1.8;
    for (int nb = 5; nb; nb--) {
        R2Real z = z0;
        for (int i = 0; i < nb; i++) {
            if (i != nb - 1) {
                card(testbed, world, r2Vector(z + 2.5, y + 1.85), R2_PI / 2);
            }
            card(testbed, world, r2Vector(z, y), -25 * R2_PI / 180);
            z += 1.75;
            card(testbed, world, r2Vector(z, y), 25 * R2_PI / 180);
            z += 1.75;
        }
        y += 3.7;
        z0 += 1.75;
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
