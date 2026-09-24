/* Port of examples2d/collision_groups2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbCollisionGroups2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle floor;
    R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, -0.1);
    R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(5, 0.1));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r2InsertRigidBody(world, &rigidBody);
    r2InsertCollider(floor, &boxCollider);
    for (int i = 1; i <= 2; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1, 0.1));
        collider.position.translation = r2Vector(0, i);
        collider.collisionGroups = (R2InteractionGroups){(uint32_t)i, (uint32_t)i, R2_GROUPS_AND};
        R2ColliderHandle colliderHandle = r2InsertCollider(floor, &collider);
        tbColliderColor(testbed, colliderHandle, 0, i == 1, i == 2, 1);
    }
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 8; i++) {
            uint32_t group = i % 2 ? 2 : 1;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 0.1));
            collider.collisionGroups = (R2InteractionGroups){group, group, R2_GROUPS_AND};
            R2RigidBodyHandle handle;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(i * 0.2 - 0.8, j * 0.2 + 2.5);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);
            tbBodyColor(testbed, handle, 0, group == 1, group == 2, 1);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 1, 100);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
