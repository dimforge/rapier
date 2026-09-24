/* Port of examples3d/collision_groups3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbCollisionGroups3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle floor;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(5, 0.1, 5));
    rigidBody.canSleep = !testbed->noSleep;
    floor = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(floor, &boxCollider);
    for (int i = 1; i <= 2; i++) {
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 0.1, 1));
        collider.position.translation = r3Vector(0, i, 0);
        collider.collisionGroups = (R3InteractionGroups){(uint32_t)i, (uint32_t)i, R3_GROUPS_AND};
        R3ColliderHandle colliderHandle = r3InsertCollider(floor, &collider);
        tbColliderColor(testbed, colliderHandle, 0, i == 2, i == 1, 1);
    }
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                uint32_t group = k % 2 ? 2 : 1;
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 0.1, 0.1));
                collider.collisionGroups = (R3InteractionGroups){group, group, R3_GROUPS_AND};
                R3RigidBodyHandle handle;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(i * 0.2 - 0.8, j * 0.2 + 2.5, k * 0.2 - 0.8);
                rigidBody.canSleep = !testbed->noSleep;
                handle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(handle, &collider);
                tbBodyColor(testbed, handle, 0, group == 1, group == 2, 1);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
