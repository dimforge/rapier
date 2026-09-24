/* Port of examples3d/keva3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

/* Corresponds to keva3::buildBlock, shared by the Keva and cloth examples. */
void buildBlock(Testbed *testbed, R3World *world, R3Vector halfExtents, R3Vector shift, int numx,
                int numy, int numz) {
    const R3Vector dimensions[] = {
        halfExtents,
        r3Vector(halfExtents.z, halfExtents.y, halfExtents.x),
    };
    const R3Real blockWidth = 2.0 * halfExtents.z * numx;
    const R3Real blockHeight = 2.0 * halfExtents.y * numy;
    const R3Real spacing = (halfExtents.z * numx - halfExtents.x) / (numz - 1);

    for (int i = 0; i < numy; i++) {
        const int oldNumx = numx;
        numx = numz;
        numz = oldNumx;
        const R3Vector dim = dimensions[i % 2];
        const R3Real y = dim.y * i * 2.0;

        for (int j = 0; j < numx; j++) {
            const R3Real x = i % 2 == 0 ? spacing * j * 2.0 : dim.x * j * 2.0;
            for (int k = 0; k < numz; k++) {
                const R3Real z = i % 2 == 0 ? dim.z * k * 2.0 : spacing * k * 2.0;

                /* Build the rigid body. */
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation =
                    r3Vector(x + dim.x + shift.x, y + dim.y + shift.y, z + dim.z + shift.z);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3CuboidColliderDesc(dim);
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }

    /* Close the top. */
    const R3Vector dim = r3Vector(halfExtents.z, halfExtents.x, halfExtents.y);
    for (int i = 0; i < (int)(blockWidth / (dim.x * 2.0)); i++) {
        for (int j = 0; j < (int)(blockWidth / (dim.z * 2.0)); j++) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation =
                r3Vector(i * dim.x * 2.0 + dim.x + shift.x, dim.y + blockHeight + shift.y,
                         j * dim.z * 2.0 + dim.z + shift.z);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(dim);
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        }
    }
}

void tbKeva3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    /* Ground. */
    const R3Real groundSize = 50.0;
    const R3Real groundHeight = 0.1;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0.0, -groundHeight, 0.0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(groundSize, groundHeight, groundSize));
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    /* Create the cubes. Odd layer counts keep adjacent blocks aligned. */
    const R3Vector halfExtents = r3Vector(0.1, 0.5, 2.0);
    R3Real blockHeight = 0.0;
    const int layers[] = {0, 9, 13, 17, 21, 41};

    for (int i = 5; i >= 1; i--) {
        const int numx = i;
        const int numy = layers[i];
        const int numz = numx * 3 + 1;
        const R3Real blockWidth = numx * halfExtents.z * 2.0;
        buildBlock(testbed, world, halfExtents,
                   r3Vector(-blockWidth / 2.0, blockHeight, -blockWidth / 2.0), numx, numy, numz);
        blockHeight += numy * halfExtents.y * 2.0 + halfExtents.x * 2.0;
    }

    /* Set up the viewer. */
    tbCamera(testbed, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
