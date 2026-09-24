/* Port of examples3d/primitives3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbPrimitives3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    /* Ground. */
    const R3Real groundSize = 100.1;
    const R3Real groundHeight = 2.1;

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0.0, -groundHeight, 0.0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(groundSize, groundHeight, groundSize));
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    /* Create the primitives. */
    const int num = 8;
    const R3Real rad = 1.0;

    const R3Real shiftx = rad * 2.0 + rad;
    const R3Real shifty = rad * 2.0 + rad;
    const R3Real shiftz = rad * 2.0 + rad;
    const R3Real centerx = shiftx * (num / 2);
    const R3Real centery = shifty / 2.0;
    const R3Real centerz = shiftz * (num / 2);

    R3Real offset = -num * (rad * 2.0 + rad) * 0.5;

    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < num; i++) {
            for (int k = 0; k < num; k++) {
                const R3Real x = i * shiftx - centerx + offset;
                const R3Real y = j * shifty + centery + 3.0;
                const R3Real z = k * shiftz - centerz + offset;

                /* Build the rigid body. */
                rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(x, y, z);
                rigidBody.canSleep = !testbed->noSleep;

                switch (j % 5) {
                case 1:
                    collider = r3BallColliderDesc(rad);
                    break;
                case 2:
                    /* Rounded cylinders are faster even with a small rounding margin. */
                    collider = r3RoundCylinderColliderDesc(rad, rad, rad / 10.0);
                    break;
                case 3:
                    collider = r3ConeColliderDesc(rad, rad);
                    break;
                default:
                    collider = r3CapsuleYColliderDesc(rad, rad);
                    break;
                }

                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }

        offset -= 0.05 * rad * (num - 1.0);
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
