/* Port of examples3d/b3d_washer.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB3dWasher(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -10, 0));
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(60, 1, 60));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);

    R3RigidBodyDesc rigidBody = r3KinematicVelocityBasedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 21, 0);
    rigidBody.angvel = r3Vector(0, 0, R3_PI / 180 * 25);
    rigidBody.linvel = r3Vector(0.001, -0.002, 0);
    R3RigidBodyHandle washer;
    rigidBody.canSleep = !testbed->noSleep;
    washer = r3InsertRigidBody(world, &rigidBody);
    R3Real angle = R3_PI / 18;
    R3Vector u1 = r3Vector(1, 0, 0);
    for (int i = 0; i < 36; i++) {
        R3Vector u2 = i == 35 ? r3Vector(1, 0, 0)
                              : r3Vector(cos(angle) * u1.x - sin(angle) * u1.y,
                                         sin(angle) * u1.x + cos(angle) * u1.y, 0);
        R3Vector a1 = r3Vector(cos(angle * 0.1) * u1.x + sin(angle * 0.1) * u1.y,
                               -sin(angle * 0.1) * u1.x + cos(angle * 0.1) * u1.y, 0);
        R3Vector a2 = r3Vector(cos(angle * 0.1) * u2.x - sin(angle * 0.1) * u2.y,
                               sin(angle * 0.1) * u2.x + cos(angle * 0.1) * u2.y, 0);
        for (int part = 0; part < (i % 9 == 0 ? 2 : 1); part++) {
            R3Vector vertices[8];
            R3Vector left = part ? u1 : a1;
            R3Vector right = part ? u2 : a2;
            R3Real rmin = part ? 14 : 16;
            R3Real rmax = part ? 16 : 18;
            for (int z = 0; z < 2; z++) {
                R3Vector off = r3Vector(0, 0, z ? 10 : -10);
                vertices[z * 4] = r3VectorAdd(r3VectorScale(left, rmin), off);
                vertices[z * 4 + 1] = r3VectorAdd(r3VectorScale(left, rmax), off);
                vertices[z * 4 + 2] = r3VectorAdd(r3VectorScale(right, rmin), off);
                vertices[z * 4 + 3] = r3VectorAdd(r3VectorScale(right, rmax), off);
            }
            R3ColliderDesc collider = r3DefaultColliderDesc();
            r3ShapeDesc_SetConvexHull(&collider.shape, (R3VectorView){vertices, TB_COUNT(vertices)});
            r3InsertCollider(washer, &collider);
        }
        u1 = u2;
    }
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 20; j++) {
            for (int k = 0; k < 20; k++) {
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 0.2));
                collider.density = 1000;
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(-8 + i * 0.8, 13 + j * 0.8, -8 + k * 0.8);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 60, 35, 60, 0, 15, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
