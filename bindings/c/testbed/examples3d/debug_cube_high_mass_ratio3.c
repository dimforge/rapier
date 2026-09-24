/* Port of examples3d/debug_cube_high_mass_ratio3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugCubeHighMassRatio3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -2.2, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, 2, 2));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &collider);
    for (int i = 0; i < 4; i++) {
        for (int side = -1; side <= 1; side += 2) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(0, i * 0.8, side * 0.8);
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 0.2, 0.2));
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
            R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
            dynamicBody.position.translation = r3Vector(side * 0.8, (i + 0.5) * 0.8, 0);
            R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(0.2, 0.2, 1));
            dynamicBody.canSleep = !testbed->noSleep;
            rigidBodyHandle = r3InsertRigidBody(world, &dynamicBody);
            r3InsertCollider(rigidBodyHandle, &boxCollider);
        }
    }
    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 2 + (4 - 0.25) * 0.2 * 4, 0);
    rigidBody.additionalSolverIterations = 36;
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(2, 2, 2));
    rigidBody.canSleep = !testbed->noSleep;
    groundBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(groundBodyHandle, &boxCollider);
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
