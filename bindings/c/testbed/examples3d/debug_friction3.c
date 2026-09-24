/* Port of examples3d/debug_friction3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugFriction3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3ColliderDesc floor = r3CuboidColliderDesc(r3Vector(100, 0.1, 100));
    floor.friction = 1.5;
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 0, 0);
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &floor);
    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 1.1, 0);
    rigidBody.position =
        r3Pose(r3Vector(0, 1.1, 0), r3RotationFromAxisAngle(r3Vector(0, 1, 0), 0.3));
    rigidBody.linvel = r3Vector(sin(0.3) * 50, 0, cos(0.3) * 50);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, 1, 3));
    collider.friction = 1.5;
    rigidBody.canSleep = !testbed->noSleep;
    groundBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(groundBodyHandle, &collider);
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
