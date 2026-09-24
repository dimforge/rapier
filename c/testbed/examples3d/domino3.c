/* Port of examples3d/domino3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDomino3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(200.1, 0.1, 200.1));
    groundBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle groundBodyHandle = r3InsertRigidBody(world, &groundBody);
    r3InsertCollider(groundBodyHandle, &collider);
    R3Real angle = 0;
    R3Real radius = 10;
    int skip = 0;
    for (int i = 0; i < 4000; i++) {
        R3Real perimeter = 2 * R3_PI * radius;
        R3Real prev = angle;
        angle += 2 * R3_PI * 0.4 / perimeter;
        R3Real x = (R3Real)sin(angle);
        R3Real z = (R3Real)cos(angle);
        int nudged = fmod(angle, 2 * R3_PI) < fmod(prev, 2 * R3_PI);
        R3Real tilt = nudged || i == 3999 ? 0.2 : 0;
        if (!skip) {
            R3Rotation rotation = r3RotationMul(r3RotationFromAxisAngle(r3Vector(x, 0, z), tilt),
                                                r3RotationFromAxisAngle(r3Vector(0, 1, 0), angle));
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(x * radius, 2.1, z * radius);
            rigidBody.position = r3Pose(r3Vector(x * radius, 2.1, z * radius), rotation);
            R3RigidBodyHandle handle;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 2, 1));
            rigidBody.canSleep = !testbed->noSleep;
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);
            tbBodyColor(testbed, handle, i % 2 ? 0.6f : 0.7f, i % 2 ? 1 : 0.5f, i % 2 ? 0.6f : 0.9f,
                        1);
        } else {
            skip--;
        }
        if (nudged) {
            skip = 5;
        }
        radius += 1.5 / perimeter;
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
