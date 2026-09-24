/* Port of examples3d/stress_tests/joint_prismatic3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointPrismatic3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int m = 0; m < 8; m++) {
        for (int l = 0; l < 8; l++) {
            for (int j = 0; j < 50; j++) {
                R3Real x = j * 4;
                R3Real y = l * 10;
                R3Real z = m * 7;
                R3RigidBodyHandle parent;
                R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
                rigidBody.position.translation = r3Vector(x, y, z);
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
                rigidBody.canSleep = !testbed->noSleep;
                parent = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(parent, &collider);
                for (int i = 0; i < 5; i++) {
                    R3RigidBodyHandle handle;
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = r3Vector(x, y, z + i + 1);
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
                    rigidBody.canSleep = !testbed->noSleep;
                    handle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(handle, &collider);
                    R3JointDesc joint = r3PrismaticJointDesc(r3Vector(i % 2 ? -1 : 1, 1, 0));
                    joint.localFrame1.translation = r3Vector(0, 0, 0);
                    joint.localFrame2.translation = r3Vector(0, 0, -1);
                    r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -2, 0);
                    r3InsertImpulseJoint(parent, handle, &joint);
                    parent = handle;
                }
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 262, 63, 124, 101, 4, -3);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
