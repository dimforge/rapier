/* Port of examples3d/stress_tests/joint_fixed3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointFixed3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle handles[25];
    for (int m = 0; m < 10; m++) {
        for (int l = 0; l < 10; l++) {
            for (int j = 0; j < 5; j++) {
                for (int k = 0; k < 5; k++) {
                    for (int i = 0; i < 5; i++) {
                        int fixed = i == 0 && ((k % 4 == 0 && k != 3) || k == 4);
                        R3RigidBodyHandle handle;
                        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                        rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
                        rigidBody.position.translation = r3Vector(j * 10 + k, l * 3, m * 7 + i);
                        R3ColliderDesc collider = r3BallColliderDesc(0.4);
                        rigidBody.canSleep = !testbed->noSleep;
                        handle = r3InsertRigidBody(world, &rigidBody);
                        r3InsertCollider(handle, &collider);
                        if (i) {
                            R3JointDesc joint = r3DefaultJointDesc();
                            joint.lockedAxes = R3_JOINT_FIXED_AXES;
                            joint.localFrame1.translation = r3Vector(0, 0, 0);
                            joint.localFrame2.translation = r3Vector(0, 0, -1);
                            r3InsertImpulseJoint(handles[k * 5 + i - 1], handle, &joint);
                        }
                        if (k) {
                            R3JointDesc joint = r3DefaultJointDesc();
                            joint.lockedAxes = R3_JOINT_FIXED_AXES;
                            joint.localFrame1.translation = r3Vector(0, 0, 0);
                            joint.localFrame2.translation = r3Vector(-1, 0, 0);
                            r3InsertImpulseJoint(handles[k * 5 + i - 5], handle, &joint);
                        }
                        handles[k * 5 + i] = handle;
                    }
                }
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, -38, 14, 108, 46, 12, 23);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
