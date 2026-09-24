/* Port of examples2d/stress_tests/joint_fixed2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointFixed2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle handles[900];
    for (int xx = 0; xx < 4; xx++) {
        for (int yy = 0; yy < 4; yy++) {
            for (int k = 0; k < 30; k++) {
                for (int i = 0; i < 30; i++) {
                    R2RigidBodyHandle handle;
                    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                    rigidBody.bodyType = k ? R2_DYNAMIC : R2_FIXED;
                    rigidBody.position.translation = r2Vector(xx * 32 + k, yy * 34 - i);
                    R2ColliderDesc collider = r2BallColliderDesc(0.4);
                    rigidBody.canSleep = !testbed->noSleep;
                    handle = r2InsertRigidBody(world, &rigidBody);
                    r2InsertCollider(handle, &collider);
                    if (i) {
                        R2JointDesc joint = r2DefaultJointDesc();
                        joint.lockedAxes = R2_JOINT_FIXED_AXES;
                        joint.localFrame1.translation = r2Vector(0, 0);
                        joint.localFrame2.translation = r2Vector(0, 1);
                        r2InsertImpulseJoint(handles[k * 30 + i - 1], handle, &joint);
                    }
                    if (k) {
                        R2JointDesc joint = r2DefaultJointDesc();
                        joint.lockedAxes = R2_JOINT_FIXED_AXES;
                        joint.localFrame1.translation = r2Vector(0, 0);
                        joint.localFrame2.translation = r2Vector(-1, 0);
                        r2InsertImpulseJoint(handles[k * 30 + i - 30], handle, &joint);
                    }
                    handles[k * 30 + i] = handle;
                }
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 50, 50, 5);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
