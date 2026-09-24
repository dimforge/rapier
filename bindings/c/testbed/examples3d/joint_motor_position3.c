/* Port of examples3d/joint_motor_position3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbJointMotorPosition3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle ground;
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r3InsertRigidBody(world, &groundBody);
    for (int row = 0; row < 2; row++) {
        for (int num = 0; num < (row ? 8 : 9); num++) {
            R3Real x = -6 + 1.5 * num;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(x, row ? 4.5 : 2, 0);
            rigidBody.canSleep = 0;
            if (row) {
                rigidBody.position =
                    r3Pose(r3Vector(x, 4.5, 0), r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI));
            }
            R3RigidBodyHandle handle;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 0.5, 0.1));
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
            joint.localFrame1.translation = r3Vector(x, row ? 5 : 1.5, 0);
            joint.localFrame2.translation = r3Vector(0, -0.5, 0);
            R3Real angle = -R3_PI + R3_PI / 4 * num;
            if (row) {
                r3JointDesc_SetMotor(&joint, R3_AXIS_ANG_X, 0, 1.5, 0, 30);
                r3JointDesc_SetMotorMaxForce(&joint, R3_AXIS_ANG_X, 100);
                r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, -R3_PI, angle);
            } else {
                r3JointDesc_SetMotor(&joint, R3_AXIS_ANG_X, angle, 0, 1000, 150);
            }
            r3InsertImpulseJoint(ground, handle, &joint);
        }
    }
    r3SetGravity(world, r3Vector(0, 0, 0));
    /* Set up the viewer. */
    tbCamera(testbed, 15, 5, 42, 13, 1, 1);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
