/* Port of examples2d/joint_motor_position2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbJointMotorPosition2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle ground;
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &groundBody);
    for (int row = 0; row < 2; row++) {
        for (int num = 0; num < (row ? 8 : 9); num++) {
            R2Real x = -6 + 1.5 * num;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, row ? 4.5 : 2);
            rigidBody.canSleep = 0;
            if (row) {
                rigidBody.position = r2Pose(r2Vector(x, 4.5), r2Rotation(R2_PI));
            }
            R2RigidBodyHandle handle;
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 0.5));
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);
            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame1.translation = r2Vector(x, row ? 5 : 1.5);
            joint.localFrame2.translation = r2Vector(0, -0.5);
            R2Real angle = -R2_PI + R2_PI / 4 * num;
            if (row) {
                r2JointDesc_SetMotor(&joint, R2_AXIS_ANG_X, 0, 1.5, 0, 30);
                r2JointDesc_SetMotorMaxForce(&joint, R2_AXIS_ANG_X, 100);
                r2JointDesc_SetLimits(&joint, R2_AXIS_ANG_X, -R2_PI, angle);
            } else {
                r2JointDesc_SetMotor(&joint, R2_AXIS_ANG_X, angle, 0, 1000, 150);
            }
            r2InsertImpulseJoint(ground, handle, &joint);
        }
    }
    r2SetGravity(world, r2Vector(0, 0));
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
