/* Port of examples2d/debug_angular_limits2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugAngularLimits2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, 0));
    int multi = (int)tbSetting(testbed, "Multibody joints", 0, 0, 1, 1);
    const R2Real limits[][2] = {{-45, 45}, {0, 270}, {135, 225}, {-350, 0}, {-200, 200}};
    for (int i = 0; i < 5; i++) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            R2Vector position = r2Vector(i * 4, dir > 0 ? 0 : -4);
            R2RigidBodyHandle anchor;
            R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
            groundBody.position.translation = position;
            R2ColliderDesc collider = r2BallColliderDesc(0.2);
            groundBody.canSleep = !testbed->noSleep;
            anchor = r2InsertRigidBody(world, &groundBody);
            r2InsertCollider(anchor, &collider);
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2VectorAdd(position, r2Vector(1, 0));
            rigidBody.angularDamping = 3;
            rigidBody.canSleep = 0;
            R2RigidBodyHandle handle;
            R2ColliderDesc boxCollider = r2CuboidColliderDesc(r2Vector(0.5, 0.1));
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &boxCollider);
            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame1.translation = r2Vector(0, 0);
            joint.localFrame2.translation = r2Vector(-1, 0);
            r2JointDesc_SetLimits(&joint, R2_AXIS_ANG_X, limits[i][0] * R2_PI / 180,
                                 limits[i][1] * R2_PI / 180);
            r2JointDesc_SetMotor(&joint, R2_AXIS_ANG_X, 0, dir * 5, 0, 20);
            if (multi) {
                r2InsertMultibodyJoint(anchor, handle, &joint);
            } else {
                r2InsertImpulseJoint(anchor, handle, &joint);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 8, -2, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
