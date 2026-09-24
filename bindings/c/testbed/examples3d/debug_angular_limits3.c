/* Port of examples3d/debug_angular_limits3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugAngularLimits3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, 0, 0));
    int multi = (int)tbSetting(testbed, "Multibody joints", 0, 0, 1, 1);
    const R3Real limits[][2] = {{-45, 45}, {0, 270}, {135, 225}, {-350, 0}, {-200, 200}};
    for (int i = 0; i < 5; i++) {
        for (int dir = 1; dir >= -1; dir -= 2) {
            R3Vector position = r3Vector(i * 4, dir > 0 ? 0 : -4, 0);
            R3RigidBodyHandle anchor;
            R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
            groundBody.position.translation = position;
            R3ColliderDesc collider = r3BallColliderDesc(0.2);
            groundBody.canSleep = !testbed->noSleep;
            anchor = r3InsertRigidBody(world, &groundBody);
            r3InsertCollider(anchor, &collider);
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3VectorAdd(position, r3Vector(1, 0, 0));
            rigidBody.angularDamping = 3;
            rigidBody.canSleep = 0;
            R3RigidBodyHandle handle;
            R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(0.5, 0.1, 0.1));
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &boxCollider);
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
            joint.localFrame1.translation = r3Vector(0, 0, 0);
            joint.localFrame2.translation = r3Vector(-1, 0, 0);
            r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, limits[i][0] * R3_PI / 180,
                                 limits[i][1] * R3_PI / 180);
            r3JointDesc_SetMotor(&joint, R3_AXIS_ANG_X, 0, dir * 5, 0, 20);
            if (multi) {
                r3InsertMultibodyJoint(anchor, handle, &joint);
            } else {
                r3InsertImpulseJoint(anchor, handle, &joint);
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 8, -2, 25, 8, -2, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
