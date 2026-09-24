/* Port of examples3d/debug_multibody_ang_motor_pos3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugMultibodyAngMotorPos3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle handle;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
    rigidBody.canSleep = !testbed->noSleep;
    handle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(handle, &collider);
    R3RigidBodyHandle handleB;
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(0, 1, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
    dynamicBody.canSleep = !testbed->noSleep;
    handleB = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(handleB, &boxCollider);
    R3JointDesc joint = r3DefaultJointDesc();
    joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
    joint.localFrame1.translation = r3Vector(0, 4, 0);
    joint.localFrame2.translation = r3Vector(0, 0, 0);
    for (uint32_t axis = R3_AXIS_ANG_X; axis <= R3_AXIS_ANG_Z; axis++) {
        r3JointDesc_SetMotor(&joint, axis, axis == R3_AXIS_ANG_X ? 1 : 0, 0, 1000, 200);
    }
    r3InsertMultibodyJoint(handle, handleB, &joint);
    /* Set up the viewer. */
    tbCamera(testbed, 20, 0, 0, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
