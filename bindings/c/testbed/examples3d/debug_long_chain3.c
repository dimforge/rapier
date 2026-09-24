/* Port of examples3d/debug_long_chain3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugLongChain3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle last = {0};
    const R3Real shift = (R3Real)0.2 * (R3Real)2.2;
    for (int i = 0; i < 85; i++) {
        R3RigidBodyHandle handle;
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.bodyType = i ? R3_DYNAMIC : R3_FIXED;
        rigidBody.position.translation = r3Vector(0, 0, i * shift);
        R3ColliderDesc collider = r3BallColliderDesc(0.2);
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);
        if (i) {
            R3JointDesc joint = r3DefaultJointDesc();
            joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
            joint.localFrame1.translation = r3Vector(0, 0, i == 1 ? 0 : shift / 2);
            joint.localFrame2.translation = r3Vector(0, 0, i == 1 ? -shift : -shift / 2);
            r3InsertMultibodyJoint(last, handle, &joint);
        }
        last = handle;
    }
    /* Set up the viewer. */
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
