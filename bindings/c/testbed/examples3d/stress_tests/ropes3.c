/* Port of examples3d/stress_tests/ropes3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsRopes3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 64; i++) {
        R3Vector top = r3Vector(i / 8 * 4, 0, i % 8 * 4);
        R3RigidBodyHandle parent;
        R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
        groundBody.position.translation = top;
        groundBody.canSleep = !testbed->noSleep;
        parent = r3InsertRigidBody(world, &groundBody);

        for (int s = 0; s < 60; s++) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3VectorAdd(top, r3Vector(0, -(s + 0.5), 0));
            rigidBody.linvel = r3Vector(2, 0, 0);
            R3RigidBodyHandle handle;
            R3ColliderDesc collider = r3CapsuleYColliderDesc(0.35, 0.1);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);

            R3JointDesc joint = r3DefaultJointDesc();
            joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
            joint.localFrame1.translation = r3Vector(0, s ? -0.5 : 0, 0);
            joint.localFrame2.translation = r3Vector(0, 0.5, 0);
            joint.contactsEnabled = 0;
            r3InsertImpulseJoint(parent, handle, &joint);
            parent = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, -45, -10, -45, 14, -30, 14);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
