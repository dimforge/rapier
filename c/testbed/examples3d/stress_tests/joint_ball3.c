/* Port of examples3d/stress_tests/joint_ball3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointBall3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle *handles = calloc(1, 10000 * sizeof(*handles));
    if (!handles) {
        abort();
    }
    for (int k = 0; k < 100; k++) {
        for (int i = 0; i < 100; i++) {
            int fixed = i == 0 && (k % 4 == 0 || k == 99);
            R3RigidBodyHandle handle;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
            rigidBody.position.translation = r3Vector(k, 0, i);
            R3ColliderDesc collider = r3BallColliderDesc(0.4);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);
            if (i) {
                R3JointDesc joint = r3DefaultJointDesc();
                joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
                joint.localFrame1.translation = r3Vector(0, 0, 0);
                joint.localFrame2.translation = r3Vector(0, 0, -1);
                r3InsertImpulseJoint(handles[k * 100 + i - 1], handle, &joint);
            }
            if (k) {
                R3JointDesc joint = r3DefaultJointDesc();
                joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
                joint.localFrame1.translation = r3Vector(0, 0, 0);
                joint.localFrame2.translation = r3Vector(-1, 0, 0);
                r3InsertImpulseJoint(handles[k * 100 + i - 100], handle, &joint);
            }
            handles[k * 100 + i] = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, -110, -46, 170, 54, -38, 29);
    free(handles);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
