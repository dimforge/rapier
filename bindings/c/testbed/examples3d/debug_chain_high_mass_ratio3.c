/* Port of examples3d/debug_chain_high_mass_ratio3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugChainHighMassRatio3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle last = {0};
    for (int i = 0; i < 17; i++) {
        R3Real r = i == 16 ? 2 : 0.2;
        R3Real a = (R3Real)0.2 * (R3Real)1.1;
        R3Real b = r + (R3Real)0.2 * (R3Real)0.1;
        R3Real z = i ? (i - 1) * 2 * a + a + b : 0;
        R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
        body.bodyType = i ? R3_DYNAMIC : R3_FIXED;
        body.position.translation = r3Vector(0, 0, z);
        body.additionalSolverIterations = 16;
        R3RigidBodyHandle handle;
        R3ColliderDesc collider = r3BallColliderDesc(r);
        body.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &body);
        r3InsertCollider(handle, &collider);
        if (i) {
            R3JointDesc joint = r3DefaultJointDesc();
            joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
            joint.localFrame1.translation = r3Vector(0, 0, i == 1 ? 0 : a);
            joint.localFrame2.translation = r3Vector(0, 0, i == 1 ? -a * 2 : -b);
            r3InsertImpulseJoint(last, handle, &joint);
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
