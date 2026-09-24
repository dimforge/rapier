/* Port of examples2d/joints2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbJoints2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    int variable = (int)tbSetting(testbed, "Variable softness", 0, 0, 1, 1);
    R2RigidBodyHandle *handles = calloc(1, 100 * sizeof(*handles));
    if (!handles) {
        abort();
    }
    for (int k = 0; k < 10; k++) {
        for (int i = 0; i < 10; i++) {
            int fixed = i == 0 && k == 0;
            R2RigidBodyHandle handle;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.bodyType = fixed ? R2_FIXED : R2_DYNAMIC;
            rigidBody.position.translation = r2Vector(k, -i);
            R2ColliderDesc collider = r2BallColliderDesc(0.4);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);
            for (int dir = 0; dir < 2; dir++) {
                if (dir ? k > 0 : i > 0) {
                    R2JointDesc joint = r2RevoluteJointDesc();
                    joint.localFrame1.translation = r2Vector(0, 0);
                    joint.localFrame2.translation = dir ? r2Vector(-1, 0) : r2Vector(0, 1);
                    if (variable) {
                        R2Real scale = (i > k ? i : k) + 1;
                        joint.softness = (R2SpringCoefficients){5 * scale, 0.1 * scale};
                    }
                    r2InsertImpulseJoint(handles[k * 10 + i - (dir ? 10 : 1)], handle,
                                         &joint);
                }
            }
            handles[k * 10 + i] = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 4.0, -4.0, 20);
    free(handles);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
