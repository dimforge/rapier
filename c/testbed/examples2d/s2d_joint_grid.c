/* Port of examples2d/s2d_joint_grid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dJointGrid(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle *handles = calloc(1, 10000 * sizeof(*handles));
    if (!handles) {
        abort();
    }
    for (int k = 0; k < 100; k++) {
        for (int i = 0; i < 100; i++) {
            int fixed = k >= 47 && k <= 53 && i == 0;
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
                    joint.localFrame1.translation = dir ? r2Vector(0.5, 0) : r2Vector(0, -0.5);
                    joint.localFrame2.translation = dir ? r2Vector(-0.5, 0) : r2Vector(0, 0.5);
                    joint.contactsEnabled = 0;
                    r2InsertImpulseJoint(handles[k * 100 + i - (dir ? 100 : 1)], handle,
                                         &joint);
                }
            }
            handles[k * 100 + i] = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);
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
