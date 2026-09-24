/* Port of examples2d/b2d_joint_grid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dJointGrid(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyHandle *handles = calloc(1, 10000 * sizeof(*handles));
    if (!handles) {
        abort();
    }
    for (int k = 0; k < 100; k++) {
        for (int i = 0; i < 100; i++) {
            int fixed = k >= 47 && k <= 53 && i == 0;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.bodyType = fixed ? R2_FIXED : R2_DYNAMIC;
            rigidBody.position.translation = r2Vector(k, -i);
            if (!fixed) {
                rigidBody.canSleep = 0;
            }
            R2ColliderDesc collider = r2BallColliderDesc(0.4);
            collider.collisionGroups = (R2InteractionGroups){2, UINT32_MAX ^ 2, R2_GROUPS_AND};
            R2RigidBodyHandle handle;
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);
            for (int dir = 0; dir < 2; dir++) {
                if (dir ? k > 0 : i > 0) {
                    R2JointDesc joint = r2DefaultJointDesc();
                    joint.lockedAxes = 3;
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
    tbCamera2(testbed, 50, -50, 4);
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
