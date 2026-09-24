/* Port of examples3d/b3d_joint_grid.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB3dJointGrid(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    r3SetGravity(world, r3Vector(0, -10, 0));
    R3RigidBodyHandle *handles = calloc(1, 10000 * sizeof(*handles));
    if (!handles) {
        abort();
    }
    for (int k = 0; k < 100; k++) {
        for (int i = 0; i < 100; i++) {
            int fixed = i == 0;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
            rigidBody.position.translation = r3Vector(k, -i, 0);
            if (!fixed) {
                rigidBody.canSleep = 0;
            }
            R3ColliderDesc collider = r3BallColliderDesc(0.4);
            R3RigidBodyHandle handle;
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(handle, &collider);
            for (int dir = 0; dir < 2; dir++) {
                if (dir ? k > 0 : i > 0) {
                    R3JointDesc joint = r3DefaultJointDesc();
                    joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
                    joint.localFrame1.translation =
                        dir ? r3Vector(0.5, 0, 0) : r3Vector(0, -0.5, 0);
                    joint.localFrame2.translation =
                        dir ? r3Vector(-0.5, 0, 0) : r3Vector(0, 0.5, 0);
                    r3InsertImpulseJoint(handles[k * 100 + i - (dir ? 100 : 1)], handle,
                                         &joint);
                }
            }
            handles[k * 100 + i] = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 50, -25, 90, 50, -50, 0);
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
