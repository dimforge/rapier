/* Port of examples2d/stress_tests/joint_prismatic2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointPrismatic2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int l = 0; l < 25; l++) {
        for (int j = 0; j < 50; j++) {
            R2Real x = j * 4;
            R2Real y = l * 24;
            R2RigidBodyHandle parent;
            R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
            rigidBody.position.translation = r2Vector(x, y);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
            rigidBody.canSleep = !testbed->noSleep;
            parent = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(parent, &collider);
            for (int i = 0; i < 10; i++) {
                R2RigidBodyHandle handle;
                R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
                rigidBody.position.translation = r2Vector(x, y - i - 1);
                R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
                rigidBody.canSleep = !testbed->noSleep;
                handle = r2InsertRigidBody(world, &rigidBody);
                r2InsertCollider(handle, &collider);
                R2JointDesc joint = r2PrismaticJointDesc(r2Vector(i % 2 ? -1 : 1, 1));
                joint.localFrame1.translation = r2Vector(0, 0);
                joint.localFrame2.translation = r2Vector(0, 1);
                r2JointDesc_SetLimits(&joint, R2_AXIS_LIN_X, -1.5, 1.5);
                r2InsertImpulseJoint(parent, handle, &joint);
                parent = handle;
            }
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 80, 80, 15);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
