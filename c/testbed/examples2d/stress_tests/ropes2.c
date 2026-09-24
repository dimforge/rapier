/* Port of examples2d/stress_tests/ropes2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsRopes2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    for (int i = 0; i < 64; i++) {
        R2Vector top = r2Vector(i * 4, 0);
        R2RigidBodyHandle parent;
        R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
        groundBody.position.translation = top;
        groundBody.canSleep = !testbed->noSleep;
        parent = r2InsertRigidBody(world, &groundBody);

        for (int s = 0; s < 60; s++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2VectorAdd(top, r2Vector(0, -(s + 0.5)));
            rigidBody.linvel = r2Vector(2, 0);
            R2RigidBodyHandle handle;
            R2ColliderDesc collider = r2CapsuleYColliderDesc(0.35, 0.1);
            rigidBody.canSleep = !testbed->noSleep;
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);

            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame1.translation = r2Vector(0, s ? -0.5 : 0);
            joint.localFrame2.translation = r2Vector(0, 0.5);
            joint.contactsEnabled = 0;
            r2InsertImpulseJoint(parent, handle, &joint);
            parent = handle;
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 128, -30, 4);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
