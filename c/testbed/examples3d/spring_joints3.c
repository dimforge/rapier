/* Port of examples3d/spring_joints3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbSpringJoints3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyHandle ground;
    R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
    groundBody.position.translation = r3Vector(0, 0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r3InsertRigidBody(world, &groundBody);
    R3Real damping = 2 * (R3Real)sqrt(1000 * 4 * R3_PI / 3 * 0.125);
    for (int i = 0; i <= 30; i++) {
        R3Real x = -6 + 1.5 * i;
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(x, 4.5, 0);
        rigidBody.canSleep = 0;
        R3RigidBodyHandle handle;
        R3ColliderDesc ballCollider = r3BallColliderDesc(0.5);
        if (testbed->noSleep) {
            rigidBody.canSleep = 0;
            rigidBody.sleeping = 0;
        }
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &ballCollider);
        R3JointDesc joint = r3SpringJointDesc(0, 1000, i / 15.0 * damping);
        joint.localFrame1.translation = r3Vector(x, 1.5, 0);
        r3InsertImpulseJoint(ground, handle, &joint);
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.5, 0.5, 0.5));
        collider.density = 100;
        R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
        dynamicBody.position.translation = r3Vector(x, 9.5, 0);
        dynamicBody.canSleep = !testbed->noSleep;
        R3RigidBodyHandle dynamicBodyHandle = r3InsertRigidBody(world, &dynamicBody);
        r3InsertCollider(dynamicBodyHandle, &collider);
    }
    /* Set up the viewer. */
    tbCamera(testbed, 15, 5, 42, 13, 1, 1);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
