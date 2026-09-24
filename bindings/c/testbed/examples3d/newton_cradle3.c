/* Port of examples3d/newton_cradle3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbNewtonCradle3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int i = 0; i < 5; i++) {
        R3RigidBodyHandle ground;
        R3RigidBodyDesc groundBody = r3FixedRigidBodyDesc();
        groundBody.position.translation = r3Vector(i * 1.01, 5, 0);
        groundBody.canSleep = !testbed->noSleep;
        ground = r3InsertRigidBody(world, &groundBody);
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(i * 1.01, 0, 0);
        rigidBody.linvel = r3Vector(i == 4 ? 7 : 0, 0, 0);
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        collider.restitution = 1;
        R3RigidBodyHandle handle;
        rigidBody.canSleep = !testbed->noSleep;
        handle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(handle, &collider);
        R3JointDesc joint = r3DefaultJointDesc();
        joint.lockedAxes = 7;
        joint.localFrame1.translation = r3Vector(0, 0, 0);
        joint.localFrame2.translation = r3Vector(0, 5, 0);
        r3InsertImpulseJoint(ground, handle, &joint);
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
