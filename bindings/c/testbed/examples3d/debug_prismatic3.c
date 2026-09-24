/* Port of examples3d/debug_prismatic3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugPrismatic3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, -0.1, 0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(50, 0.1, 50));
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);
    R3RigidBodyHandle box;
    R3RigidBodyDesc dynamicBody = r3DynamicRigidBodyDesc();
    dynamicBody.position.translation = r3Vector(0, 5, 0);
    R3ColliderDesc boxCollider = r3CuboidColliderDesc(r3Vector(1, 0.25, 1));
    dynamicBody.canSleep = !testbed->noSleep;
    box = r3InsertRigidBody(world, &dynamicBody);
    r3InsertCollider(box, &boxCollider);
    R3Vector offsets[] = {r3Vector(1, -1, -1), r3Vector(-1, -1, -1), r3Vector(1, -1, 1),
                          r3Vector(-1, -1, 1)};
    for (int i = 0; i < 4; i++) {
        R3RigidBodyHandle wheel;
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(r3Vector(0, 5, 0), offsets[i]);
        R3ColliderDesc collider = r3BallColliderDesc(0.5);
        rigidBody.canSleep = !testbed->noSleep;
        wheel = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(wheel, &collider);
        R3JointDesc joint = r3PrismaticJointDesc(r3Vector(0, 1, 0));
        joint.localFrame1.translation = offsets[i];
        joint.localFrame2.translation = r3Vector(0, 0, 0);
        r3JointDesc_SetMotor(&joint, R3_AXIS_LIN_X, 0, 0, 0.05, 0.2);
        r3InsertImpulseJoint(box, wheel, &joint);
    }
    R3RigidBodyDesc payloadBody = r3DynamicRigidBodyDesc();
    payloadBody.position.translation = r3Vector(1, 2.6, -1);
    R3ColliderDesc payloadCollider = r3CuboidColliderDesc(r3Vector(0.5, 0.1, 0.5));
    payloadBody.canSleep = !testbed->noSleep;
    rigidBodyHandle = r3InsertRigidBody(world, &payloadBody);
    r3InsertCollider(rigidBodyHandle, &payloadCollider);
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
