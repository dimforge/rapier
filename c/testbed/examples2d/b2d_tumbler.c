/* Port of examples2d/b2d_tumbler.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dTumbler(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyHandle ground;
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &groundBody);
    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 10);
    rigidBody.canSleep = 0;
    R2RigidBodyHandle drum;
    if (testbed->noSleep) {
        rigidBody.canSleep = 0;
        rigidBody.sleeping = 0;
    }
    drum = r2InsertRigidBody(world, &rigidBody);
    const R2Vector half[] = {r2Vector(0.5, 10), r2Vector(0.5, 10), r2Vector(10, 0.5),
                             r2Vector(10, 0.5)};
    const R2Vector off[] = {r2Vector(10, 0), r2Vector(-10, 0), r2Vector(0, 10), r2Vector(0, -10)};
    for (int i = 0; i < 4; i++) {
        R2ColliderDesc collider = r2CuboidColliderDesc(half[i]);
        collider.position.translation = off[i];
        collider.density = 50;
        r2InsertCollider(drum, &collider);
    }
    R2JointDesc joint = r2RevoluteJointDesc();
    joint.localFrame1.translation = r2Vector(0, 10);
    joint.localFrame2.translation = r2Vector(0, 0);
    r2JointDesc_SetMotor(&joint, R2_AXIS_ANG_X, 0, R2_PI / 180 * 25, 0, 1.0e5);
    r2JointDesc_SetMotorMaxForce(&joint, R2_AXIS_ANG_X, 1.0e8);
    r2InsertImpulseJoint(ground, drum, &joint);
    for (int i = 0; i < 45; i++) {
        for (int j = 0; j < 45; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-9 + j * 0.4, 1 + i * 0.4);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.125, 0.125));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 10, 12);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
