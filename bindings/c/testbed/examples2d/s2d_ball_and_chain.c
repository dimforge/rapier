/* Port of examples2d/s2d_ball_and_chain.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dBallAndChain(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    R2RigidBodyHandle ground;
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    ground = r2InsertRigidBody(world, &groundBody);

    R2RigidBodyHandle prev = ground;
    for (int i = 0; i < 40; i++) {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0 + 0.5 + i, 20);
        rigidBody.linearDamping = 0.1;
        rigidBody.angularDamping = 0.1;
        R2SharedShape *shape = r2CapsuleSharedShape(r2Vector(-0.5, 0), r2Vector(0.5, 0), 0.125);
        R2ColliderDesc collider = r2DefaultColliderDesc();
        collider.shape.kind = R2_SHAPE_DESC_SHARED;
        collider.shape.sharedShape = shape;
        collider.friction = 0.6;
        collider.density = 20;
        R2RigidBodyHandle handle;
        rigidBody.canSleep = !testbed->noSleep;
        handle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(handle, &collider);
        R2JointDesc joint = r2RevoluteJointDesc();
        joint.localFrame1.translation = i ? r2Vector(0.5, 0) : r2Vector(0, 20);
        joint.localFrame2.translation = r2Vector(-0.5, 0);
        joint.contactsEnabled = 0;
        r2InsertImpulseJoint(prev, handle, &joint);
        prev = handle;

        r2FreeSharedShape(shape);
    }
    R2RigidBodyDesc dynamicBody = r2DynamicRigidBodyDesc();
    dynamicBody.position.translation = r2Vector(48, 20);
    dynamicBody.linearDamping = 0.1;
    dynamicBody.angularDamping = 0.1;
    R2ColliderDesc ballCollider = r2BallColliderDesc(8);
    ballCollider.density = 20;
    ballCollider.friction = 0.6;
    R2RigidBodyHandle handleH;
    dynamicBody.canSleep = !testbed->noSleep;
    handleH = r2InsertRigidBody(world, &dynamicBody);
    r2InsertCollider(handleH, &ballCollider);
    R2JointDesc jointValue = r2RevoluteJointDesc();
    jointValue.localFrame1.translation = r2Vector(0.5, 0);
    jointValue.localFrame2.translation = r2Vector(-8, 0);
    jointValue.contactsEnabled = 0;
    r2InsertImpulseJoint(prev, handleH, &jointValue);
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
