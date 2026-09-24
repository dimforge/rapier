/* Port of examples2d/stress_tests/ragdolls2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct Part {
    R2RigidBodyHandle handle;
    R2Vector offset;
} Part;

/* A body and its offset from the ragdoll's torso. */
static Part part(R2World *world, R2Vector origin, R2Vector offset, R2ColliderDesc collider,
                 int noSleep) {
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = r2VectorAdd(origin, offset);
    body.canSleep = !noSleep;
    Part result = {.offset = offset};
    result.handle = r2InsertRigidBody(world, &body);
    r2InsertCollider(result.handle, &collider);

    return result;
}

static R2JointDesc revolute(Part parent, Part child, R2Vector anchor, R2Real min, R2Real max) {
    R2JointDesc joint = r2RevoluteJointDesc();
    joint.localFrame1.translation = r2VectorSub(anchor, parent.offset);
    joint.localFrame2.translation = r2VectorSub(anchor, child.offset);
    r2JointDesc_SetLimits(&joint, R2_AXIS_ANG_X, min, max);
    joint.contactsEnabled = 0;
    return joint;
}

/* Ten bodies and nine limited joints, matching the Rust ragdoll. */
static void ragdoll(R2World *world, R2Vector origin, int noSleep) {
    R2ColliderDesc collider = r2CapsuleYColliderDesc(.3, .15);
    Part torso = part(world, origin, r2Vector(0, 0), collider, noSleep);
    collider = r2BallColliderDesc(.15);
    Part head = part(world, origin, r2Vector(0, 0.55), collider, noSleep);
    R2JointDesc neck = revolute(torso, head, r2Vector(0, 0.42), -.5, .5);
    R2ImpulseJointHandle jointHandle =
        r2InsertImpulseJoint(torso.handle, head.handle, &neck);

    for (int side = -1; side <= 1; side += 2) {
        collider = r2CapsuleXColliderDesc(.14, .06);
        Part upperArm = part(world, origin, r2Vector(side * .36, .25), collider, noSleep);
        collider = r2CapsuleXColliderDesc(.14, .06);
        Part forearm = part(world, origin, r2Vector(side * .70, .25), collider, noSleep);
        collider = r2CapsuleYColliderDesc(.16, .07);
        Part thigh = part(world, origin, r2Vector(side * .09, -.52), collider, noSleep);
        collider = r2CapsuleYColliderDesc(.16, .07);
        Part shin = part(world, origin, r2Vector(side * .09, -.92), collider, noSleep);
        R2JointDesc shoulder = revolute(torso, upperArm, r2Vector(side * .19, .25), -1.2, 1.2);
        jointHandle = r2InsertImpulseJoint(torso.handle, upperArm.handle, &shoulder);

        R2JointDesc elbow = revolute(upperArm, forearm, r2Vector(side * .53, .25), 0, 2.5);
        jointHandle = r2InsertImpulseJoint(upperArm.handle, forearm.handle, &elbow);

        R2JointDesc hip = revolute(torso, thigh, r2Vector(side * .09, -.33), -1, 1);
        jointHandle = r2InsertImpulseJoint(torso.handle, thigh.handle, &hip);

        R2JointDesc knee = revolute(thigh, shin, r2Vector(side * .09, -.72), 0, 2.3);
        jointHandle = r2InsertImpulseJoint(thigh.handle, shin.handle, &knee);
    }
}

void tbStressTestsRagdolls2(Testbed *testbed) {
    R2World *world = r2NewWorld();
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(1000, 1));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* 200 ragdolls: 20 columns and 10 layers. */
    for (int layer = 0; layer < 10; ++layer) {
        for (int col = 0; col < 20; ++col) {
            ragdoll(world, r2Vector(col * 2.2, 1.5 + layer * 2.6), testbed->noSleep);
        }
    }
    tbCamera2(testbed, 22, 6, 15);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
