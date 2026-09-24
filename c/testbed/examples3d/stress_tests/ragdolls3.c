/* Port of examples3d/stress_tests/ragdolls3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

typedef struct Part {
    R3RigidBodyHandle handle;
    R3Vector offset;
} Part;

/* A body and its offset from the ragdoll's torso. */
static Part part(R3World *world, R3Vector origin, R3Vector offset, R3ColliderDesc collider,
                 int noSleep) {
    R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
    body.position.translation = r3VectorAdd(origin, offset);
    body.canSleep = !noSleep;
    Part result = {.offset = offset};
    result.handle = r3InsertRigidBody(world, &body);
    r3InsertCollider(result.handle, &collider);

    return result;
}

static R3JointDesc revolute(Part parent, Part child, R3Vector anchor, R3Real min, R3Real max) {
    R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
    joint.localFrame1.translation = r3VectorSub(anchor, parent.offset);
    joint.localFrame2.translation = r3VectorSub(anchor, child.offset);
    r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, min, max);
    joint.contactsEnabled = 0;
    return joint;
}

static R3JointDesc spherical(Part parent, Part child, R3Vector anchor, R3Real limit) {
    R3JointDesc joint = r3SphericalJointDesc();
    joint.localFrame1.translation = r3VectorSub(anchor, parent.offset);
    joint.localFrame2.translation = r3VectorSub(anchor, child.offset);
    r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, -limit, limit);
    r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_Y, -limit, limit);
    r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_Z, -limit, limit);
    joint.contactsEnabled = 0;
    return joint;
}

/* Ten bodies and nine limited joints, matching the Rust ragdoll. */
static void ragdoll(R3World *world, R3Vector origin, int noSleep) {
    R3ColliderDesc collider = r3CapsuleYColliderDesc(.3, .15);
    Part torso = part(world, origin, r3Vector(0, 0, 0), collider, noSleep);
    collider = r3BallColliderDesc(.15);
    Part head = part(world, origin, r3Vector(0, 0.55, 0), collider, noSleep);
    R3JointDesc neck = spherical(torso, head, r3Vector(0, 0.42, 0), .5);
    r3InsertImpulseJoint(torso.handle, head.handle, &neck);

    for (int side = -1; side <= 1; side += 2) {
        collider = r3CapsuleXColliderDesc(.14, .06);
        Part upperArm = part(world, origin, r3Vector(side * .36, .25, 0), collider, noSleep);
        collider = r3CapsuleXColliderDesc(.14, .06);
        Part forearm = part(world, origin, r3Vector(side * .70, .25, 0), collider, noSleep);
        collider = r3CapsuleYColliderDesc(.16, .07);
        Part thigh = part(world, origin, r3Vector(side * .09, -.52, 0), collider, noSleep);
        collider = r3CapsuleYColliderDesc(.16, .07);
        Part shin = part(world, origin, r3Vector(side * .09, -.92, 0), collider, noSleep);
        R3JointDesc shoulder = spherical(torso, upperArm, r3Vector(side * .19, .25, 0), 1.2);
        r3InsertImpulseJoint(torso.handle, upperArm.handle, &shoulder);

        R3JointDesc elbow = revolute(upperArm, forearm, r3Vector(side * .53, .25, 0), 0, 2.5);
        r3InsertImpulseJoint(upperArm.handle, forearm.handle, &elbow);

        R3JointDesc hip = spherical(torso, thigh, r3Vector(side * .09, -.33, 0), 1);
        r3InsertImpulseJoint(torso.handle, thigh.handle, &hip);

        R3JointDesc knee = revolute(thigh, shin, r3Vector(side * .09, -.72, 0), 0, 2.3);
        r3InsertImpulseJoint(thigh.handle, shin.handle, &knee);
    }
}

void tbStressTestsRagdolls3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(100, 1, 100));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* 125 ragdolls: a 5 by 5 grid with 5 layers. */
    for (int layer = 0; layer < 5; ++layer) {
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 5; ++col) {
                ragdoll(world, r3Vector(col * 2.2, 1.5 + layer * 2.6, row * 2.2), testbed->noSleep);
            }
        }
    }
    tbCamera(testbed, -12, 10, -12, 4.5, 1, 4.5);

    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
