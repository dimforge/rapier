/* Port of examples3d/debug_articulations3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbDebugArticulations3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3Rotation rotation = r3RotationFromAxisAngle(r3Vector(0.1, 0, 0.1), (R3Real)sqrt(0.02));
    for (int n = 0; n < 2; n++) {
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(30, 0.01, 30));
        collider.position = r3Pose(r3Vector(0, n ? -3 : -3.02, 0), rotation);
        if (n) {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(0, 0, 0);
            rigidBody.canSleep = !testbed->noSleep;
            R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(rigidBodyHandle, &collider);
        } else {
            r3InsertColliderWithoutParent(world, &collider);
        }
    }
    R3RigidBodyHandle handles[225];
    for (int k = 0; k < 15; k++) {
        for (int i = 0; i < 15; i++) {
            R3SharedShape *shape =
                r3CapsuleSharedShape(r3Vector(0, 0, -0.5), r3Vector(0, 0, 0.5), 0.4);
            R3RigidBodyHandle child;
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.bodyType = i ? R3_DYNAMIC : R3_FIXED;
            rigidBody.position.translation = r3Vector(k, 0, i * 2);
            R3ColliderDesc collider = r3DefaultColliderDesc();
            collider.shape.kind = R3_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = shape;
            rigidBody.canSleep = !testbed->noSleep;
            child = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(child, &collider);

            if (i) {
                R3JointDesc joint = r3DefaultJointDesc();
                joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
                joint.localFrame1.translation = r3Vector(0, 0, 0);
                joint.localFrame2.translation = r3Vector(0, 0, -2);
                r3InsertMultibodyJoint(handles[k * 15 + i - 1], child, &joint);
            }
            if (k && i) {
                R3JointDesc joint = r3DefaultJointDesc();
                joint.lockedAxes = R3_JOINT_SPHERICAL_AXES;
                joint.localFrame1.translation = r3Vector(0, 0, 0);
                joint.localFrame2.translation = r3Vector(-1, 0, 0);
                r3InsertImpulseJoint(handles[(k - 1) * 15 + i], child, &joint);
            }
            handles[k * 15 + i] = child;
            r3FreeSharedShape(shape);
        }
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
