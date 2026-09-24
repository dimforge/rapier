/* Port of examples3d/joints3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include <float.h>

static void createPrismaticJoints(Testbed *testbed, R3World *world, R3Vector origin, size_t num,
                                  int useArticulations) {
    R3RigidBodyHandle currParent;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = origin;
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.4, .4, .4));
        currParent = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(currParent, &collider);
    }
    for (size_t i = 0; i < num; ++i) {
        R3RigidBodyHandle currChild;
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(origin.x, origin.y, origin.z + (i + 1) * 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.4, .4, .4));
            currChild = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(currChild, &collider);
        }
        const R3Vector axis = r3VectorNormalize(r3Vector(i % 2 == 0 ? 1 : -1, 1, 0));
        R3JointDesc prism = r3PrismaticJointDesc(axis);
        prism.localFrame1.translation = r3Vector(0, 0, 0);
        prism.localFrame2.translation = r3Vector(0, 0, -2);
        r3JointDesc_SetLimits(&prism, R3_AXIS_LIN_X, -2, 2);
        if (useArticulations) {
            r3InsertMultibodyJoint(currParent, currChild, &prism);
        } else {
            r3InsertImpulseJoint(currParent, currChild, &prism);
        }

        currParent = currChild;
    }
}

static void createActuatedPrismaticJoints(Testbed *testbed, R3World *world, R3Vector origin,
                                          size_t num, int useArticulations) {
    R3RigidBodyHandle currParent;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = origin;
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.4, .4, .4));
        currParent = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(currParent, &collider);
    }
    for (size_t i = 0; i < num; ++i) {
        R3RigidBodyHandle currChild;
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = r3Vector(origin.x, origin.y, origin.z + (i + 1) * 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.4, .4, .4));
            currChild = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(currChild, &collider);
        }
        const R3Vector axis = r3VectorNormalize(r3Vector(i % 2 == 0 ? 1 : -1, 1, 0));
        R3JointDesc prism = r3PrismaticJointDesc(axis);
        prism.localFrame1.translation = r3Vector(0, 0, 2);
        prism.localFrame2.translation = r3Vector(0, 0, 0);
        if (i == 0) {
            r3JointDesc_SetMotorVelocity(&prism, R3_AXIS_LIN_X, 2, 1e5);
            r3JointDesc_SetLimits(&prism, R3_AXIS_LIN_X, -2, 5);
            r3JointDesc_SetMotorMaxForce(&prism, R3_AXIS_LIN_X, 100);
        } else if (i == 1) {
            r3JointDesc_SetLimits(&prism, R3_AXIS_LIN_X, -FLT_MAX, 5);
            r3JointDesc_SetMotorVelocity(&prism, R3_AXIS_LIN_X, 6, 1e3);
            r3JointDesc_SetMotorMaxForce(&prism, R3_AXIS_LIN_X, 100);
        } else {
            r3JointDesc_SetMotorPosition(&prism, R3_AXIS_LIN_X, 2, 1e3, 1e2);
            r3JointDesc_SetMotorMaxForce(&prism, R3_AXIS_LIN_X, 60);
        }
        if (useArticulations) {
            r3InsertMultibodyJoint(currParent, currChild, &prism);
        } else {
            r3InsertImpulseJoint(currParent, currChild, &prism);
        }

        currParent = currChild;
    }
}

static void createRevoluteJoints(Testbed *testbed, R3World *world, R3Vector origin, size_t num,
                                 int useArticulations) {
    R3RigidBodyHandle currParent;
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(origin.x, origin.y, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(.4, .4, .4));
        currParent = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(currParent, &collider);
    }
    for (size_t i = 0; i < num; ++i) {
        const R3Real z = origin.z + i * 4 + 2;
        const R3Vector positions[] = {{origin.x, origin.y, z},
                                      {origin.x + 2, origin.y, z},
                                      {origin.x + 2, origin.y, z + 2},
                                      {origin.x, origin.y, z + 2}};
        R3RigidBodyHandle handles[4];
        for (size_t k = 0; k < 4; ++k) {
            {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = positions[k];
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
                handles[k] = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(handles[k], &collider);
            }
        }
        {
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
            joint.localFrame2.translation = r3Vector(0, 0, -2);
            if (useArticulations) {
                r3InsertMultibodyJoint(currParent, handles[0], &joint);
            } else {
                r3InsertImpulseJoint(currParent, handles[0], &joint);
            }
        }
        {
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(1, 0, 0));
            joint.localFrame2.translation = r3Vector(-2, 0, 0);
            if (useArticulations) {
                r3InsertMultibodyJoint(handles[0], handles[1], &joint);
            } else {
                r3InsertImpulseJoint(handles[0], handles[1], &joint);
            }
        }
        {
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
            joint.localFrame2.translation = r3Vector(0, 0, -2);
            if (useArticulations) {
                r3InsertMultibodyJoint(handles[1], handles[2], &joint);
            } else {
                r3InsertImpulseJoint(handles[1], handles[2], &joint);
            }
        }
        {
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(1, 0, 0));
            joint.localFrame2.translation = r3Vector(2, 0, 0);
            if (useArticulations) {
                r3InsertMultibodyJoint(handles[2], handles[3], &joint);
            } else {
                r3InsertImpulseJoint(handles[2], handles[3], &joint);
            }
        }
        currParent = handles[3];
    }
}

static void createRevoluteJointsWithLimits(Testbed *testbed, R3World *world, R3Vector origin,
                                           int useArticulations) {
    R3RigidBodyDesc groundBuilder = r3FixedRigidBodyDesc();
    groundBuilder.position.translation = origin;
    R3RigidBodyHandle ground = r3InsertRigidBody(world, &groundBuilder);

    R3RigidBodyHandle body1;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(0, 0, 0));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(4, 0.2, 2));
        body1 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(body1, &collider);
    }
    R3RigidBodyHandle body2;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(0, 0, 6));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(4, 0.2, 2));
        body2 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(body2, &collider);
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, -.2, .2);
        if (useArticulations) {
            r3InsertMultibodyJoint(ground, body1, &joint);
        } else {
            r3InsertImpulseJoint(ground, body1, &joint);
        }
    }
    {
        R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
        joint.localFrame2.translation = r3Vector(0, 0, -6);
        r3JointDesc_SetLimits(&joint, R3_AXIS_ANG_X, -.2, .2);
        if (useArticulations) {
            r3InsertMultibodyJoint(body1, body2, &joint);
        } else {
            r3InsertImpulseJoint(body1, body2, &joint);
        }
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(-2, 4, 0));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.6, 0.6, 0.6));
        collider.friction = 1;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(2, 16, 6));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.6, 0.6, 0.6));
        collider.friction = 1;
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
}

static void createSphericalJointsWithLimits(Testbed *testbed, R3World *world, R3Vector origin,
                                            int useArticulations) {
    R3RigidBodyDesc groundBuilder = r3FixedRigidBodyDesc();
    groundBuilder.position.translation = origin;
    R3RigidBodyHandle ground = r3InsertRigidBody(world, &groundBuilder);

    R3RigidBodyHandle body1;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(0, 0, 3));
        rigidBody.linvel = r3Vector(20, 20, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
        body1 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(body1, &collider);
    }
    R3RigidBodyHandle body2;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(origin, r3Vector(0, 0, 6));
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
        body2 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(body2, &collider);
    }
    {
        R3JointDesc joint = r3SphericalJointDesc();
        joint.localFrame2.translation = r3Vector(0, 0, -3);
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -0.2, 0.2);
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_Y, -0.2, 0.2);
        if (useArticulations) {
            r3InsertMultibodyJoint(ground, body1, &joint);
        } else {
            r3InsertImpulseJoint(ground, body1, &joint);
        }
    }
    {
        R3JointDesc joint = r3SphericalJointDesc();
        joint.localFrame2.translation = r3Vector(0, 0, -3);
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -0.3, 0.3);
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_Y, -0.3, 0.3);
        if (useArticulations) {
            r3InsertMultibodyJoint(body1, body2, &joint);
        } else {
            r3InsertImpulseJoint(body1, body2, &joint);
        }
    }
}

static void createFixedJoints(Testbed *testbed, R3World *world, R3Vector origin, size_t num,
                              int useArticulations) {
    R3RigidBodyHandle *bodyHandles = malloc(num * num * sizeof(*bodyHandles));
    if (!bodyHandles) {
        abort();
    }
    size_t count = 0;
    for (size_t i = 0; i < num; ++i) {
        for (size_t k = 0; k < num; ++k) {
            const int fixed = i == 0 && ((k % 4 == 0 && k != num - 2) || k == num - 1);
            R3RigidBodyHandle childHandle;
            {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
                rigidBody.position.translation = r3Vector(origin.x + k, origin.y, origin.z + i);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider = r3BallColliderDesc(.4);
                childHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(childHandle, &collider);
            }
            if (i > 0) {
                {
                    R3JointDesc joint = r3FixedJointDesc();
                    joint.localFrame2.translation = r3Vector(0, 0, -1);
                    if (useArticulations) {
                        r3InsertMultibodyJoint(bodyHandles[count - num], childHandle,
                                               &joint);
                    } else {
                        r3InsertImpulseJoint(bodyHandles[count - num], childHandle, &joint);
                    }
                }
            }
            if (k > 0) {
                {
                    R3JointDesc joint = r3FixedJointDesc();
                    joint.localFrame2.translation = r3Vector(-1, 0, 0);
                    r3InsertImpulseJoint(bodyHandles[count - 1], childHandle, &joint);
                }
            }
            bodyHandles[count++] = childHandle;
        }
    }
    free(bodyHandles);
}

static void createSphericalJoints(Testbed *testbed, R3World *world, size_t num,
                                  int useArticulations) {
    R3RigidBodyHandle *bodyHandles = malloc(num * num * sizeof(*bodyHandles));
    if (!bodyHandles) {
        abort();
    }
    size_t count = 0;
    for (size_t k = 0; k < num; ++k) {
        for (size_t i = 0; i < num; ++i) {
            const int fixed = i == 0 && (k % 4 == 0 || k == num - 1);
            R3RigidBodyHandle childHandle;
            {
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.bodyType = fixed ? R3_FIXED : R3_DYNAMIC;
                rigidBody.position.translation = r3Vector(k, 0, i * 2);
                rigidBody.canSleep = !testbed->noSleep;
                R3ColliderDesc collider =
                    r3CapsuleColliderDesc(r3Vector(0, 0, -0.5), r3Vector(0, 0, 0.5), .4);
                childHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(childHandle, &collider);
            }
            if (i > 0) {
                {
                    R3JointDesc joint = r3SphericalJointDesc();
                    joint.localFrame2.translation = r3Vector(0, 0, -2);
                    if (useArticulations) {
                        r3InsertMultibodyJoint(bodyHandles[count - 1], childHandle, &joint);
                    } else {
                        r3InsertImpulseJoint(bodyHandles[count - 1], childHandle, &joint);
                    }
                }
            }
            if (k > 0) {
                {
                    R3JointDesc joint = r3SphericalJointDesc();
                    joint.localFrame2.translation = r3Vector(-1, 0, 0);
                    r3InsertImpulseJoint(bodyHandles[count - num], childHandle, &joint);
                }
            }
            bodyHandles[count++] = childHandle;
        }
    }
    free(bodyHandles);
}

static void createActuatedRevoluteJoints(Testbed *testbed, R3World *world, R3Vector origin,
                                         size_t num, int useArticulations) {
    R3RigidBodyHandle parentHandle = {NULL, UINT32_MAX, UINT32_MAX};
    for (size_t i = 0; i < num; ++i) {
        R3RigidBodyHandle childHandle;
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.bodyType = i == 0 ? R3_FIXED : R3_DYNAMIC;
            rigidBody.position.translation =
                r3Vector(origin.x, origin.y + (i >= 1 ? -2 : 0), origin.z + i * 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.8, 2.4 / (i + 1), 0.4));
            childHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(childHandle, &collider);
        }
        if (i > 0) {
            R3JointDesc joint = r3RevoluteJointDesc(r3Vector(0, 0, 1));
            joint.localFrame2.translation = r3Vector(0, 0, -2);
            r3JointDesc_SetMotorModel(&joint, R3_AXIS_ANG_X, 0);
            if (i % 3 == 1) {
                r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_X, -20, 100);
            } else if (i == num - 1) {
                r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_X, R3_PI / 2, 200, 100);
            }
            if (i == 1) {
                joint.localFrame2.translation = r3Vector(0, 2, -2);
                r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_X, -2, 1000);
            }
            if (useArticulations) {
                r3InsertMultibodyJoint(parentHandle, childHandle, &joint);
            } else {
                r3InsertImpulseJoint(parentHandle, childHandle, &joint);
            }
        }
        parentHandle = childHandle;
    }
}

static void createActuatedSphericalJoints(Testbed *testbed, R3World *world, R3Vector origin,
                                          size_t num, int useArticulations) {
    R3RigidBodyHandle parentHandle = {NULL, UINT32_MAX, UINT32_MAX};
    for (size_t i = 0; i < num; ++i) {
        R3RigidBodyHandle childHandle;
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.bodyType = i == 0 ? R3_FIXED : R3_DYNAMIC;
            rigidBody.position.translation = r3Vector(origin.x, origin.y, origin.z + i * 2);
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3CapsuleYColliderDesc(.8 / (i + 1), .4);
            childHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(childHandle, &collider);
        }
        if (i > 0) {
            R3JointDesc joint = r3SphericalJointDesc();
            joint.localFrame1.translation = r3Vector(0, 0, 2);
            if (i == 1) {
                r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_X, 0, .1);
                r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_Y, .5, .1);
                r3JointDesc_SetMotorVelocity(&joint, R3_AXIS_ANG_Z, -2, .1);
            } else if (i == num - 1) {
                r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_X, 0, .2, 1);
                r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_Y, 1, .2, 1);
                r3JointDesc_SetMotorPosition(&joint, R3_AXIS_ANG_Z, R3_PI / 2, .2, 1);
            }
            if (useArticulations) {
                r3InsertMultibodyJoint(parentHandle, childHandle, &joint);
            } else {
                r3InsertImpulseJoint(parentHandle, childHandle, &joint);
            }
        }
        parentHandle = childHandle;
    }
}

static void createCoupledJoints(Testbed *testbed, R3World *world, R3Vector origin,
                                int useArticulations) {
    R3RigidBodyDesc builder = r3FixedRigidBodyDesc();
    builder.position.translation = origin;
    R3RigidBodyHandle ground = r3InsertRigidBody(world, &builder);

    R3RigidBodyHandle body1;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = origin;
        rigidBody.linvel = r3Vector(5, 5, 5);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
        body1 = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(body1, &collider);
    }
    {
        R3JointDesc joint = r3DefaultJointDesc();
        joint.lockedAxes = 0;
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_X, -3, 3);
        r3JointDesc_SetLimits(&joint, R3_AXIS_LIN_Y, 0, 3);
        joint.coupledAxes = 2 | 4;
        if (useArticulations) {
            r3InsertMultibodyJoint(ground, body1, &joint);
        } else {
            r3InsertImpulseJoint(ground, body1, &joint);
        }
    }
}

void joints3Run(Testbed *testbed, int useArticulations) {
    R3World *world = r3NewWorld();
    createPrismaticJoints(testbed, world, r3Vector(20, 5, 0), 4, useArticulations);
    createActuatedPrismaticJoints(testbed, world, r3Vector(25, 5, 0), 4, useArticulations);
    createRevoluteJoints(testbed, world, r3Vector(20, 0, 0), 3, useArticulations);
    createRevoluteJointsWithLimits(testbed, world, r3Vector(34, 0, 0), useArticulations);
    createFixedJoints(testbed, world, r3Vector(0, 10, 0), 10, useArticulations);
    createActuatedRevoluteJoints(testbed, world, r3Vector(20, 10, 0), 6, useArticulations);
    createActuatedSphericalJoints(testbed, world, r3Vector(13, 10, 0), 3, useArticulations);
    createSphericalJoints(testbed, world, 15, useArticulations);
    createSphericalJointsWithLimits(testbed, world, r3Vector(-5, 0, 0), useArticulations);
    createCoupledJoints(testbed, world, r3Vector(0, 20, 0), useArticulations);
    tbCamera(testbed, 15, 5, 42, 13, 1, 1);
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
