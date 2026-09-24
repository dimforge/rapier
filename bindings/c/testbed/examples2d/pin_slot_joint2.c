/* Port of examples2d/pin_slot_joint2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/character.h"

void tbPinSlotJoint2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(3, 0.1));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    R2RigidBodyHandle characterHandle, cubeHandle, ballHandle;
    {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 0.3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.15, 0.3));
        characterHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(characterHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(1, 1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.4, 0.4));
        cubeHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(cubeHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(1, 1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2BallColliderDesc(.1);
        ballHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(ballHandle, &collider);
    }
    {
        R2JointDesc fixedJoint = r2FixedJointDesc();
        fixedJoint.localFrame1.translation = r2Vector(0, 0);
        fixedJoint.localFrame2.translation = r2Vector(0, -0.4);
        r2InsertImpulseJoint(cubeHandle, ballHandle, &fixedJoint);
    }
    {
        R2JointDesc pinSlotJoint = r2PinSlotJointDesc(r2Vector(1 / sqrt(2), 1 / sqrt(2)));
        pinSlotJoint.localFrame1.translation = r2Vector(2, 2);
        pinSlotJoint.localFrame2.translation = r2Vector(0, 0.4);
        r2JointDesc_SetLimits(&pinSlotJoint, R2_AXIS_LIN_X, -1, INFINITY);
        r2InsertImpulseJoint(characterHandle, cubeHandle, &pinSlotJoint);
    }
    CharacterControlMode controlMode = CHARACTER_KINEMATIC;
    R2KinematicCharacterController *controller = NULL;
    R2PidController *pid = NULL;
    controller = r2NewKinematicCharacterController();
    pid = r2NewPidController();
    tbCamera2(testbed, 0, 1, 100);

    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
            updateCharacter(testbed, world, &controlMode, controller, pid, characterHandle);
        }
    }
    r2FreePidController(pid);
    r2FreeKinematicCharacterController(controller);
    r2FreeWorld(world);
}
