/* Port of examples2d/rope_joints2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/character.h"

void tbRopeJoints2(Testbed *testbed) {
    R2World *world = r2NewWorld();

    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, -0.1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.75, 0.1));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(-0.85, 0.75);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 0.75));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R2RigidBodyDesc rigidBody = r2FixedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0.85, 0.75);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 0.75));
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &collider);
    }
    /* Manually controlled character, tethered to a ball. */
    R2RigidBodyHandle characterHandle;
    {
        R2RigidBodyDesc rigidBody = r2KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 0.3);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.15, 0.3));
        characterHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(characterHandle, &collider);
    }
    R2RigidBodyHandle childHandle;
    {
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(1, 1);
        rigidBody.canSleep = !testbed->noSleep;
        R2ColliderDesc collider = r2BallColliderDesc(.04);
        childHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(childHandle, &collider);
    }
    {
        R2JointDesc joint = r2RopeJointDesc(2);
        r2InsertImpulseJoint(characterHandle, childHandle, &joint);
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
