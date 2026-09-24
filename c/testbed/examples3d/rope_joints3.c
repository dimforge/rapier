/* Port of examples3d/rope_joints3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

#include "utils/character.h"

void tbRopeJoints3(Testbed *testbed) {
    R3World *world = r3NewWorld();

    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.75, 0.1, 0.75));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(-0.85, 0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 0.1, 0.75));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0.85, 0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 0.1, 0.75));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.1, -0.85);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.75, 0.1, 0.1));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.1, 0.85);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.75, 0.1, 0.1));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    /* Manually controlled character, tethered to a ball. */
    R3RigidBodyHandle characterHandle;
    {
        R3RigidBodyDesc rigidBody = r3KinematicPositionBasedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 0.3, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.15, 0.3, 0.15));
        characterHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(characterHandle, &collider);
    }
    tbBodyColor(testbed, characterHandle, 1, 131.0f / 255, 244.0f / 255, 1);
    R3RigidBodyHandle childHandle;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(1, 1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3BallColliderDesc(.04);
        childHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(childHandle, &collider);
    }
    {
        R3JointDesc joint = r3RopeJointDesc(2);
        r3InsertImpulseJoint(characterHandle, childHandle, &joint);
    }
    CharacterControlMode controlMode = CHARACTER_KINEMATIC;
    R3KinematicCharacterController *controller = NULL;
    R3PidController *pid = NULL;
    controller = r3NewKinematicCharacterController();
    pid = r3NewPidController();
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    tbSetWorld(testbed, world);
    testbed->snapshotSupported = 0;

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            updateCharacter(testbed, world, &controlMode, controller, pid, characterHandle);
        }
    }
    r3FreePidController(pid);
    r3FreeKinematicCharacterController(controller);
    r3FreeWorld(world);
}
