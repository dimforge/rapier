/* Port of examples3d/vehicle_joints3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbVehicleJoints3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    R3Real heights[101 * 101];
    for (int j = 0; j <= 100; ++j) {
        for (int i = 0; i <= 100; ++i) {
            heights[i + j * 101] = -cos(i * .3) - cos(j * .3);
        }
    }
    R3ColliderDesc ground = r3DefaultColliderDesc();
    ground.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    ground.shape.heights = (R3RealView){heights, (101) * (101)};
    ground.shape.rows = 101;
    ground.shape.columns = 101;
    ground.shape.scale = r3Vector(60, .4, 60);
    ground.shape.flags = 0;
    ground.position.translation = r3Vector(-7, 0, 0);
    ground.friction = 1;
    r3InsertColliderWithoutParent(world, &ground);

    const R3InteractionGroups carGroups = {1, ~UINT32_C(1), 0};
    const R3Vector wheelParams[] = {{.6874, .2783, -.7802},
                                    {-.6874, .2783, -.7802},
                                    {.64, .2783, 1.0254},
                                    {-.64, .2783, 1.0254}};
    const R3Real suspensionHeight = .12, maxSteeringAngle = 35 * R3_PI / 180;
    const R3Real driveStrength = 1, wheelRadius = .28;
    const R3Vector carPosition = {0, wheelRadius + suspensionHeight, 0};
    const R3Vector bodyPositionInCarSpace = {0, .4739, 0};
    R3RigidBodyHandle bodyHandle;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3VectorAdd(carPosition, bodyPositionInCarSpace);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.65, 0.3, 0.9));
        collider.density = 100;
        collider.collisionGroups = carGroups;
        bodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(bodyHandle, &collider);
    }

    R3ImpulseJointHandle steeringJoints[2], motorJoints[2];
    for (size_t wheelId = 0; wheelId < TB_COUNT(wheelParams); ++wheelId) {
        const int isFront = wheelId >= 2;
        const R3Vector wheelCenter = r3VectorAdd(carPosition, wheelParams[wheelId]);
        R3SharedShape *ball = r3BallSharedShape(wheelRadius);
        R3MassProperties axleMassProps = r3SharedShape_MassProperties(ball, 100);
        r3FreeSharedShape(ball);
        R3RigidBodyDesc axleRb = r3DynamicRigidBodyDesc();
        axleRb.position.translation = wheelCenter;
        axleRb.canSleep = !testbed->noSleep;
        R3RigidBodyHandle axleHandle = r3InsertRigidBody(world, &axleRb);

        r3RigidBody_SetAdditionalMassProperties(axleHandle, axleMassProps, 1);
        R3RigidBodyHandle wheelHandle;
        {
            R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
            rigidBody.position.translation = wheelCenter;
            rigidBody.canSleep = !testbed->noSleep;
            R3ColliderDesc collider = r3BallColliderDesc(wheelRadius);
            collider.density = 100;
            collider.collisionGroups = carGroups;
            collider.friction = 1;
            wheelHandle = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(wheelHandle, &collider);
        }
        R3ColliderDesc wheelFakeCo = r3CylinderColliderDesc(wheelRadius / 2, wheelRadius);
        wheelFakeCo.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), R3_PI / 2);
        wheelFakeCo.isSensor = 1;
        wheelFakeCo.density = 0;
        wheelFakeCo.collisionGroups = (R3InteractionGroups){0, 0, 0};
        r3InsertCollider(wheelHandle, &wheelFakeCo);

        uint8_t lockedAxes = 1 | 4 | 8 | 32;
        if (!isFront) {
            lockedAxes |= 16;
        }
        R3JointDesc suspensionJoint = r3DefaultJointDesc();
        suspensionJoint.lockedAxes = lockedAxes;
        r3JointDesc_SetLimits(&suspensionJoint, R3_AXIS_LIN_Y, 0, suspensionHeight);
        r3JointDesc_SetMotorPosition(&suspensionJoint, R3_AXIS_LIN_Y, 0, 1e4, 1e3);
        suspensionJoint.localFrame1.translation =
            r3VectorSub(wheelParams[wheelId], bodyPositionInCarSpace);
        if (isFront) {
            r3JointDesc_SetLimits(&suspensionJoint, R3_AXIS_ANG_Y, -maxSteeringAngle,
                                 maxSteeringAngle);
        }
        R3ImpulseJointHandle bodyAxleJointHandle =
            r3InsertImpulseJoint(bodyHandle, axleHandle, &suspensionJoint);

        R3JointDesc wheelJoint = r3RevoluteJointDesc(r3Vector(1, 0, 0));
        R3ImpulseJointHandle wheelJointHandle =
            r3InsertImpulseJoint(axleHandle, wheelHandle, &wheelJoint);

        if (isFront) {
            steeringJoints[wheelId - 2] = bodyAxleJointHandle;
            motorJoints[wheelId - 2] = wheelJointHandle;
        }
    }
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
            const R3Real thrust = -driveStrength * testbed->inputDirection.y;
            const R3Real steering = -testbed->inputDirection.x;
            const R3Real boost = testbed->boost ? 1.5 : 1;
            const R3Bool shouldWakeUp = thrust != 0 || steering != 0;
            for (size_t i = 0; i < TB_COUNT(steeringJoints); ++i) {
                r3ImpulseJoint_SetMotorPosition(steeringJoints[i], R3_AXIS_ANG_Y,
                                               maxSteeringAngle * steering, 1e4, 1e3, shouldWakeUp);
            }
            const R3Real sidewaysShift = sin(maxSteeringAngle * steering) * .5;
            const R3Real speedDiff =
                sidewaysShift > 0 ? hypot(1, sidewaysShift) : 1 / hypot(1, sidewaysShift);
            const R3Real ms[] = {1 / speedDiff, speedDiff};
            for (size_t i = 0; i < TB_COUNT(motorJoints); ++i) {
                r3ImpulseJoint_SetMotorVelocity(motorJoints[i], R3_AXIS_ANG_X,
                                               -30 * thrust * ms[i] * boost, 1e2, shouldWakeUp);
            }
        }
    }
    r3FreeWorld(world);
}
