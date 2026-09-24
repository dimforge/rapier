/* Port of examples3d/utils/character.rs. Shared by the character and tether demos. */
#ifndef EXAMPLE_CHARACTER_3D_H
#define EXAMPLE_CHARACTER_3D_H
#include "testbed.h"
#include "rapier_math.h"

typedef enum CharacterControlMode { CHARACTER_KINEMATIC, CHARACTER_PID } CharacterControlMode;

static void updateCharacter(Testbed *viewer, R3World *world, CharacterControlMode *controlMode,
                            R3KinematicCharacterController *controller, R3PidController *pid,
                            R3RigidBodyHandle characterHandle) {
    R3Real dt = r3TimeStep(world);

    static const char *const modes[] = {"Kinematic", "PID"};
    const CharacterControlMode mode =
        (CharacterControlMode)tbChoice(viewer, "Control mode", 0, modes, TB_COUNT(modes), 1, 0);
    if (mode != *controlMode) {
        r3RigidBody_SetBodyType(characterHandle,
                               mode == CHARACTER_KINEMATIC ? R3_KINEMATIC_POSITION_BASED
                                                           : R3_DYNAMIC,
                               mode == CHARACTER_PID);
        *controlMode = mode;
    }
    R3Real speed = tbLiveSetting(viewer, "Character speed", .1, 0, 1, 0);
    if (viewer->slow) {
        speed /= 10;
    }
    R3Vector desiredMovement =
        r3VectorAdd(r3VectorScale(viewer->cameraRight, viewer->inputDirection.x),
                    r3VectorScale(viewer->cameraForward, viewer->inputDirection.y));
    desiredMovement.y = (viewer->jump ? 2 : 0) - (viewer->descend ? 1 : 0);
    desiredMovement = r3VectorScale(desiredMovement, speed);
    R3Vector translation = r3RigidBody_Translation(characterHandle);
    if (mode == CHARACTER_PID) {
        R3PidGains gains = r3PidController_Gains(pid);
        const R3Real linKp = tbLiveSetting(viewer, "Linear Kp", 60, 0, 100, 0);
        gains.lin_kp = r3Vector(linKp, linKp, linKp);
        const R3Real linKi = tbLiveSetting(viewer, "Linear Ki", 1, 0, 10, 0);
        gains.lin_ki = r3Vector(linKi, linKi, linKi);
        const R3Real linKd = tbLiveSetting(viewer, "Linear Kd", 0.8, 0, 1, 0);
        gains.lin_kd = r3Vector(linKd, linKd, linKd);
        const R3Real angKp = tbLiveSetting(viewer, "Angular Kp", 60, 0, 100, 0);
        gains.ang_kp = r3Vector(angKp, angKp, angKp);
        const R3Real angKi = tbLiveSetting(viewer, "Angular Ki", 1, 0, 10, 0);
        gains.ang_ki = r3Vector(angKi, angKi, angKi);
        const R3Real angKd = tbLiveSetting(viewer, "Angular Kd", 0.8, 0, 1, 0);
        gains.ang_kd = r3Vector(angKd, angKd, angKd);
        r3PidController_SetGains(pid, gains);
        uint32_t axes = 56; /* Angular axes. */
        if (desiredMovement.x != 0 || desiredMovement.y != 0 || desiredMovement.z != 0) {
            axes |= desiredMovement.y == 0 ? 5 : 7;
        }
        r3PidController_SetAxes(pid, axes);
        R3Pose target = r3TranslationPose(r3VectorAdd(translation, desiredMovement));
        target.rotation = r3RigidBody_Rotation(characterHandle);
        R3Vector correctiveLinear, linvel;
        R3AngVector correctiveAngular, angvel;
        R3VelocityCorrection pidControllerRigidBodyCorrectionResult = r3PidController_RigidBodyCorrection(pid, dt, characterHandle, target, r3Vector(0, 0, 0), r3Vector(0, 0, 0));
        correctiveLinear = pidControllerRigidBodyCorrectionResult.linear;
        correctiveAngular = pidControllerRigidBodyCorrectionResult.angularVelocity;
        linvel = r3RigidBody_Linvel(characterHandle);
        angvel = r3RigidBody_Angvel(characterHandle);
        r3RigidBody_SetLinvel(characterHandle, r3VectorAdd(linvel, correctiveLinear), 1);
        r3RigidBody_SetAngvel(characterHandle, r3VectorAdd(angvel, correctiveAngular), 1);
        return;
    }
    /* Kinematic character settings, applied live. */
    R3CharacterControllerSettings settings = r3KinematicCharacterController_Settings(controller);
    settings.slide = (R3Bool)tbLiveSetting(viewer, "Slide", settings.slide, 0, 1, 1);
    settings.max_slope_climb_angle = tbLiveSetting(
        viewer, "Maximum climb angle", settings.max_slope_climb_angle, 0, 2 * R3_PI, 0);
    settings.min_slope_slide_angle = tbLiveSetting(
        viewer, "Minimum slide angle", settings.min_slope_slide_angle, 0, R3_PI / 2, 0);
    settings.snap_to_ground =
        (R3Bool)tbLiveSetting(viewer, "Snap to ground", settings.snap_to_ground, 0, 1, 1);
    settings.snap_distance.value = tbLiveSetting(viewer, "Snap distance (relative height)",
                                                 settings.snap_distance.value, 0, 10, 0);
    r3KinematicCharacterController_SetSlide(controller, settings.slide);
    r3KinematicCharacterController_SetSlopes(controller, settings.max_slope_climb_angle,
                                            settings.min_slope_slide_angle);
    r3KinematicCharacterController_SetSnapToGround(controller, settings.snap_to_ground,
                                                  settings.snap_distance);
    desiredMovement.y -= speed; /* Artificial gravity, as in the Rust utility. */
    size_t colliderCount = r3RigidBody_Colliders(characterHandle, NULL, 0);
    R3ColliderHandle *handles = malloc(colliderCount * sizeof(*handles));
    if (!colliderCount || !handles) {
        abort();
    }
    colliderCount = r3RigidBody_Colliders(characterHandle, handles, colliderCount);

    const R3ColliderHandle colliderHandle = handles[0];
    free(handles);
    R3Pose pose;
    R3SharedShape *shape = NULL;
    R3Real mass;
    pose = r3Collider_Position(colliderHandle);
    shape = r3Collider_CloneShape(colliderHandle);
    mass = r3RigidBody_Mass(characterHandle);
    R3QueryFilter filter = r3DefaultQueryFilter();
    filter.exclude_rigid_body = characterHandle;
    R3QueryOptions query = r3DefaultQueryOptions();
    query.filter = filter;
    R3CharacterMovement movement = r3KinematicCharacterController_MoveShape(world, &query, controller, dt, shape, pose, desiredMovement);

    tbBodyColor(viewer, characterHandle, movement.grounded ? .1f : .8f,
                movement.grounded ? .8f : .1f, .1f, 1);
    r3KinematicCharacterController_SolveCharacterCollisionImpulses(controller, shape, dt,
                                                                  mass, &filter);
    r3FreeSharedShape(shape);

    r3RigidBody_SetNextKinematicTranslation(characterHandle,
                                           r3VectorAdd(translation, movement.translation));
}
#endif
