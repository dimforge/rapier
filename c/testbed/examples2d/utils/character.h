/* Port of examples2d/utils/character.rs. Shared by the character and tether demos. */
#ifndef EXAMPLE_CHARACTER_2D_H
#define EXAMPLE_CHARACTER_2D_H
#include "testbed.h"
#include "rapier_math.h"

typedef enum CharacterControlMode { CHARACTER_KINEMATIC, CHARACTER_PID } CharacterControlMode;

static void updateCharacter(Testbed *viewer, R2World *world, CharacterControlMode *controlMode,
                            R2KinematicCharacterController *controller, R2PidController *pid,
                            R2RigidBodyHandle characterHandle) {
    R2Real dt = r2TimeStep(world);

    static const char *const modes[] = {"Kinematic", "PID"};
    const CharacterControlMode mode =
        (CharacterControlMode)tbChoice(viewer, "Control mode", 0, modes, TB_COUNT(modes), 1, 0);
    if (mode != *controlMode) {
        r2RigidBody_SetBodyType(characterHandle,
                               mode == CHARACTER_KINEMATIC ? R2_KINEMATIC_POSITION_BASED
                                                           : R2_DYNAMIC,
                               mode == CHARACTER_PID);
        *controlMode = mode;
    }
    R2Real speed = tbLiveSetting(viewer, "Character speed", .1, 0, 1, 0);
    if (viewer->slow) {
        speed /= 10;
    }
    R2Vector desiredMovement = r2Vector(viewer->inputDirection.x, 0);
    desiredMovement.y = (viewer->jump ? 2 : 0) - (viewer->descend ? 1 : 0);
    desiredMovement = r2VectorScale(desiredMovement, speed);
    R2Vector translation = r2RigidBody_Translation(characterHandle);
    if (mode == CHARACTER_PID) {
        R2PidGains gains = r2PidController_Gains(pid);
        const R2Real linKp = tbLiveSetting(viewer, "Linear Kp", 60, 0, 100, 0);
        gains.lin_kp = r2Vector(linKp, linKp);
        const R2Real linKi = tbLiveSetting(viewer, "Linear Ki", 1, 0, 10, 0);
        gains.lin_ki = r2Vector(linKi, linKi);
        const R2Real linKd = tbLiveSetting(viewer, "Linear Kd", 0.8, 0, 1, 0);
        gains.lin_kd = r2Vector(linKd, linKd);
        const R2Real angKp = tbLiveSetting(viewer, "Angular Kp", 60, 0, 100, 0);
        gains.ang_kp = angKp;
        const R2Real angKi = tbLiveSetting(viewer, "Angular Ki", 1, 0, 10, 0);
        gains.ang_ki = angKi;
        const R2Real angKd = tbLiveSetting(viewer, "Angular Kd", 0.8, 0, 1, 0);
        gains.ang_kd = angKd;
        r2PidController_SetGains(pid, gains);
        uint32_t axes = 32; /* Angular axes. */
        if (desiredMovement.x != 0 || desiredMovement.y != 0) {
            axes |= desiredMovement.y == 0 ? 1 : 3;
        }
        r2PidController_SetAxes(pid, axes);
        R2Pose target = r2TranslationPose(r2VectorAdd(translation, desiredMovement));
        R2Vector correctiveLinear, linvel;
        R2AngVector correctiveAngular, angvel;
        R2VelocityCorrection pidControllerRigidBodyCorrectionResult = r2PidController_RigidBodyCorrection(pid, dt, characterHandle, target, r2Vector(0, 0), 0);
        correctiveLinear = pidControllerRigidBodyCorrectionResult.linear;
        correctiveAngular = pidControllerRigidBodyCorrectionResult.angularVelocity;
        linvel = r2RigidBody_Linvel(characterHandle);
        angvel = r2RigidBody_Angvel(characterHandle);
        r2RigidBody_SetLinvel(characterHandle, r2VectorAdd(linvel, correctiveLinear), 1);
        r2RigidBody_SetAngvel(characterHandle, angvel + correctiveAngular, 1);
        return;
    }
    /* Kinematic character settings, applied live. */
    R2CharacterControllerSettings settings = r2KinematicCharacterController_Settings(controller);
    settings.slide = (R2Bool)tbLiveSetting(viewer, "Slide", settings.slide, 0, 1, 1);
    settings.max_slope_climb_angle = tbLiveSetting(
        viewer, "Maximum climb angle", settings.max_slope_climb_angle, 0, 2 * R2_PI, 0);
    settings.min_slope_slide_angle = tbLiveSetting(
        viewer, "Minimum slide angle", settings.min_slope_slide_angle, 0, R2_PI / 2, 0);
    settings.snap_to_ground =
        (R2Bool)tbLiveSetting(viewer, "Snap to ground", settings.snap_to_ground, 0, 1, 1);
    settings.snap_distance.value = tbLiveSetting(viewer, "Snap distance (relative height)",
                                                 settings.snap_distance.value, 0, 10, 0);
    r2KinematicCharacterController_SetSlide(controller, settings.slide);
    r2KinematicCharacterController_SetSlopes(controller, settings.max_slope_climb_angle,
                                            settings.min_slope_slide_angle);
    r2KinematicCharacterController_SetSnapToGround(controller, settings.snap_to_ground,
                                                  settings.snap_distance);
    desiredMovement.y -= speed; /* Artificial gravity, as in the Rust utility. */
    size_t colliderCount = r2RigidBody_Colliders(characterHandle, NULL, 0);
    R2ColliderHandle *handles = malloc(colliderCount * sizeof(*handles));
    if (!colliderCount || !handles) {
        abort();
    }
    colliderCount = r2RigidBody_Colliders(characterHandle, handles, colliderCount);

    const R2ColliderHandle colliderHandle = handles[0];
    free(handles);
    R2Pose pose;
    R2SharedShape *shape = NULL;
    R2Real mass;
    pose = r2Collider_Position(colliderHandle);
    shape = r2Collider_CloneShape(colliderHandle);
    mass = r2RigidBody_Mass(characterHandle);
    R2QueryFilter filter = r2DefaultQueryFilter();
    filter.exclude_rigid_body = characterHandle;
    R2QueryOptions query = r2DefaultQueryOptions();
    query.filter = filter;
    R2CharacterMovement movement = r2KinematicCharacterController_MoveShape(world, &query, controller, dt, shape, pose, desiredMovement);

    tbBodyColor(viewer, characterHandle, movement.grounded ? .1f : .8f,
                movement.grounded ? .8f : .1f, .1f, 1);
    r2KinematicCharacterController_SolveCharacterCollisionImpulses(controller, shape, dt,
                                                                  mass, &query);
    r2FreeSharedShape(shape);

    r2RigidBody_SetNextKinematicTranslation(characterHandle,
                                           r2VectorAdd(translation, movement.translation));
}
#endif
