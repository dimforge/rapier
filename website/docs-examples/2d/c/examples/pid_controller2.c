#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = r2Vector(0.0, 1.0);
    R2RigidBodyHandle body_handle = r2InsertRigidBody(world, &body);
    R2ColliderDesc collider = r2BallColliderDesc(0.5);
    r2InsertCollider(body_handle, &collider);

    // DOCUSAURUS: Pid start
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    R2PidController *pid = r2NewPidController();
    R2PidGains gains = r2PidController_Gains(pid);
    gains.lin_kp = r2Vector(60.0, 60.0);
    gains.lin_ki = r2Vector(0.0, 0.0);
    gains.lin_kd = r2Vector(0.8, 0.8);
    r2PidController_SetGains(pid, gains);
    r2PidController_SetAxes(pid, R2_AXES_MASK_LIN_X | R2_AXES_MASK_LIN_Y);
    R2Vector target = r2Vector(3.0, 2.0);

    for (int i = 0; i < 200; i++) {
        R2Real dt = r2TimeStep(world);
        // The correction is the velocity change bringing the body closer to its target pose.
        R2VelocityCorrection correction = r2PidController_RigidBodyCorrection(
            pid, dt, body_handle,
            r2TranslationPose(target), // The target pose.
            r2Vector(0.0, 0.0),        // The target linear velocity.
            0.0);                      // The target angular velocity.
        R2Vector linvel = r2VectorAdd(r2RigidBody_Linvel(body_handle), correction.linear);
        R2AngVector angvel = r2RigidBody_Angvel(body_handle) + correction.angularVelocity;
        r2RigidBody_SetLinvel(body_handle, linvel, 1);
        r2RigidBody_SetAngvel(body_handle, angvel, 1);

        r2Step(world, NULL, NULL);
    }

    r2FreePidController(pid);
    // DOCUSAURUS: Pid stop

    R2Vector translation = r2RigidBody_Translation(body_handle);
    printf("Body position: (%f, %f)\n", (double)translation.x, (double)translation.y);

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
