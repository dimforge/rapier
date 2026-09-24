#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);
    R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
    body.position.translation = r3Vector(0.0, 1.0, 0.0);
    R3RigidBodyHandle body_handle = r3InsertRigidBody(world, &body);
    R3ColliderDesc collider = r3BallColliderDesc(0.5);
    r3InsertCollider(body_handle, &collider);

    // DOCUSAURUS: Pid start
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    R3PidController *pid = r3NewPidController();
    R3PidGains gains = r3PidController_Gains(pid);
    gains.lin_kp = r3Vector(60.0, 60.0, 60.0);
    gains.lin_ki = r3Vector(0.0, 0.0, 0.0);
    gains.lin_kd = r3Vector(0.8, 0.8, 0.8);
    r3PidController_SetGains(pid, gains);
    r3PidController_SetAxes(pid, R3_AXES_MASK_LIN_X | R3_AXES_MASK_LIN_Y | R3_AXES_MASK_LIN_Z);
    R3Vector target = r3Vector(3.0, 2.0, 0.0);

    for (int i = 0; i < 200; i++) {
        R3Real dt = r3TimeStep(world);
        // The correction is the velocity change bringing the body closer to its target pose.
        R3VelocityCorrection correction = r3PidController_RigidBodyCorrection(
            pid, dt, body_handle,
            r3TranslationPose(target), // The target pose.
            r3Vector(0.0, 0.0, 0.0),   // The target linear velocity.
            r3Vector(0.0, 0.0, 0.0));  // The target angular velocity.
        R3Vector linvel = r3VectorAdd(r3RigidBody_Linvel(body_handle), correction.linear);
        R3AngVector angvel = r3VectorAdd(r3RigidBody_Angvel(body_handle), correction.angularVelocity);
        r3RigidBody_SetLinvel(body_handle, linvel, 1);
        r3RigidBody_SetAngvel(body_handle, angvel, 1);

        r3Step(world, NULL, NULL);
    }

    r3FreePidController(pid);
    // DOCUSAURUS: Pid stop

    R3Vector translation = r3RigidBody_Translation(body_handle);
    printf("Body position: (%f, %f, %f)\n", (double)translation.x, (double)translation.y,
           (double)translation.z);

    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
