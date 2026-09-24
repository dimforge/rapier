/* Port of examples3d/vehicle_controller3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbVehicleController3(Testbed *testbed) {
    R3World *world = r3NewWorld();
    {
        R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, -0.1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(5, 0.1, 5));
        R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(rigidBodyHandle, &collider);
    }
    const R3Real hw = .3, hh = .15;
    R3RigidBodyHandle vehicleHandle;
    {
        R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
        rigidBody.position.translation = r3Vector(0, 1, 0);
        rigidBody.canSleep = !testbed->noSleep;
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(hw * 2, hh, hw));
        collider.density = 100;
        vehicleHandle = r3InsertRigidBody(world, &rigidBody);
        r3InsertCollider(vehicleHandle, &collider);
    }

    R3WheelTuning tuning = r3DefaultWheelTuning();
    tuning.suspension_stiffness = 100;
    tuning.suspension_damping = 10;
    R3DynamicRayCastVehicleController *vehicle =
        r3NewDynamicRayCastVehicleController(vehicleHandle);
    const R3Vector wheelPositions[] = {
        {hw * 1.5, -hh, hw}, {hw * 1.5, -hh, -hw}, {-hw * 1.5, -hh, hw}, {-hw * 1.5, -hh, -hw}};
    for (size_t i = 0; i < TB_COUNT(wheelPositions); ++i) {
        r3DynamicRayCastVehicleController_AddWheel(vehicle, wheelPositions[i], r3Vector(0, -1, 0),
                                                  r3Vector(0, 0, 1), hh, hh / 4, &tuning);
    }
    for (int j = 0; j < 1; ++j) {
        for (int k = 0; k < 4; ++k) {
            for (int i = 0; i < 8; ++i) {
                {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation =
                        r3Vector(i * .2 - .8, j * .2 + .1, k * .2 + .8);
                    rigidBody.canSleep = !testbed->noSleep;
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.1, 0.1, 0.1));
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);
                }
            }
        }
    }
    {
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, .1, 5));
        collider.position.translation = r3Vector(7, 0.3, 0);
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), 0.2);
        r3InsertColliderWithoutParent(world, &collider);
    }
    {
        R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(2, .1, 5));
        collider.position.translation = r3Vector(10.1, 2.2, 0);
        collider.position.rotation = r3RotationFromAxisAngle(r3Vector(0, 0, 1), 0.9);
        r3InsertColliderWithoutParent(world, &collider);
    }
    R3Real heights[21 * 21];
    for (int j = 0; j <= 20; ++j) {
        for (int i = 0; i <= 20; ++i) {
            heights[i + j * 21] = -cos(i * .25) - cos(j * .25);
        }
    }
    R3ColliderDesc collider = r3DefaultColliderDesc();
    collider.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    collider.shape.heights = (R3RealView){heights, (21) * (21)};
    collider.shape.rows = 21;
    collider.shape.columns = 21;
    collider.shape.scale = r3Vector(10, .4, 10);
    collider.shape.flags = 0;
    collider.position.translation = r3Vector(-7, 0, 0);
    r3InsertColliderWithoutParent(world, &collider);

    tbCamera(testbed, 10, 10, 10, 0, 0, 0);
    testbed->snapshotSupported = 0;
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            const R3Real engineForce = 30 * testbed->inputDirection.y;
            const R3Real steeringAngle = -.7 * testbed->inputDirection.x;
            for (size_t i = 0; i < 2; ++i) {
                r3DynamicRayCastVehicleController_SetWheelControls(vehicle, i, steeringAngle,
                                                                  engineForce, 0);
            }
            /* The chassis colliders are always excluded by the controller. */
            R3QueryOptions options = r3DefaultQueryOptions();
            options.filter.flags = R3_QUERY_EXCLUDE_DYNAMIC;

            R3Real dt = r3TimeStep(world);
            r3DynamicRayCastVehicleController_UpdateVehicle(vehicle, dt, &options);
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeDynamicRayCastVehicleController(vehicle);
    r3FreeWorld(world);
}
