#include "snippets.h"

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R3World *world = r3NewWorld();
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    // DOCUSAURUS: Vehicle start
    // The chassis is an ordinary dynamic rigid-body.
    const R3Real hw = 0.3;
    const R3Real hh = 0.15;
    R3RigidBodyDesc chassis_body = r3DynamicRigidBodyDesc();
    chassis_body.position.translation = r3Vector(0.0, 1.0, 0.0);
    R3RigidBodyHandle chassis_handle = r3InsertRigidBody(world, &chassis_body);
    R3ColliderDesc chassis_collider = r3CuboidColliderDesc(r3Vector(hw * 2.0, hh, hw));
    chassis_collider.density = 100.0;
    r3InsertCollider(chassis_handle, &chassis_collider);

    // The tuning shared by the wheels: the suspension and the grip.
    R3WheelTuning tuning = r3DefaultWheelTuning();
    tuning.suspension_stiffness = 100.0;
    tuning.suspension_damping = 10.0;

    // The controller must be freed (with r3FreeDynamicRayCastVehicleController) before its world.
    R3DynamicRayCastVehicleController *vehicle = r3NewDynamicRayCastVehicleController(chassis_handle);
    const R3Vector wheel_positions[4] = {
        {hw * 1.5, -hh, hw},
        {hw * 1.5, -hh, -hw},
        {-hw * 1.5, -hh, hw},
        {-hw * 1.5, -hh, -hw},
    };

    for (size_t i = 0; i < 4; i++) {
        // The position of the wheel, the direction its suspension pushes along, its axle, the
        // rest length of its suspension, and its radius; all in the local frame of the chassis.
        r3DynamicRayCastVehicleController_AddWheel(vehicle, wheel_positions[i], r3Vector(0.0, -1.0, 0.0),
                                                   r3Vector(0.0, 0.0, 1.0), hh, hh / 4.0, &tuning);
    }
    // DOCUSAURUS: Vehicle stop

    // DOCUSAURUS: VehicleUpdate start
    // The wheels are ray-casted against the scene: the chassis itself is always excluded, and
    // every other dynamic body is generally excluded from these ray-casts too.
    R3QueryOptions options = r3DefaultQueryOptions();
    options.filter.flags = R3_QUERY_EXCLUDE_DYNAMIC;

    for (int i = 0; i < 200; i++) {
        // The vehicle is driven by setting the steering angle, the engine force, and the brake of
        // its wheels. Here the two front wheels (indices 0 and 1) are the driving and steering ones.
        r3DynamicRayCastVehicleController_SetWheelControls(vehicle, 0, 0.2, 30.0, 0.0);
        r3DynamicRayCastVehicleController_SetWheelControls(vehicle, 1, 0.2, 30.0, 0.0);

        r3DynamicRayCastVehicleController_UpdateVehicle(vehicle, r3TimeStep(world), &options);

        r3Step(world, NULL, NULL);
    }

    printf("Vehicle speed: %f\n", (double)r3DynamicRayCastVehicleController_CurrentVehicleSpeed(vehicle));
    // DOCUSAURUS: VehicleUpdate stop

    // DOCUSAURUS: VehicleWheels start
    // The wheels are given in the order they were added to the controller.
    R3WheelState wheels[4];
    size_t num_wheels = r3DynamicRayCastVehicleController_Wheels(vehicle, wheels, 4);
    for (size_t i = 0; i < num_wheels; i++) {
        // The world-space center of the wheel, its current suspension length, rotation angle, etc.
        printf("Wheel %zu: center (%f, %f, %f), suspension length %f, rotation %f, in contact: %u\n", i,
               (double)wheels[i].center.x, (double)wheels[i].center.y, (double)wheels[i].center.z,
               (double)wheels[i].suspension_length, (double)wheels[i].rotation,
               (unsigned)wheels[i].is_in_contact);
    }
    // DOCUSAURUS: VehicleWheels stop

    r3FreeDynamicRayCastVehicleController(vehicle);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
