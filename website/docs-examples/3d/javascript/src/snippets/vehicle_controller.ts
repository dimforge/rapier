import RAPIER from '@dimforge/rapier3d';

{
    let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
    world.createCollider(RAPIER.ColliderDesc.cuboid(100.0, 0.1, 100.0));

    // DOCUSAURUS: Vehicle start
    // The chassis is an ordinary dynamic rigid-body.
    let hw = 0.3;
    let hh = 0.15;
    let chassis = world.createRigidBody(
        RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 1.0, 0.0),
    );
    world.createCollider(RAPIER.ColliderDesc.cuboid(hw * 2.0, hh, hw).setDensity(100.0), chassis);

    let vehicle = world.createVehicleController(chassis);
    let wheelPositions = [
        { x: hw * 1.5, y: -hh, z: hw }, { x: hw * 1.5, y: -hh, z: -hw },
        { x: -hw * 1.5, y: -hh, z: hw }, { x: -hw * 1.5, y: -hh, z: -hw },
    ];

    for (let position of wheelPositions) {
        // The position of the wheel, the direction its suspension pushes along, its axle, the
        // rest length of its suspension, and its radius; all in the local frame of the chassis.
        vehicle.addWheel(position, { x: 0.0, y: -1.0, z: 0.0 }, { x: 0.0, y: 0.0, z: 1.0 }, hh, hh / 4.0);
    }

    // The tuning of each wheel: its suspension and its grip.
    for (let i = 0; i < vehicle.numWheels(); ++i) {
        vehicle.setWheelSuspensionStiffness(i, 100.0);
        vehicle.setWheelSuspensionCompression(i, 10.0);
        vehicle.setWheelSuspensionRelaxation(i, 10.0);
    }
    // DOCUSAURUS: Vehicle stop

    // DOCUSAURUS: VehicleUpdate start
    for (let k = 0; k < 200; ++k) {
        // The vehicle is driven by setting the engine force, the brake, and the steering angle of
        // its wheels. Here the two front wheels are the driving and steering ones.
        vehicle.setWheelEngineForce(0, 30.0);
        vehicle.setWheelSteering(0, 0.2);
        vehicle.setWheelEngineForce(1, 30.0);
        vehicle.setWheelSteering(1, 0.2);

        // The wheels are ray-casted against the scene: the dynamic bodies, including the chassis
        // itself, are generally excluded from these ray-casts.
        vehicle.updateVehicle(world.integrationParameters.dt, RAPIER.QueryFilterFlags.EXCLUDE_DYNAMIC);
        world.step();
    }

    console.log("Vehicle speed:", vehicle.currentVehicleSpeed());
    // DOCUSAURUS: VehicleUpdate stop
}
