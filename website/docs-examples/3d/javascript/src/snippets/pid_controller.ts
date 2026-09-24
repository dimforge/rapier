import RAPIER from '@dimforge/rapier3d';

{
    let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
    world.createCollider(RAPIER.ColliderDesc.cuboid(100.0, 0.1, 100.0));
    let body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 1.0, 0.0));
    world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

    // DOCUSAURUS: Pid start
    // The proportional, integral, and derivative gains of the controller, acting on the linear
    // axes only: the body is pushed toward its target without its rotation being controlled.
    let pid = world.createPidController(60.0, 0.0, 0.8, RAPIER.PidAxesMask.AllLin);
    let target = { x: 3.0, y: 2.0, z: 0.0 };

    for (let k = 0; k < 200; ++k) {
        // The correction is applied to the velocity of the rigid-body.
        pid.applyLinearCorrection(body, target, { x: 0.0, y: 0.0, z: 0.0 });
        world.step();
    }
    // DOCUSAURUS: Pid stop
}
