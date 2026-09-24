import RAPIER from '@dimforge/rapier2d';

{
    let world = new RAPIER.World({ x: 0.0, y: -9.81 });
    world.createCollider(RAPIER.ColliderDesc.cuboid(100.0, 0.1));
    let body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(0.0, 1.0));
    world.createCollider(RAPIER.ColliderDesc.ball(0.5), body);

    // DOCUSAURUS: DebugRender start
    for (let k = 0; k < 10; ++k) {
        world.step();

        // The buffers are the lines to be drawn: two floats per vertex, four per color, and two
        // vertices per line. They are meant to be given to the line renderer of your application.
        let buffers = world.debugRender();
        console.log(buffers.vertices.length / 4, "lines to draw");
    }
    // DOCUSAURUS: DebugRender stop
}
