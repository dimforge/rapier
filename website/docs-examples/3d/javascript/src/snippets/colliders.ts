import RAPIER, { Vector3, ColliderDesc } from '@dimforge/rapier3d';

{
    let vertices = new Float32Array([-1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, 1.0, 0.0]);
    let indices = new Uint32Array([0, 2, 1]);
    let heights = new Float32Array([0.0, 1.0, 0.5, 0.9]);
    let scale = new Vector3(1.0, 1.0, 1.0);

    // DOCUSAURUS: Creation start
    // The physics world.
    let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });

    // Builder for a ball-shaped collider.
    let example1 = RAPIER.ColliderDesc.ball(0.5);
    // Builder for a cuboid-shaped collider.
    let example2 = RAPIER.ColliderDesc.cuboid(0.5, 0.2, 0.1);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    let example3 = RAPIER.ColliderDesc.capsule(0.5, 0.2);
    // Builder for a triangle-mesh-shaped collider.
    let example4 = RAPIER.ColliderDesc.trimesh(vertices, indices);
    // Builder for a heightfield-shaped collider.
    let example5 = RAPIER.ColliderDesc.heightfield(2, 2, heights, scale);
    // Builder for a collider with the given shape.
    let colliderDesc = new RAPIER.ColliderDesc(new RAPIER.Ball(0.5))
        // The collider translation wrt. the body it is attached to.
        // Default: the zero vector.
        .setTranslation(1.0, 2.0, 3.0)
        // The collider rotation wrt. the body it is attached to, as a unit quaternion.
        // Default: the identity rotation.
        .setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 })
        // The collider density. If non-zero the collider's mass and angular inertia will be added
        // to the inertial properties of the body it is attached to.
        // Default: 1.0
        .setDensity(1.3)
        // The friction coefficient of this collider.
        // Default: 0.5
        .setFriction(0.8)
        // Whether this collider is a sensor.
        // Default: false
        .setSensor(true);

    // Create the collider, without attaching it to a rigid-body.
    let handle = world.createCollider(colliderDesc);
    // Or create the collider and attach it to a rigid-body.
    let rigidBody = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
    let collider = world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Creation stop

    let position = rigidBody.translation();
    console.log("Rigid-body position: ", position.x, position.y);
}

let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
{
    // DOCUSAURUS: Mass start
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 2.0, 1.0)
        .setDensity(2.0);
    // Second option: by setting the mass of the collider.
    let colliderDesc2 = RAPIER.ColliderDesc.cuboid(1.0, 2.0, 1.0)
        .setMass(0.8);
    // Third option: by setting the mass-properties explicitly.
    let colliderDesc3 = RAPIER.ColliderDesc.cuboid(1.0, 2.0, 1.0)
        .setMassProperties(0.5, { x: 0.0, y: 1.0, z: 0.0 }, { x: 0.3, y: 0.2, z: 0.1 }, { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    let collider = world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Mass stop
}

{
    // DOCUSAURUS: Position1 start
    /* Set the collider position when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setTranslation(1.0, 2.0, 3.0)
        .setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the collider position after the collider creation. */
    collider.setTranslation({ x: 1.0, y: 2.0, z: 3.0 });
    collider.setRotation({ w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    // DOCUSAURUS: Position2 stop
}
{
    // DOCUSAURUS: Position3 start
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setTranslation(1.0, 2.0, 3.0);
    // Attach the collider to the rigid-body. The collider's position wrt. the rigid-body
    // is automatically set to the collider current position when this method is called.
    let collider = world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Position3 stop

    // DOCUSAURUS: Position4 start
    /* Set the collider position wrt. its parent after the collider creation. */
    collider.setTranslationWrtParent({ x: 1.0, y: 2.0, z: 3.0 });
    // DOCUSAURUS: Position4 stop
}