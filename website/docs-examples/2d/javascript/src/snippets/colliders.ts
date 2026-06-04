import RAPIER, { Vector2, ColliderDesc } from '@dimforge/rapier2d';

{
    let vertices = new Float32Array([-1.0, -1.0, 1.0, -1.0, 1.0, 1.0]);
    let indices = new Uint32Array([0, 2, 1]);
    let heights = new Float32Array([0.0, 1.0, 0.5, 0.9]);
    let scale = new Vector2(1.0, 1.0);

    // DOCUSAURUS: Creation start
    // The physics world.
    let world = new RAPIER.World({ x: 0.0, y: -9.81 });

    // Builder for a ball-shaped collider.
    let example1 = RAPIER.ColliderDesc.ball(0.5);
    // Builder for a cuboid-shaped collider.
    let example2 = RAPIER.ColliderDesc.cuboid(0.5, 0.2);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    let example3 = RAPIER.ColliderDesc.capsule(0.5, 0.2);
    // Builder for a triangle-mesh-shaped collider.
    let example4 = RAPIER.ColliderDesc.trimesh(vertices, indices);
    // Builder for a heightfield-shaped collider.
    let example5 = RAPIER.ColliderDesc.heightfield(heights, scale);
    // Builder for a collider with the given shape.
    let colliderDesc = new RAPIER.ColliderDesc(new RAPIER.Ball(0.5))
        // The collider translation wrt. the body it is attached to.
        // Default: the zero vector.
        .setTranslation(1.0, 2.0)
        // The collider rotation wrt. the body it is attached to.
        // Default: the identity rotation.
        .setRotation(3.14)
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

let world = new RAPIER.World({ x: 0.0, y: -9.81 });
{
    // DOCUSAURUS: ColliderType1 start
    /* Set the collider type when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setSensor(true);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: ColliderType1 stop

    // DOCUSAURUS: ColliderType2 start
    /* Set the collider type after the collider creation. */
    collider.setSensor(true);
    // DOCUSAURUS: ColliderType2 stop
}

{
    // DOCUSAURUS: Mass start
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let colliderDesc = RAPIER.ColliderDesc.cuboid(1.0, 2.0)
        .setDensity(2.0);
    // Second option: by setting the mass of the collider.
    let colliderDesc2 = RAPIER.ColliderDesc.cuboid(1.0, 2.0)
        .setMass(0.8);
    // Third option: by setting the mass-properties explicitly.
    let colliderDesc3 = RAPIER.ColliderDesc.cuboid(1.0, 2.0)
        .setMassProperties(0.5, { x: 0.0, y: 1.0 }, 0.3);
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    let collider = world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Mass stop
}

{
    // DOCUSAURUS: Position1 start
    /* Set the collider position when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setTranslation(1.0, 2.0)
        .setRotation(0.4);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the collider position after the collider creation. */
    collider.setTranslation({ x: 1.0, y: 2.0 });
    collider.setRotation(0.4);
    // DOCUSAURUS: Position2 stop
}
{
    // DOCUSAURUS: Position3 start
    let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
    let rigidBody = world.createRigidBody(rigidBodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setTranslation(1.0, 2.0);
    // Attach the collider to the rigid-body. The collider's position wrt. the rigid-body
    // is automatically set to the collider current position when this method is called.
    let collider = world.createCollider(colliderDesc, rigidBody);
    // DOCUSAURUS: Position3 stop

    // DOCUSAURUS: Position4 start
    /* Set the collider position wrt. its parent after the collider creation. */
    collider.setTranslationWrtParent({ x: 1.0, y: 2.0 });
    // DOCUSAURUS: Position4 stop
}

{
    // DOCUSAURUS: Friction1 start
    /* Set the friction coefficient and friction combine rule
      when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setFriction(0.7)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Friction1 stop

    // DOCUSAURUS: Friction2 start
    /* Set the friction coefficient and friction combine rule
       after the collider creation. */
    collider.setFriction(0.7);
    collider.setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min);
    // DOCUSAURUS: Friction2 stop
}

{
    // DOCUSAURUS: Restitution1 start
    /* Set the restitution coefficient and restitution combine rule
      when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setRestitution(0.7)
        .setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Min);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Restitution1 stop

    // DOCUSAURUS: Restitution2 start
    /* Set the restitution coefficient and restitution combine rule
       after the collider creation. */
    collider.setRestitution(0.7);
    collider.setRestitutionCombineRule(RAPIER.CoefficientCombineRule.Min);
    // DOCUSAURUS: Restitution2 stop
}

{
    // DOCUSAURUS: Groups1 start
    /* Set the collision groups and solver groups when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setCollisionGroups(0x000D0004)
        .setSolverGroups(0x00500010);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: Groups1 stop

    // DOCUSAURUS: Groups2 start
    /* Set the collision groups and solver groups after the collider creation. */
    collider.setCollisionGroups(0x000D0004);
    collider.setSolverGroups(0x000D0004);
    // DOCUSAURUS: Groups2 stop
}

{
    // DOCUSAURUS: ActiveCollisionTypes1 start
    /* Set the active collision types when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setActiveCollisionTypes(RAPIER.ActiveCollisionTypes.DEFAULT |
            RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: ActiveCollisionTypes1 stop

    // DOCUSAURUS: ActiveCollisionTypes2 start
    /* Set the active collision types after the collider creation. */
    collider.setActiveCollisionTypes(RAPIER.ActiveCollisionTypes.DEFAULT |
        RAPIER.ActiveCollisionTypes.KINEMATIC_FIXED);
    // DOCUSAURUS: ActiveCollisionTypes2 stop
}

{
    // DOCUSAURUS: ActiveEvents1 start
    /* Set the active events when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: ActiveEvents1 stop

    // DOCUSAURUS: ActiveEvents2 start
    /* Set the active events after the collider creation. */
    collider.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    // DOCUSAURUS: ActiveEvents2 stop
}
{
    // DOCUSAURUS: ActiveHooks1 start
    /* Set the active hooks when the collider is created. */
    let colliderDesc = RAPIER.ColliderDesc.ball(0.5)
        .setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
    let collider = world.createCollider(colliderDesc);
    // DOCUSAURUS: ActiveHooks1 stop

    // DOCUSAURUS: ActiveHooks2 start
    /* Set the active hooks after the collider creation. */
    collider.setActiveHooks(RAPIER.ActiveHooks.FILTER_CONTACT_PAIRS);
    // DOCUSAURUS: ActiveHooks2 stop
}