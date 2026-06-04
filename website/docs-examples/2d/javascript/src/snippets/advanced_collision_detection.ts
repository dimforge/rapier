import RAPIER from '@dimforge/rapier2d';


let world = new RAPIER.World({ x: 0.0, y: -9.81 });
{

    // DOCUSAURUS: Events start
    let eventQueue = new RAPIER.EventQueue(true);
    world.step(eventQueue);

    eventQueue.drainCollisionEvents((handle1, handle2, started) => {
        /* Handle the collision event. */
    });

    eventQueue.drainContactForceEvents(event => {
        let handle1 = event.collider1(); // Handle of the first collider involved in the event.
        let handle2 = event.collider2(); // Handle of the second collider involved in the event.
        /* Handle the contact force event. */
    });
    // DOCUSAURUS: Events stop
}

let colliderDesc = new RAPIER.ColliderDesc(new RAPIER.Ball(0.5));
let collider1 = world.createCollider(colliderDesc);
let collider2 = world.createCollider(colliderDesc);
let collider = world.createCollider(colliderDesc);
{
    // DOCUSAURUS: ContactGraph start
    world.contactPairsWith(collider, (otherCollider) => {
        // This closure is called on each collider object potentially
        // in contact with `collider`.
    });

    world.contactPair(collider1, collider2, (manifold, flipped) => {
        // Contact information can be read from `manifold`. 
    });
    // DOCUSAURUS: ContactGraph stop
}

{
    // DOCUSAURUS: IntersectionGraph start
    world.intersectionPairsWith(collider, (otherCollider) => {
        // This closure is called on each collider potentially
        // intersecting the collider `collider`.
    });

    let intersections = world.intersectionPair(collider1, collider2);
    // DOCUSAURUS: IntersectionGraph stop
}
{
    // DOCUSAURUS: IntersectionTest start
    let shape = new RAPIER.Cuboid(1.0, 2.0);
    let shapePos = { x: 1.0, y: 2.0 };
    let shapeRot = 0.1;

    world.intersectionsWithShape(shapePos, shapeRot, shape, (handle) => {
        console.log("The collider", handle, "intersects our shape.");
        return true; // Return `false` instead if we want to stop searching for other colliders that contain this point.
    });

    let aabbCenter = { x: -1.0, y: -2.0 };
    let aabbHalfExtents = { x: 0.5, y: 0.6 };
    world.collidersWithAabbIntersectingAabb(aabbCenter, aabbHalfExtents, (handle) => {
        console.log("The collider", handle, "has an AABB intersecting our test AABB");
        return true; // Return `false` instead if we want to stop searching for other colliders that contain this point.
    });
    // DOCUSAURUS: IntersectionTest stop
}
