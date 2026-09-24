/* Port of examples2d/s2d_arch.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbS2dArch(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    const R2Vector ps1[] = {r2Vector(16.0 * 0.25, 0.0 * 0.25),
                            r2Vector(14.93803712795643 * 0.25, 5.133601056842984 * 0.25),
                            r2Vector(13.79871746027416 * 0.25, 10.24928069555078 * 0.25),
                            r2Vector(12.56252963284711 * 0.25, 15.34107019122473 * 0.25),
                            r2Vector(11.20040987372525 * 0.25, 20.39856541571217 * 0.25),
                            r2Vector(9.66521217819836 * 0.25, 25.40369899225096 * 0.25),
                            r2Vector(7.87179930638133 * 0.25, 30.3179337000085 * 0.25),
                            r2Vector(5.635199558196225 * 0.25, 35.03820717801641 * 0.25),
                            r2Vector(2.405937953536585 * 0.25, 39.09554102558315 * 0.25)};
    const R2Vector ps2[] = {r2Vector(24.0 * 0.25, 0.0 * 0.25),
                            r2Vector(22.33619528222415 * 0.25, 6.02299846205841 * 0.25),
                            r2Vector(20.54936888969905 * 0.25, 12.00964361211476 * 0.25),
                            r2Vector(18.60854610798073 * 0.25, 17.9470321677465 * 0.25),
                            r2Vector(16.46769273811807 * 0.25, 23.81367936585418 * 0.25),
                            r2Vector(14.05325025774858 * 0.25, 29.57079353071012 * 0.25),
                            r2Vector(11.23551045834022 * 0.25, 35.13775818285372 * 0.25),
                            r2Vector(7.752568160730571 * 0.25, 40.30450679009583 * 0.25),
                            r2Vector(3.016931552701656 * 0.25, 44.28891593799322 * 0.25)};
    R2SharedShape *shape = r2SegmentSharedShape(r2Vector(-100, 0), r2Vector(100, 0));
    R2ColliderDesc floor = r2DefaultColliderDesc();
    floor.shape.kind = R2_SHAPE_DESC_SHARED;
    floor.shape.sharedShape = shape;
    floor.friction = 0.6;
    r2InsertColliderWithoutParent(world, &floor);
    for (int side = 0; side < 2; side++) {
        for (int i = 0; i < 8; i++) {
            R2Vector vertices[4];
            if (!side) {
                vertices[0] = ps1[i];
                vertices[1] = ps2[i];
                vertices[2] = ps2[i + 1];
                vertices[3] = ps1[i + 1];
            } else {
                vertices[0] = r2Vector(-ps2[i].x, ps2[i].y);
                vertices[1] = r2Vector(-ps1[i].x, ps1[i].y);
                vertices[2] = r2Vector(-ps1[i + 1].x, ps1[i + 1].y);
                vertices[3] = r2Vector(-ps2[i + 1].x, ps2[i + 1].y);
            }
            R2ColliderDesc collider = r2DefaultColliderDesc();
            r2ShapeDesc_SetConvexHull(&collider.shape, (R2VectorView){vertices, 4});
            collider.friction = 0.6;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(0, 0);
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    R2Vector position[] = {ps1[8], ps2[8], r2Vector(-ps1[8].x, ps1[8].y),
                           r2Vector(-ps2[8].x, ps2[8].y)};
    R2ColliderDesc objectCollider = r2DefaultColliderDesc();
    r2ShapeDesc_SetConvexHull(&objectCollider.shape, (R2VectorView){position, 4});
    objectCollider.friction = 0.6;
    R2RigidBodyDesc dynamicBody = r2DynamicRigidBodyDesc();
    dynamicBody.position.translation = r2Vector(0, 0);
    dynamicBody.canSleep = !testbed->noSleep;
    R2RigidBodyHandle dynamicBodyHandle = r2InsertRigidBody(world, &dynamicBody);
    r2InsertCollider(dynamicBodyHandle, &objectCollider);

    for (int i = 0; i < 4; i++) {
        objectCollider = r2CuboidColliderDesc(r2Vector(2, 0.5));
        objectCollider.friction = 0.6;
        R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
        rigidBody.position.translation = r2Vector(0, 0.5 + ps2[8].y + i);
        rigidBody.canSleep = !testbed->noSleep;
        R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
        r2InsertCollider(rigidBodyHandle, &objectCollider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 2.5, 20);

    r2FreeSharedShape(shape);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
