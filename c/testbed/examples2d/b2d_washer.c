/* Port of examples2d/b2d_washer.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbB2dWasher(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -10));
    R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
    groundBody.position.translation = r2Vector(0, 0);
    groundBody.canSleep = !testbed->noSleep;
    r2InsertRigidBody(world, &groundBody);

    R2RigidBodyDesc rigidBody = r2KinematicVelocityBasedRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 10);
    rigidBody.angvel = R2_PI / 180 * 25;
    rigidBody.linvel = r2Vector(0.001, -0.002);
    R2RigidBodyHandle washer;
    rigidBody.canSleep = !testbed->noSleep;
    washer = r2InsertRigidBody(world, &rigidBody);
    R2Real angle = R2_PI / 18;
    R2Vector u1 = r2Vector(1, 0);
    for (int i = 0; i < 36; i++) {
        R2Vector u2 = i == 35 ? r2Vector(1, 0)
                              : r2Vector(cos(angle) * u1.x - sin(angle) * u1.y,
                                         sin(angle) * u1.x + cos(angle) * u1.y);
        R2Vector a1 = r2Vector(cos(angle * 0.1) * u1.x + sin(angle * 0.1) * u1.y,
                               -sin(angle * 0.1) * u1.x + cos(angle * 0.1) * u1.y);
        R2Vector a2 = r2Vector(cos(angle * 0.1) * u2.x - sin(angle * 0.1) * u2.y,
                               sin(angle * 0.1) * u2.x + cos(angle * 0.1) * u2.y);
        for (int part = 0; part < (i % 9 == 0 ? 2 : 1); part++) {
            R2Vector vertices[4];
            R2Vector left = part ? u1 : a1;
            R2Vector right = part ? u2 : a2;
            R2Real rmin = part ? 14 : 16;
            R2Real rmax = part ? 16 : 18;
            vertices[0] = r2VectorScale(left, rmin);
            vertices[1] = r2VectorScale(left, rmax);
            vertices[2] = r2VectorScale(right, rmin);
            vertices[3] = r2VectorScale(right, rmax);
            R2ColliderDesc collider = r2DefaultColliderDesc();
            r2ShapeDesc_SetConvexHull(&collider.shape, (R2VectorView){vertices, TB_COUNT(vertices)});
            r2InsertCollider(washer, &collider);
        }
        u1 = u2;
    }
    for (int i = 0; i < 90; i++) {
        for (int j = 0; j < 90; j++) {
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = r2Vector(-9.9 + j * 0.21, 0.1 + i * 0.21);
            R2ColliderDesc collider = r2CuboidColliderDesc(r2Vector(0.1, 0.1));
            rigidBody.canSleep = !testbed->noSleep;
            R2RigidBodyHandle rigidBodyHandle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(rigidBodyHandle, &collider);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 10, 12);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
