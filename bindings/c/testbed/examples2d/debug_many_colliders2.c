/* Port of examples2d/debug_many_colliders2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

static const R2Vector part0[] = {
    {(R2Real)(525.0 * 0.01), (R2Real)(104.0 * 0.01)},
    {(R2Real)(540.0 * 0.01), (R2Real)(104.0 * 0.01)},
    {(R2Real)(419.0 * 0.01), (R2Real)(119.0 * 0.01)},
};

static const R2Vector part1[] = {
    {(R2Real)(419.0 * 0.01), (R2Real)(119.0 * 0.01)},
    {(R2Real)(449.0 * 0.01), (R2Real)(74.0 * 0.01)},
    {(R2Real)(510.0 * 0.01), (R2Real)(59.0 * 0.01)},
    {(R2Real)(525.0 * 0.01), (R2Real)(104.0 * 0.01)},
};

static const R2Vector part2[] = {
    {(R2Real)(299.0 * 0.01), (R2Real)(134.0 * 0.01)},
    {(R2Real)(419.0 * 0.01), (R2Real)(119.0 * 0.01)},
    {(R2Real)(540.0 * 0.01), (R2Real)(104.0 * 0.01)},
    {(R2Real)(540.0 * 0.01), (R2Real)(134.0 * 0.01)},
};

static const R2Vector part3[] = {
    {(R2Real)(315.0 * 0.01), (R2Real)(450.0 * 0.01)},
    {(R2Real)(179.0 * 0.01), (R2Real)(284.0 * 0.01)},
    {(R2Real)(179.0 * 0.01), (R2Real)(254.0 * 0.01)},
    {(R2Real)(224.0 * 0.01), (R2Real)(224.0 * 0.01)},
};

static const R2Vector part4[] = {
    {(R2Real)(224.0 * 0.01), (R2Real)(224.0 * 0.01)},
    {(R2Real)(224.0 * 0.01), (R2Real)(223.0 * 0.01)},
    {(R2Real)(299.0 * 0.01), (R2Real)(134.0 * 0.01)},
    {(R2Real)(540.0 * 0.01), (R2Real)(134.0 * 0.01)},
    {(R2Real)(555.0 * 0.01), (R2Real)(134.0 * 0.01)},
    {(R2Real)(555.0 * 0.01), (R2Real)(209.0 * 0.01)},
    {(R2Real)(359.0 * 0.01), (R2Real)(465.0 * 0.01)},
    {(R2Real)(315.0 * 0.01), (R2Real)(450.0 * 0.01)},
};

static const R2Vector part5[] = {
    {(R2Real)(119.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(134.0 * 0.01), (R2Real)(314.0 * 0.01)},
    {(R2Real)(179.0 * 0.01), (R2Real)(284.0 * 0.01)},
    {(R2Real)(315.0 * 0.01), (R2Real)(450.0 * 0.01)},
    {(R2Real)(315.0 * 0.01), (R2Real)(465.0 * 0.01)},
    {(R2Real)(300.0 * 0.01), (R2Real)(465.0 * 0.01)},
};

static const R2Vector part6[] = {
    {(R2Real)(164.0 * 0.01), (R2Real)(510.0 * 0.01)},
    {(R2Real)(134.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(240.0 * 0.01), (R2Real)(510.0 * 0.01)},
};

static const R2Vector part7[] = {
    {(R2Real)(240.0 * 0.01), (R2Real)(510.0 * 0.01)},
    {(R2Real)(240.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(164.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(164.0 * 0.01), (R2Real)(510.0 * 0.01)},
};

static const R2Vector part8[] = {
    {(R2Real)(134.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(104.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(119.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(300.0 * 0.01), (R2Real)(465.0 * 0.01)},
    {(R2Real)(270.0 * 0.01), (R2Real)(510.0 * 0.01)},
    {(R2Real)(240.0 * 0.01), (R2Real)(510.0 * 0.01)},
};

static const R2Vector part9[] = {
    {(R2Real)(615.0 * 0.01), (R2Real)(269.0 * 0.01)},
    {(R2Real)(660.0 * 0.01), (R2Real)(284.0 * 0.01)},
    {(R2Real)(660.0 * 0.01), (R2Real)(359.0 * 0.01)},
};

static const R2Vector part10[] = {
    {(R2Real)(673.6813186813187 * 0.01), (R2Real)(390.010989010989 * 0.01)},
    {(R2Real)(660.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(675.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(675.0 * 0.01), (R2Real)(389.0 * 0.01)},
};

static const R2Vector part11[] = {
    {(R2Real)(675.0 * 0.01), (R2Real)(389.0 * 0.01)},
    {(R2Real)(735.0 * 0.01), (R2Real)(434.0 * 0.01)},
    {(R2Real)(735.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(720.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(673.6813186813187 * 0.01), (R2Real)(390.010989010989 * 0.01)},
};

static const R2Vector part12[] = {
    {(R2Real)(645.0 * 0.01), (R2Real)(540.0 * 0.01)},
    {(R2Real)(645.0 * 0.01), (R2Real)(555.0 * 0.01)},
    {(R2Real)(494.0 * 0.01), (R2Real)(555.0 * 0.01)},
    {(R2Real)(494.0 * 0.01), (R2Real)(540.0 * 0.01)},
};

static const R2Vector part13[] = {
    {(R2Real)(705.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(645.0 * 0.01), (R2Real)(540.0 * 0.01)},
    {(R2Real)(494.0 * 0.01), (R2Real)(540.0 * 0.01)},
    {(R2Real)(464.0 * 0.01), (R2Real)(540.0 * 0.01)},
    {(R2Real)(464.0 * 0.01), (R2Real)(525.0 * 0.01)},
};

static const R2Vector part14[] = {
    {(R2Real)(660.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(720.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(705.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(464.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(434.0 * 0.01), (R2Real)(525.0 * 0.01)},
    {(R2Real)(434.0 * 0.01), (R2Real)(510.0 * 0.01)},
};

static const R2Vector part15[] = {
    {(R2Real)(660.0 * 0.01), (R2Real)(359.0 * 0.01)},
    {(R2Real)(434.0 * 0.01), (R2Real)(510.0 * 0.01)},
    {(R2Real)(404.0 * 0.01), (R2Real)(510.0 * 0.01)},
    {(R2Real)(404.0 * 0.01), (R2Real)(495.0 * 0.01)},
};

static const R2Vector part16[] = {
    {(R2Real)(404.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(359.0 * 0.01), (R2Real)(495.0 * 0.01)},
    {(R2Real)(359.0 * 0.01), (R2Real)(465.0 * 0.01)},
    {(R2Real)(555.0 * 0.01), (R2Real)(209.0 * 0.01)},
    {(R2Real)(570.0 * 0.01), (R2Real)(209.0 * 0.01)},
    {(R2Real)(615.0 * 0.01), (R2Real)(269.0 * 0.01)},
    {(R2Real)(660.0 * 0.01), (R2Real)(359.0 * 0.01)},
};

static R2ColliderDesc polygon(const R2Vector *position, size_t n) {
    R2ColliderDesc collider = r2DefaultColliderDesc();
    r2ShapeDesc_SetConvexHull(&collider.shape, (R2VectorView){position, n});
    return collider;
}

void tbDebugManyColliders2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, 0));
    R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
    rigidBody.position.translation = r2Vector(0, 0);
    rigidBody.canSleep = 0;
    rigidBody.angvel = 1;
    R2RigidBodyHandle handle;
    if (testbed->noSleep) {
        rigidBody.canSleep = 0;
        rigidBody.sleeping = 0;
    }
    handle = r2InsertRigidBody(world, &rigidBody);
    for (int i = 0; i < 130; i++) {
        R2ColliderDesc collider = polygon(part0, TB_COUNT(part0));
        r2InsertCollider(handle, &collider);
        collider = polygon(part1, TB_COUNT(part1));
        r2InsertCollider(handle, &collider);
        collider = polygon(part2, TB_COUNT(part2));
        r2InsertCollider(handle, &collider);
        collider = polygon(part3, TB_COUNT(part3));
        r2InsertCollider(handle, &collider);
        collider = polygon(part4, TB_COUNT(part4));
        r2InsertCollider(handle, &collider);
        collider = polygon(part5, TB_COUNT(part5));
        r2InsertCollider(handle, &collider);
        collider = polygon(part6, TB_COUNT(part6));
        r2InsertCollider(handle, &collider);
        collider = polygon(part7, TB_COUNT(part7));
        r2InsertCollider(handle, &collider);
        collider = polygon(part8, TB_COUNT(part8));
        r2InsertCollider(handle, &collider);
        collider = polygon(part9, TB_COUNT(part9));
        r2InsertCollider(handle, &collider);
        collider = polygon(part10, TB_COUNT(part10));
        r2InsertCollider(handle, &collider);
        collider = polygon(part11, TB_COUNT(part11));
        r2InsertCollider(handle, &collider);
        collider = polygon(part12, TB_COUNT(part12));
        r2InsertCollider(handle, &collider);
        collider = polygon(part13, TB_COUNT(part13));
        r2InsertCollider(handle, &collider);
        collider = polygon(part14, TB_COUNT(part14));
        r2InsertCollider(handle, &collider);
        collider = polygon(part15, TB_COUNT(part15));
        r2InsertCollider(handle, &collider);
        collider = polygon(part16, TB_COUNT(part16));
        r2InsertCollider(handle, &collider);
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 5, 3, 40);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
