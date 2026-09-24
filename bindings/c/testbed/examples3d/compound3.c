/* Port of examples3d/compound3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbCompound3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    /* Ground. */
    const R3Real groundSize = 50.0;
    const R3Real groundHeight = 0.1;
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0.0, -groundHeight, 0.0);
    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(groundSize, groundHeight, groundSize));
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &collider);

    /* Create the cubes. */
    const int num = 8;
    const int numy = 15;
    const R3Real rad = 0.2;
    const R3Real shift = rad * 4.0 + rad;
    const R3Real centerx = shift * (num / 2);
    const R3Real centery = shift / 2.0;
    const R3Real centerz = shift * (num / 2);
    R3Real offset = -2.4;

    for (int j = 0; j < numy; j++) {
        for (int i = 0; i < num; i++) {
            for (int k = 0; k < num; k++) {
                const R3Real x = i * shift * 5.0 - centerx + offset;
                const R3Real y = j * (shift * 5.0) + centery + 3.0;
                const R3Real z = k * shift * 2.0 - centerz + offset;
                rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(x, y, z);
                rigidBody.canSleep = !testbed->noSleep;

                /* First option: attach several colliders to a single rigid body. */
                if (j < numy / 2) {
                    R3ColliderDesc collider1 = r3CuboidColliderDesc(r3Vector(rad * 10.0, rad, rad));
                    R3ColliderDesc collider2 = r3CuboidColliderDesc(r3Vector(rad, rad * 10.0, rad));
                    collider2.position.translation = r3Vector(rad * 10.0, rad * 10.0, 0.0);
                    R3ColliderDesc collider3 = r3CuboidColliderDesc(r3Vector(rad, rad * 10.0, rad));
                    collider3.position.translation = r3Vector(-rad * 10.0, rad * 10.0, 0.0);
                    R3RigidBodyHandle handle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(handle, &collider1);
                    r3InsertCollider(handle, &collider2);
                    r3InsertCollider(handle, &collider3);
                } else {
                    /* Second option: attach one collider with a compound shape. */
                    const R3Pose poses[] = {
                        r3TranslationPose(r3Vector(0.0, 0.0, 0.0)),
                        r3TranslationPose(r3Vector(rad * 10.0, rad * 10.0, 0.0)),
                        r3TranslationPose(r3Vector(-rad * 10.0, rad * 10.0, 0.0)),
                    };
                    R3SharedShape *shapes[3] = {NULL};
                    shapes[0] = r3CuboidSharedShape(r3Vector(rad * 10.0, rad, rad));
                    shapes[1] = r3CuboidSharedShape(r3Vector(rad, rad * 10.0, rad));
                    shapes[2] = r3CuboidSharedShape(r3Vector(rad, rad * 10.0, rad));
                    R3CompoundShapeDesc colliderParts[TB_COUNT(shapes)];
                    for (size_t part = 0; part < TB_COUNT(shapes); ++part) {
                        colliderParts[part].pose = poses[part];
                        colliderParts[part].shape = r3DefaultShapeDesc();
                        colliderParts[part].shape.kind = R3_SHAPE_DESC_SHARED;
                        colliderParts[part].shape.sharedShape =
                            ((const R3SharedShape *const *)shapes)[part];
                    }
                    collider = r3DefaultColliderDesc();
                    collider.shape.kind = R3_SHAPE_DESC_COMPOUND;
                    collider.shape.children =
                        (R3CompoundShapeView){colliderParts, TB_COUNT(shapes)};
                    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(rigidBodyHandle, &collider);

                    for (size_t part = 0; part < TB_COUNT(shapes); part++) {
                        r3FreeSharedShape(shapes[part]);
                    }
                }
            }
        }
        offset -= 0.07;
    }

    /* Set up the viewer. */
    tbCamera(testbed, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
