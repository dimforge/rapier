/* Port of examples3d/heightfield3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbHeightfield3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    R3Real heights[441];
    for (int j = 0; j <= 20; j++) {
        for (int i = 0; i <= 20; i++) {
            heights[i + j * 21] =
                i == 0 || i == 20 || j == 0 || j == 20 ? 10 : (R3Real)(sin(i * 5) + cos(j * 5));
        }
    }
    R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
    rigidBody.position.translation = r3Vector(0, 0, 0);
    R3ColliderDesc groundCollider = r3DefaultColliderDesc();
    groundCollider.shape.kind = R3_SHAPE_DESC_HEIGHTFIELD;
    groundCollider.shape.heights = (R3RealView){heights, (21) * (21)};
    groundCollider.shape.rows = 21;
    groundCollider.shape.columns = 21;
    groundCollider.shape.scale = r3Vector(100, 1, 100);
    groundCollider.shape.flags = 0;
    rigidBody.canSleep = !testbed->noSleep;
    R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
    r3InsertCollider(rigidBodyHandle, &groundCollider);

    for (int j = 0; j < 20; j++) {
        for (int i = 0; i < 8; i++) {
            for (int k = 0; k < 8; k++) {
                R3ColliderDesc collider;
                switch (j % 6) {
                case 0:
                    collider = r3CuboidColliderDesc(r3Vector(1, 1, 1));
                    break;
                case 1:
                    collider = r3BallColliderDesc(1);
                    break;
                case 2:
                    collider = r3RoundCylinderColliderDesc(1, 1, 0.1);
                    break;
                case 3:
                    collider = r3ConeColliderDesc(1, 1);
                    break;
                case 4:
                    collider = r3CapsuleYColliderDesc(1, 1);
                    break;
                default: {
                    R3Pose poses[] = {r3TranslationPose(r3Vector(0, 0, 0)),
                                      r3TranslationPose(r3Vector(1, 0, 0)),
                                      r3TranslationPose(r3Vector(-1, 0, 0))};
                    R3SharedShape *shapes[3] = {NULL};
                    shapes[0] = r3CuboidSharedShape(r3Vector(1.0, 0.5, 0.5));
                    shapes[1] = r3CuboidSharedShape(r3Vector(0.5, 1.0, 0.5));
                    shapes[2] = r3CuboidSharedShape(r3Vector(0.5, 1.0, 0.5));
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
                    R3SharedShape *compoundShape = r3ShapeDesc_Build(&collider.shape);
                    collider.shape.kind = R3_SHAPE_DESC_SHARED;
                    collider.shape.sharedShape = compoundShape;
                    for (size_t part = 0; part < TB_COUNT(shapes); part++) {
                        r3FreeSharedShape(shapes[part]);
                    }
                    break;
                }
                }
                R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                rigidBody.position.translation = r3Vector(i * 3 - 12, j * 3 + 4.5, k * 3 - 12);
                rigidBody.canSleep = !testbed->noSleep;
                R3RigidBodyHandle rigidBodyHandle = r3InsertRigidBody(world, &rigidBody);
                r3InsertCollider(rigidBodyHandle, &collider);
                if (collider.shape.kind == R3_SHAPE_DESC_SHARED) {
                    r3FreeSharedShape((R3SharedShape *)collider.shape.sharedShape);
                }
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 100, 100, 100, 0, 0, 0);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
