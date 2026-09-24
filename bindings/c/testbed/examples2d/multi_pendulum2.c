/* Port of examples2d/multi_pendulum2.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "example_math.h"

void tbMultiPendulum2(Testbed *testbed) {
    /* World. */
    R2World *world = r2NewWorld();

    r2SetGravity(world, r2Vector(0, -9.81));
    int randomize = (int)tbSetting(testbed, "Randomize", 0, 0, 1, 1);
    int count = (int)tbSetting(testbed, "Pendulum Count", 32, 1, 64, 1);
    int segments = (int)tbSetting(testbed, "Pendulum Segments", 4, 1, 40, 1);
    R2Real spacing = 2 * segments + 4;
    int cols = (int)ceil(sqrt(count));
    int rows = (count + cols - 1) / cols;
    for (int i = 0; i < count; i++) {
        R2Vector end = r2Vector((i % cols - (cols - 1) * 0.5) * spacing,
                                ((rows - 1) * 0.5 - i / cols) * spacing);
        R2Vector local = r2Vector(0, 0);
        R2RigidBodyHandle parent;
        R2RigidBodyDesc groundBody = r2FixedRigidBodyDesc();
        groundBody.position.translation = end;
        groundBody.canSleep = !testbed->noSleep;
        parent = r2InsertRigidBody(world, &groundBody);

        for (int n = 0; n < segments; n++) {
            R2Real angle = randomize ? (exampleRandom(&testbed->randomState) - 0.5) * R2_PI : 0;
            R2RigidBodyDesc rigidBody = r2DynamicRigidBodyDesc();
            rigidBody.position.translation = end;
            rigidBody.position = r2Pose(end, r2Rotation(angle));
            rigidBody.canSleep = 0;
            R2SharedShape *shape = r2CapsuleSharedShape(r2Vector(-1, 0), r2Vector(1, 0), 0.2);
            R2ColliderDesc collider = r2DefaultColliderDesc();
            collider.shape.kind = R2_SHAPE_DESC_SHARED;
            collider.shape.sharedShape = shape;
            collider.position.translation = r2Vector(1, 0);
            R2RigidBodyHandle handle;
            if (testbed->noSleep) {
                rigidBody.canSleep = 0;
                rigidBody.sleeping = 0;
            }
            handle = r2InsertRigidBody(world, &rigidBody);
            r2InsertCollider(handle, &collider);
            R2JointDesc joint = r2RevoluteJointDesc();
            joint.localFrame1.translation = local;
            joint.localFrame2.translation = r2Vector(0, 0);
            joint.contactsEnabled = 0;
            r2InsertImpulseJoint(parent, handle, &joint);
            parent = handle;
            local = r2Vector(2, 0);
            end = r2VectorAdd(end, r2Vector(2 * cos(angle), 2 * sin(angle)));

            r2FreeSharedShape(shape);
        }
    }
    /* Set up the viewer. */
    tbCamera2(testbed, 0, 0, (float)fmin(1000 / ((cols > rows ? cols : rows) * spacing), 20));

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r2Step(world, NULL, NULL);
        }
    }
    r2FreeWorld(world);
}
