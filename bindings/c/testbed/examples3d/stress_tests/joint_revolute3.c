/* Port of examples3d/stress_tests/joint_revolute3.rs. */
#include "testbed.h"
#include "rapier_helpers.h"
#include "rapier_math.h"

void tbStressTestsJointRevolute3(Testbed *testbed) {
    /* World. */
    R3World *world = r3NewWorld();

    for (int l = 0; l < 4; l++) {
        for (int j = 0; j < 50; j++) {
            R3Real x = j * 8;
            R3Real y = l * 60;
            R3RigidBodyHandle parent;
            R3RigidBodyDesc rigidBody = r3FixedRigidBodyDesc();
            rigidBody.position.translation = r3Vector(x, y, 0);
            R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
            rigidBody.canSleep = !testbed->noSleep;
            parent = r3InsertRigidBody(world, &rigidBody);
            r3InsertCollider(parent, &collider);
            for (int i = 0; i < 10; i++) {
                R3Real z = i * 4 + 2;
                R3Vector positions[] = {r3Vector(x, y, z), r3Vector(x + 2, y, z),
                                        r3Vector(x + 2, y, z + 2), r3Vector(x, y, z + 2)};
                R3Vector axes[] = {r3Vector(0, 0, 1), r3Vector(1, 0, 0), r3Vector(0, 0, 1),
                                   r3Vector(1, 0, 0)};
                R3Vector anchors[] = {r3Vector(0, 0, -2), r3Vector(-2, 0, 0), r3Vector(0, 0, -2),
                                      r3Vector(2, 0, 0)};
                R3RigidBodyHandle handles[4];
                for (int k = 0; k < 4; k++) {
                    R3RigidBodyDesc rigidBody = r3DynamicRigidBodyDesc();
                    rigidBody.position.translation = positions[k];
                    R3ColliderDesc collider = r3CuboidColliderDesc(r3Vector(0.4, 0.4, 0.4));
                    rigidBody.canSleep = !testbed->noSleep;
                    handles[k] = r3InsertRigidBody(world, &rigidBody);
                    r3InsertCollider(handles[k], &collider);
                }
                for (int k = 0; k < 4; k++) {
                    R3JointDesc joint = r3RevoluteJointDesc(axes[k]);
                    joint.localFrame1.translation = r3Vector(0, 0, 0);
                    joint.localFrame2.translation = anchors[k];
                    r3InsertImpulseJoint(k ? handles[k - 1] : parent, handles[k], &joint);
                }
                parent = handles[3];
            }
        }
    }
    /* Set up the viewer. */
    tbCamera(testbed, 478, 83, 228, 134, 83, -116);

    /* Set up rendering and run the simulation. */
    tbSetWorld(testbed, world);

    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
