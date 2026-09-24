/* Compiled against the header of the C testbed, with a stub viewer rendering 100 frames, since the
 * user-guide snippets are not linked to the testbed itself. */
#include "snippets.h"
#include "../../../../../c/testbed/testbed.h"

int tbRenderFrame(Testbed *testbed, R3World **world) {
    testbed->world = *world;
    return testbed->step++ < 100;
}
int tbSimulating(Testbed *testbed) {
    (void)testbed;
    return 1;
}
void tbSetWorld(Testbed *testbed, R3World *world) {
    testbed->world = world;
}
void tbCamera(Testbed *testbed, float x, float y, float z, float tx, float ty, float tz) {
    (void)testbed;
    (void)x, (void)y, (void)z, (void)tx, (void)ty, (void)tz;
}

// DOCUSAURUS: Scene start
void tbBouncingBall3(Testbed *testbed) {
    /* The scene itself, built like in any other application. */
    R3World *world = r3NewWorld();
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r3InsertCollider(r3InsertRigidBody(world, &ball_body), &ball);

    /* Hand the world to the viewer, and place the camera (eye, then target). */
    tbSetWorld(testbed, world);
    tbCamera(testbed, 10, 10, 10, 0, 0, 0);

    /* The rendering loop: it ends when the user closes the window or selects another scene. The
     * viewer may replace the world, e.g., when a snapshot is restored, hence its address. */
    while (tbRenderFrame(testbed, &world)) {
        if (tbSimulating(testbed)) {
            r3Step(world, NULL, NULL);
        }
    }
    r3FreeWorld(world);
}
// DOCUSAURUS: Scene stop

int main(void) {
    snippets_init();
    static Testbed testbed;

    // DOCUSAURUS: Registry start
    /* An entry of the `tbExamples` array: identifier, group, name, source, scene function, and the
     * reason why it is unavailable (NULL if it is available). */
    TbExample entry = {"bouncing_ball3", "Demos", "Bouncing ball", "examples3d/bouncing_ball3.c",
                       tbBouncingBall3, NULL};
    // DOCUSAURUS: Registry stop

    entry.run(&testbed);
    printf("Ran %llu frames of \"%s\".\n", (unsigned long long)testbed.step, entry.name);
    return EXIT_SUCCESS;
}
