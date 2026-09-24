#include "snippets.h"
#include <math.h>

// DOCUSAURUS: DebugColor start
// The lines are given with HSLA colors. A renderer expecting RGBA colors has to convert them.
static void hsla_to_rgba(const float hsla[4], float rgba[4]) {
    float h = fmodf(hsla[0], 360.0f) / 60.0f;
    float c = (1.0f - fabsf(2.0f * hsla[2] - 1.0f)) * hsla[1];
    float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
    float m = hsla[2] - c / 2.0f;
    float r = 0.0f, g = 0.0f, b = 0.0f;

    if (h < 1.0f) {
        r = c;
        g = x;
    } else if (h < 2.0f) {
        r = x;
        g = c;
    } else if (h < 3.0f) {
        g = c;
        b = x;
    } else if (h < 4.0f) {
        g = x;
        b = c;
    } else if (h < 5.0f) {
        r = x;
        b = c;
    } else {
        r = c;
        b = x;
    }

    rgba[0] = r + m;
    rgba[1] = g + m;
    rgba[2] = b + m;
    rgba[3] = hsla[3];
}
// DOCUSAURUS: DebugColor stop

int main(void) {
    snippets_init(); /* aborts on any Rapier error, so the ctest fails */
    R2World *world = r2NewWorld();
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);
    R2RigidBodyDesc body = r2DynamicRigidBodyDesc();
    body.position.translation = r2Vector(0.0, 1.0);
    R2RigidBodyHandle body_handle = r2InsertRigidBody(world, &body);
    R2ColliderDesc ball = r2BallColliderDesc(0.5);
    r2InsertCollider(body_handle, &ball);

    // DOCUSAURUS: DebugRender start
    // The mode selects what is drawn.
    uint32_t mode = R2_DEBUG_COLLIDER_SHAPES | R2_DEBUG_CONTACTS;
    // The buffer receiving the lines is kept from one frame to the next.
    R2DebugLine *lines = NULL;
    size_t capacity = 0;
    size_t num_lines = 0;

    for (int i = 0; i < 10; i++) {
        r2Step(world, NULL, NULL);

        // The debug-rendering is done after the step, once per frame to be drawn:
        // get the number of lines, grow the buffer if needed, then copy the lines.
        num_lines = r2DebugRender(world, mode, NULL, 0);
        if (num_lines > capacity) {
            capacity = num_lines;
            lines = realloc(lines, capacity * sizeof(*lines));
        }
        num_lines = r2DebugRender(world, mode, lines, capacity);

        for (size_t j = 0; j < num_lines; j++) {
            float rgba[4];
            hsla_to_rgba(lines[j].color, rgba);
            // Give the segment from `lines[j].a` to `lines[j].b`, with the color `rgba`,
            // to the renderer of the application.
        }
    }

    printf("%zu lines to draw\n", num_lines);
    free(lines);
    // DOCUSAURUS: DebugRender stop

    // DOCUSAURUS: DebugRenderStyle start
    // The style gives the colors (in HSLA) and sizes of the lines.
    R2DebugRenderStyle style = r2DefaultDebugRenderStyle();
    // Draw the colliders attached to dynamic rigid-bodies in blue.
    style.collider_dynamic_color[0] = 240.0f;
    style.collider_dynamic_color[1] = 1.0f;
    style.collider_dynamic_color[2] = 0.5f;
    style.collider_dynamic_color[3] = 1.0f;
    // Draw longer contact normals.
    style.contact_normal_length = 0.5;

    num_lines = r2DebugRenderWithStyle(world, mode, &style, NULL, 0);
    lines = malloc(num_lines * sizeof(*lines));
    num_lines = r2DebugRenderWithStyle(world, mode, &style, lines, num_lines);
    // DOCUSAURUS: DebugRenderStyle stop
    printf("%zu lines drawn with the custom style\n", num_lines);
    free(lines);

    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
