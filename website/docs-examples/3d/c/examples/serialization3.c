#include "snippets.h"

/* Adapted from the basic simulation example of the user guide. */
static R3World *setup_physics_scene(void) {
    R3World *world = r3NewWorld();
    r3SetGravity(world, r3Vector(0.0, -9.81, 0.0));

    /* Create the ground. */
    R3ColliderDesc ground = r3CuboidColliderDesc(r3Vector(100.0, 0.1, 100.0));
    r3InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R3RigidBodyDesc ball_body = r3DynamicRigidBodyDesc();
    ball_body.position.translation = r3Vector(0.0, 10.0, 0.0);
    R3RigidBodyHandle ball_body_handle = r3InsertRigidBody(world, &ball_body);
    R3ColliderDesc ball = r3BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r3InsertCollider(ball_body_handle, &ball);

    /* Run the simulation for a while before serializing it. */
    for (int i = 0; i < 200; i++) {
        r3Step(world, NULL, NULL);
    }

    return world;
}

int main(void) {
    snippets_init();
    R3World *world = setup_physics_scene();

    // DOCUSAURUS: Serialization start
    /* Serialize the whole physics world. */
    R3Bytes *snapshot = r3SerializeWorld(world);
    /* The serialized bytes, borrowed from the snapshot, e.g., to be written to a file. */
    R3ByteView bytes = r3Bytes_Data(snapshot);
    printf("The snapshot takes %zu bytes.\n", bytes.count);

    /* Deserialize it: this creates a new world, independent from the original one. */
    R3World *deserialized = r3DeserializeWorld(bytes.data, bytes.count);
    r3FreeBytes(snapshot);

    /* The simulation can continue using the deserialized world. */
    r3Step(deserialized, NULL, NULL);
    // DOCUSAURUS: Serialization stop

    // DOCUSAURUS: RestoredHandles start
    /* The handles of the original world don't refer to the deserialized world: get new ones. */
    size_t num_bodies = r3RigidBodyHandles(deserialized, NULL, 0);
    R3RigidBodyHandle *bodies = malloc(num_bodies * sizeof(R3RigidBodyHandle));
    num_bodies = r3RigidBodyHandles(deserialized, bodies, num_bodies);

    for (size_t i = 0; i < num_bodies; i++) {
        printf("Restored rigid-body {%u, %u} at altitude %f\n", bodies[i].index, bodies[i].generation,
               (double)r3RigidBody_Translation(bodies[i]).y);
    }
    free(bodies);
    // DOCUSAURUS: RestoredHandles stop

    r3FreeWorld(deserialized);
    r3FreeWorld(world);
    return EXIT_SUCCESS;
}
