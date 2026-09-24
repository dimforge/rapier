#include "snippets.h"

/* Adapted from the basic simulation example of the user guide. */
static R2World *setup_physics_scene(void) {
    R2World *world = r2NewWorld();
    r2SetGravity(world, r2Vector(0.0, -9.81));

    /* Create the ground. */
    R2ColliderDesc ground = r2CuboidColliderDesc(r2Vector(100.0, 0.1));
    r2InsertColliderWithoutParent(world, &ground);

    /* Create the bouncing ball. */
    R2RigidBodyDesc ball_body = r2DynamicRigidBodyDesc();
    ball_body.position.translation = r2Vector(0.0, 10.0);
    R2RigidBodyHandle ball_body_handle = r2InsertRigidBody(world, &ball_body);
    R2ColliderDesc ball = r2BallColliderDesc(0.5);
    ball.restitution = 0.7;
    r2InsertCollider(ball_body_handle, &ball);

    /* Run the simulation for a while before serializing it. */
    for (int i = 0; i < 200; i++) {
        r2Step(world, NULL, NULL);
    }

    return world;
}

int main(void) {
    snippets_init();
    R2World *world = setup_physics_scene();

    // DOCUSAURUS: Serialization start
    /* Serialize the whole physics world. */
    R2Bytes *snapshot = r2SerializeWorld(world);
    /* The serialized bytes, borrowed from the snapshot, e.g., to be written to a file. */
    R2ByteView bytes = r2Bytes_Data(snapshot);
    printf("The snapshot takes %zu bytes.\n", bytes.count);

    /* Deserialize it: this creates a new world, independent from the original one. */
    R2World *deserialized = r2DeserializeWorld(bytes.data, bytes.count);
    r2FreeBytes(snapshot);

    /* The simulation can continue using the deserialized world. */
    r2Step(deserialized, NULL, NULL);
    // DOCUSAURUS: Serialization stop

    // DOCUSAURUS: RestoredHandles start
    /* The handles of the original world don't refer to the deserialized world: get new ones. */
    size_t num_bodies = r2RigidBodyHandles(deserialized, NULL, 0);
    R2RigidBodyHandle *bodies = malloc(num_bodies * sizeof(R2RigidBodyHandle));
    num_bodies = r2RigidBodyHandles(deserialized, bodies, num_bodies);

    for (size_t i = 0; i < num_bodies; i++) {
        printf("Restored rigid-body {%u, %u} at altitude %f\n", bodies[i].index, bodies[i].generation,
               (double)r2RigidBody_Translation(bodies[i]).y);
    }
    free(bodies);
    // DOCUSAURUS: RestoredHandles stop

    r2FreeWorld(deserialized);
    r2FreeWorld(world);
    return EXIT_SUCCESS;
}
