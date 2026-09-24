use rapier2d::prelude::*;

fn main() {
    let world = setup_physics_scene();

    // DOCUSAURUS: Serialization start
    // Serialize the whole physics world.
    let serialized = bincode::serde::encode_to_vec(&world, bincode::config::standard()).unwrap();
    // Deserialize it.
    let (mut deserialized, _): (PhysicsWorld, usize) =
        bincode::serde::decode_from_slice(&serialized, bincode::config::standard()).unwrap();
    // The simulation can continue using the deserialized world.
    deserialized.step();
    // DOCUSAURUS: Serialization stop
}

/// Adapted from the basic simulation example of the user guide.
fn setup_physics_scene() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    /* Create the ground. */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1), None);

    /* Create the bouncing ball. */
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    /* Run the simulation for a while before serializing it. */
    for _ in 0..200 {
        world.step();
    }

    world
}
