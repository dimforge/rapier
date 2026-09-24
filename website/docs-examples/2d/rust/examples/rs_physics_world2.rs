use rapier2d::prelude::*;

fn main() {
    // DOCUSAURUS: World start
    // The world owns every structure needed by the simulation.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);
    world.integration_parameters.dt = 1.0 / 60.0;

    /* Create the ground: a collider without any parent rigid-body. */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1), None);

    /* Create the bouncing ball: the rigid-body and its collider are inserted at once. */
    let (ball_handle, _ball_collider) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        world.step();
        println!("Ball altitude: {}", world.bodies[ball_handle].translation().y);
    }
    // DOCUSAURUS: World stop

    // DOCUSAURUS: WorldQueries start
    // The scene queries are available on the world directly.
    let ray = Ray::new(Vector::new(0.0, 10.0), Vector::new(0.0, -1.0));
    if let Some((handle, distance)) = world.cast_ray(&ray, 100.0, true, QueryFilter::default()) {
        println!("Collider {:?} hit at distance {}", handle, distance);
    }

    // Every set remains reachable as a public field of the world.
    let num_pairs = world.narrow_phase.contact_pairs().count();
    println!("{} contact pairs", num_pairs);
    // DOCUSAURUS: WorldQueries stop
}
