use rapier2d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();

    /* Create the ground. */
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1), None);

    /* Create the bouncing ball. */
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        world.step();
        // DOCUSAURUS: IslandManager start
        // Iter on each rigid-bodies that moved (dynamic and kinematic).
        for rigid_body_handle in world.islands.active_bodies() {
            let rigid_body = &world.bodies[rigid_body_handle];
            println!(
                "Rigid body {:?} has a new position: {:?}",
                rigid_body_handle,
                rigid_body.position()
            );
        }
        // DOCUSAURUS: IslandManager stop
    }

    // DOCUSAURUS: QueryPipeline start
    // A temporary query pipeline borrowing the broad-phase, the narrow-phase, and the sets. This
    // is what `PhysicsWorld::query_pipeline` does.
    let query_pipeline = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        QueryFilter::default(),
    );
    // DOCUSAURUS: QueryPipeline stop

    let ray = Ray::new(Vector::new(0.0, 5.0), Vector::new(0.0, -1.0));
    let _ = query_pipeline.cast_ray(&ray, 10.0, true);
}
