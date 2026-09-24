use rapier2d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0)),
        ColliderBuilder::ball(0.5),
    );

    // DOCUSAURUS: Quarantine start
    world.step();

    // After the step, the objects neutralized by this step are known.
    let quarantined = world.quarantine().bodies().to_vec();
    for handle in quarantined {
        println!("The rigid-body {:?} went non-finite and was disabled.", handle);
        // Once the cause is fixed, the rigid-body is brought back into the simulation.
        world.bodies[handle].set_enabled(true);
    }
    // DOCUSAURUS: Quarantine stop
}
