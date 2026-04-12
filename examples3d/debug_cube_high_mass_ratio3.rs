use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();

    let num_levels = 4;
    let stick_len = 2.0;
    let stick_rad = 0.2;

    /*
     * Floor.
     */
    let floor_body =
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -stick_len - stick_rad, 0.0));
    let floor_cube = ColliderBuilder::cuboid(stick_len, stick_len, stick_len);
    let (_floor_handle, _) = world.insert(floor_body, floor_cube);

    /*
     * Create a stack of capsule with a very heavy cube on top.
     */
    for i in 0..num_levels {
        let fi = i as f32;

        let body = RigidBodyBuilder::dynamic().translation(Vector::new(
            0.0,
            fi * stick_rad * 4.0,
            -(stick_len / 2.0 - stick_rad),
        ));
        let capsule = ColliderBuilder::cuboid(stick_len / 2.0, stick_rad, stick_rad);
        let (_handle, _) = world.insert(body, capsule);

        let body = RigidBodyBuilder::dynamic().translation(Vector::new(
            0.0,
            fi * stick_rad * 4.0,
            stick_len / 2.0 - stick_rad,
        ));
        let capsule = ColliderBuilder::cuboid(stick_len / 2.0, stick_rad, stick_rad);
        let (_handle, _) = world.insert(body, capsule);

        let body = RigidBodyBuilder::dynamic().translation(Vector::new(
            -(stick_len / 2.0 - stick_rad),
            (fi + 0.5) * stick_rad * 4.0,
            0.0,
        ));
        let capsule = ColliderBuilder::cuboid(stick_rad, stick_rad, stick_len / 2.0);
        let (_handle, _) = world.insert(body, capsule);

        let body = RigidBodyBuilder::dynamic().translation(Vector::new(
            stick_len / 2.0 - stick_rad,
            (fi + 0.5) * stick_rad * 4.0,
            0.0,
        ));
        let capsule = ColliderBuilder::cuboid(stick_rad, stick_rad, stick_len / 2.0);
        let (_handle, _) = world.insert(body, capsule);
    }

    /*
     * Big cube on top.
     */
    let cube_len = stick_len * 2.0;
    let floor_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(
            0.0,
            cube_len / 2.0 + (num_levels as f32 - 0.25) * stick_rad * 4.0,
            0.0,
        ))
        .additional_solver_iterations(36);
    let floor_cube = ColliderBuilder::cuboid(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0);
    let (_floor_handle, _) = world.insert(floor_body, floor_cube);

    let small_mass =
        MassProperties::from_cuboid(1.0, Vector::new(stick_rad, stick_rad, stick_len / 2.0)).mass();
    let big_mass = MassProperties::from_cuboid(
        1.0,
        Vector::new(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0),
    )
    .mass();
    println!(
        "debug_cube_high_mass_ratio3: small stick mass: {small_mass}, big cube mass: {big_mass}, mass_ratio: {}",
        big_mass / small_mass
    );

    /*
     * Set up the testbed.
     */
    testbed.set_physics_world(world);
    testbed.look_at(Vec3::new(10.0, 10.0, 10.0), Vec3::ZERO);
}
