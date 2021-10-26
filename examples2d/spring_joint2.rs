use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();

    /*
     * The ground
     */
    let ground_size = 5.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(vector![0.0, -ground_height])
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    let rb = RigidBodyBuilder::new_dynamic()
        .translation(vector![3.0, 3.0])
        .build();
    let cube1 = bodies.insert(rb);
    let collider = ColliderBuilder::cuboid(0.5, 0.5).build();
    colliders.insert_with_parent(collider, cube1, &mut bodies);

    let rb = RigidBodyBuilder::new_dynamic()
        .translation(vector![-3.0, 3.0])
        .build();
    let cube2 = bodies.insert(rb);
    let collider = ColliderBuilder::cuboid(0.5, 0.5).build();
    colliders.insert_with_parent(collider, cube2, &mut bodies);

    let spring = SpringJoint::new(
        point![0.0, 0.0], 
        point![0.0, 0.0], 
        4.0,
        5.0,
        0.0,
    );
    let _spring = joints.insert(cube1, cube2, spring);

    //testbed.add_callback(move |_, physics, _, _run_state| {
        //let mut total_energy: Real = 0.0;
        //let mut cube1pos: Option<&Isometry<Real>> = None;
        //let mut cube2pos: Option<&Isometry<Real>> = None;
        //if let Some(cube1) = physics.bodies.get(cube1) {
            //total_energy += cube1.kinetic_energy();
            //cube1pos = Some(cube1.position());
        //}
        //if let Some(cube2) = physics.bodies.get(cube2) {
            //total_energy += cube2.kinetic_energy();
            //cube2pos = Some(cube2.position());
        //}
        //if let Some(spring) = physics.joints.get(spring) {
            //let spring = spring.params.as_spring_joint().unwrap();

            //let anchor_world1 = cube1pos.unwrap() * spring.local_anchor1;
            //let anchor_world2 = cube2pos.unwrap() * spring.local_anchor2;
            //let r = anchor_world2 - anchor_world1;
            //let dx = r.magnitude() - spring.rest_length;
            //total_energy += 0.5 * spring.stiffness * dx * dx;
        //}
        //println!("System Energy (J): {:?}", total_energy);
    //});
    /*
     * Set up the testbed.
     */
    testbed.set_world_with_params(bodies, colliders, joints, Vector::zeros(), ());
    testbed.look_at(point![0.0, 3.0], 50.0);
}
