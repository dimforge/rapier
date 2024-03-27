use rapier2d::{prelude::*, parry};
use rapier_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    // Bug

    let shape1 = SharedShape::capsule_y(5.0, 10.0);
    let mut vec = Vec::<Point::<Real>>::with_capacity(3);
    unsafe {
        vec.push(Point::<Real> { coords : vector![64.0, 507.0] });
        vec.push(Point::<Real> { coords : vector![440.0, 326.0] });
        vec.push(Point::<Real> { coords : vector![1072.0, 507.0] });
    }
    let shape2 = SharedShape::convex_polyline(vec);
    let shape2 = shape2.unwrap();
    let transform1 = Isometry::new(vector![381.592, 348.491], 0.0);
    let transform2 = Isometry::new(vector![0.0, 0.0], 0.0);

    /*
     * Ground
     */
    let rad = 1.0;
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![0.0, 0.0])
        .rotation(0.0);
    let handle = bodies.insert(rigid_body);
    //let collider = ColliderBuilder::cuboid(rad, rad);
    colliders.insert_with_parent(ColliderBuilder::new(shape2.clone()), handle, &mut bodies);

    // Build the dynamic box rigid body.
    let rigid_body = RigidBodyBuilder::fixed()
        .translation(vector![381.592, 348.491])
        .can_sleep(false);
    let handle = bodies.insert(rigid_body);
    //let collider = ColliderBuilder::ball(rad);
    colliders.insert_with_parent(ColliderBuilder::new(shape1.clone()), handle, &mut bodies);
    
    if let Ok(Some(contact)) = parry::query::contact(
        &transform1, shape1.as_ref(), &transform2, shape2.as_ref(), 1.0
    ) {
        panic!("collision");
    } else {
        print!("no collision");
    }
    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![381.592, 348.491], 1.0);
}
