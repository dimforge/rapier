use rapier_testbed2d::Testbed;
use rapier2d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    #[allow(clippy::excessive_precision)]
    let mut ps1 = [
        Point::new(16.0, 0.0),
        Point::new(14.93803712795643, 5.133601056842984),
        Point::new(13.79871746027416, 10.24928069555078),
        Point::new(12.56252963284711, 15.34107019122473),
        Point::new(11.20040987372525, 20.39856541571217),
        Point::new(9.66521217819836, 25.40369899225096),
        Point::new(7.87179930638133, 30.3179337000085),
        Point::new(5.635199558196225, 35.03820717801641),
        Point::new(2.405937953536585, 39.09554102558315),
    ];

    #[allow(clippy::excessive_precision)]
    let mut ps2 = [
        Point::new(24.0, 0.0),
        Point::new(22.33619528222415, 6.02299846205841),
        Point::new(20.54936888969905, 12.00964361211476),
        Point::new(18.60854610798073, 17.9470321677465),
        Point::new(16.46769273811807, 23.81367936585418),
        Point::new(14.05325025774858, 29.57079353071012),
        Point::new(11.23551045834022, 35.13775818285372),
        Point::new(7.752568160730571, 40.30450679009583),
        Point::new(3.016931552701656, 44.28891593799322),
    ];

    let scale = 0.25;
    let friction = 0.6;

    for i in 0..9 {
        ps1[i] *= scale;
        ps2[i] *= scale;
    }

    /*
     * Ground
     */
    let collider = ColliderBuilder::segment(point![-100.0, 0.0], point![100.0, 0.0]).friction(0.6);
    colliders.insert(collider);

    /*
     * Create the arch
     */
    for i in 0..8 {
        let ps = [ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]];
        let rigid_body = RigidBodyBuilder::dynamic();
        let ground_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::convex_hull(&ps)
            .unwrap()
            .friction(friction);
        colliders.insert_with_parent(collider, ground_handle, &mut bodies);
    }

    for i in 0..8 {
        let ps = [
            point![-ps2[i].x, ps2[i].y],
            point![-ps1[i].x, ps1[i].y],
            point![-ps1[i + 1].x, ps1[i + 1].y],
            point![-ps2[i + 1].x, ps2[i + 1].y],
        ];
        let rigid_body = RigidBodyBuilder::dynamic();
        let ground_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::convex_hull(&ps)
            .unwrap()
            .friction(friction);
        colliders.insert_with_parent(collider, ground_handle, &mut bodies);
    }

    {
        let ps = [
            ps1[8],
            ps2[8],
            point![-ps1[8].x, ps1[8].y],
            point![-ps2[8].x, ps2[8].y],
        ];
        let rigid_body = RigidBodyBuilder::dynamic();
        let ground_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::convex_hull(&ps)
            .unwrap()
            .friction(friction);
        colliders.insert_with_parent(collider, ground_handle, &mut bodies);
    }

    for i in 0..4 {
        let rigid_body =
            RigidBodyBuilder::dynamic().translation(vector![0.0, 0.5 + ps2[8].y + 1.0 * i as f32]);
        let ground_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(2.0, 0.5).friction(friction);
        colliders.insert_with_parent(collider, ground_handle, &mut bodies);
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 2.5], 20.0);
}
