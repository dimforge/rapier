use rapier_testbed2d::Testbed;
use rapier2d::parry::transformation::voxelization::FillMode;
use rapier2d::prelude::*;

const VOXEL_SIZE: Real = 0.1; // 0.25;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Voxel geometry type selection.
     */
    // TODO: make the testbed support custom enums (or at least a list of option from strings and
    //       associated constants).
    let settings = testbed.example_settings_mut();
    let falling_objects = settings.get_or_set_string(
        "Falling objects",
        3, // Defaults to Mixed.
        vec![
            "Ball".to_string(),
            "Cuboid".to_string(),
            "Capsule".to_string(),
            "Mixed".to_string(),
        ],
    );
    let voxel_size_y = settings.get_or_set_f32("Voxel size y", 1.0, 0.5..=2.0);
    let voxel_size = Vector::new(1.0, voxel_size_y);
    let test_ccd = settings.get_or_set_bool("Test CCD", false);

    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create dynamic objects to fall on voxels.
     */
    let nx = 50;
    for i in 0..nx {
        for j in 0..10 {
            let mut rb = RigidBodyBuilder::dynamic().translation(vector![
                i as f32 * 2.0 - nx as f32 / 2.0,
                20.0 + j as f32 * 2.0
            ]);
            if test_ccd {
                rb = rb.linvel(vector![0.0, -1000.0]).ccd_enabled(true);
            }
            let rb_handle = bodies.insert(rb);

            let falling_objects = if falling_objects == 3 {
                j % 3
            } else {
                falling_objects
            };

            let ball_radius = 0.5;
            let co = match falling_objects {
                0 => ColliderBuilder::ball(ball_radius),
                1 => ColliderBuilder::cuboid(ball_radius, ball_radius),
                2 => ColliderBuilder::capsule_y(ball_radius, ball_radius),
                _ => unreachable!(),
            };
            colliders.insert_with_parent(co, rb_handle, &mut bodies);
        }
    }

    /*
     * Voxelization.
     */
    let polyline = vec![
        point![0.0, 0.0],
        point![0.0, 10.0],
        point![7.0, 4.0],
        point![14.0, 10.0],
        point![14.0, 0.0],
        point![13.0, 7.0],
        point![7.0, 2.0],
        point![1.0, 7.0],
    ];
    let indices: Vec<_> = (0..polyline.len() as u32)
        .map(|i| [i, (i + 1) % polyline.len() as u32])
        .collect();
    let rb = bodies.insert(RigidBodyBuilder::fixed().translation(vector![-20.0, -10.0]));
    let shape = SharedShape::voxelized_mesh(&polyline, &indices, 0.2, FillMode::default());

    colliders.insert_with_parent(ColliderBuilder::new(shape), rb, &mut bodies);

    /*
     * A voxel wavy floor.
     */
    let voxels: Vec<_> = (0..300)
        .map(|i| {
            let y = (i as f32 / 20.0).sin().clamp(-0.5, 0.5) * 20.0;
            point![(i as f32 - 125.0) * voxel_size.x / 2.0, y * voxel_size.y]
        })
        .collect();
    colliders.insert(ColliderBuilder::voxels_from_points(voxel_size, &voxels));

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 20.0], 17.0);
}
