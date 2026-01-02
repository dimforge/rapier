use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 0.2;
    let ground_height = 0.01;

    let rigid_body = RigidBodyBuilder::fixed().translation(Vector::new(0.0, -ground_height, 0.0));
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Setup groups
     */
    let num_segments = 10;
    let body = RigidBodyBuilder::fixed();
    let mut last_body = bodies.insert(body);
    let mut last_link = MultibodyJointHandle::invalid();

    for i in 0..num_segments {
        let size = 1.0 / num_segments as f32;
        let body = RigidBodyBuilder::dynamic().can_sleep(false);
        let new_body = bodies.insert(body);
        // NOTE: we add a sensor collider just to make the destbed draw a rectangle to make
        //       the demo look nicer. IK could be used without cuboid.
        let collider = ColliderBuilder::cuboid(size / 8.0, size / 2.0, size / 8.0)
            .density(0.0)
            .sensor(true);
        colliders.insert_with_parent(collider, new_body, &mut bodies);

        let link_ab = SphericalJointBuilder::new()
            .local_anchor1(Vector::new(0.0, size / 2.0 * (i != 0) as usize as f32, 0.0))
            .local_anchor2(Vector::new(0.0, -size / 2.0, 0.0))
            .build()
            .data;

        last_link = multibody_joints
            .insert(last_body, new_body, link_ab, true)
            .unwrap();

        last_body = new_body;
    }

    let mut displacements = DVector::zeros(0);

    testbed.add_callback(move |graphics, physics, _, _| {
        let Some(graphics) = graphics else { return };
        if let Some((multibody, link_id)) = physics.multibody_joints.get_mut(last_link) {
            // Ensure our displacement vector has the right number of elements.
            if displacements.nrows() < multibody.ndofs() {
                displacements = DVector::zeros(multibody.ndofs());
            } else {
                displacements.fill(0.0);
            }

            let Some(mouse_ray) = graphics.mouse().ray else {
                return;
            };

            // Cast a ray on a plane aligned with the camera passing through the origin.
            let fwd = graphics.camera_fwd_dir();
            let fwd_glam = Vector::new(-fwd.x, -fwd.y, -fwd.z).normalize();
            let halfspace = HalfSpace { normal: fwd_glam };
            let (mouse_orig, mouse_dir) = mouse_ray;
            let ray = Ray::new(
                Vector::new(mouse_orig.x, mouse_orig.y, mouse_orig.z),
                Vector::new(mouse_dir.x, mouse_dir.y, mouse_dir.z),
            );
            let Some(hit) = halfspace.cast_local_ray(&ray, f32::MAX, false) else {
                return;
            };
            let target_point = ray.point_at(hit);

            let options = InverseKinematicsOption {
                constrained_axes: JointAxesMask::LIN_AXES,
                ..Default::default()
            };

            multibody.inverse_kinematics(
                &physics.bodies,
                link_id,
                &options,
                &Pose::from_translation(target_point),
                |_| true,
                &mut displacements,
            );
            multibody.apply_displacements(displacements.as_slice());
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(Vec3::new(0.0, 0.5, 2.5), Vec3::new(0.0, 0.5, 0.0));
}
