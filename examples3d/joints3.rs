use rapier_testbed3d::Testbed;
use rapier3d::prelude::*;

fn create_coupled_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    use_articulations: bool,
) {
    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(origin.coords));
    let body1 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(origin.coords)
            .linvel(vector![5.0, 5.0, 5.0]),
    );
    colliders.insert_with_parent(ColliderBuilder::cuboid(1.0, 1.0, 1.0), body1, bodies);

    let joint1 = GenericJointBuilder::new(JointAxesMask::empty())
        .limits(JointAxis::LinX, [-3.0, 3.0])
        .limits(JointAxis::LinY, [0.0, 3.0])
        .coupled_axes(JointAxesMask::LIN_Y | JointAxesMask::LIN_Z);

    if use_articulations {
        multibody_joints.insert(ground, body1, joint1, true);
    } else {
        impulse_joints.insert(ground, body1, joint1, true);
    }
}

fn create_prismatic_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::fixed().translation(vector![origin.x, origin.y, origin.z]);
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    colliders.insert_with_parent(collider, curr_parent, bodies);

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![origin.x, origin.y, z]);
        let curr_child = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad, rad);
        colliders.insert_with_parent(collider, curr_child, bodies);

        let axis = if i % 2 == 0 {
            UnitVector::new_normalize(vector![1.0f32, 1.0, 0.0])
        } else {
            UnitVector::new_normalize(vector![-1.0f32, 1.0, 0.0])
        };

        let prism = PrismaticJointBuilder::new(axis)
            .local_anchor1(point![0.0, 0.0, 0.0])
            .local_anchor2(point![0.0, 0.0, -shift])
            .limits([-2.0, 2.0]);

        if use_articulations {
            multibody_joints.insert(curr_parent, curr_child, prism, true);
        } else {
            impulse_joints.insert(curr_parent, curr_child, prism, true);
        }
        curr_parent = curr_child;
    }
}

fn create_actuated_prismatic_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::fixed().translation(vector![origin.x, origin.y, origin.z]);
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    colliders.insert_with_parent(collider, curr_parent, bodies);

    for i in 0..num {
        let z = origin.z + (i + 1) as f32 * shift;
        let rigid_body = RigidBodyBuilder::dynamic().translation(vector![origin.x, origin.y, z]);
        let curr_child = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad, rad, rad);
        colliders.insert_with_parent(collider, curr_child, bodies);

        let axis = if i % 2 == 0 {
            UnitVector::new_normalize(vector![1.0, 1.0, 0.0])
        } else {
            UnitVector::new_normalize(vector![-1.0, 1.0, 0.0])
        };

        let mut prism = PrismaticJointBuilder::new(axis)
            .local_anchor1(point![0.0, 0.0, shift])
            .local_anchor2(point![0.0, 0.0, 0.0])
            .build();

        if i == 0 {
            prism
                .set_motor_velocity(2.0, 1.0e5)
                // We set a max impulse so that the motor doesn't fight
                // the limits with large forces.
                .set_limits([-2.0, 5.0])
                .set_motor_max_force(100.0);
        } else if i == 1 {
            prism
                .set_limits([-Real::MAX, 5.0])
                .set_motor_velocity(6.0, 1.0e3)
                // We set a max impulse so that the motor doesn't fight
                // the limits with large forces.
                .set_motor_max_force(100.0);
        } else if i > 1 {
            prism
                .set_motor_position(2.0, 1.0e3, 1.0e2)
                .set_motor_max_force(60.0);
        }

        if use_articulations {
            multibody_joints.insert(curr_parent, curr_child, prism, true);
        } else {
            impulse_joints.insert(curr_parent, curr_child, prism, true);
        }

        curr_parent = curr_child;
    }
}

fn create_revolute_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::fixed().translation(vector![origin.x, origin.y, 0.0]);
    let mut curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    colliders.insert_with_parent(collider, curr_parent, bodies);

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;
        let positions = [
            Isometry::translation(origin.x, origin.y, z),
            Isometry::translation(origin.x + shift, origin.y, z),
            Isometry::translation(origin.x + shift, origin.y, z + shift),
            Isometry::translation(origin.x, origin.y, z + shift),
        ];

        let mut handles = [curr_parent; 4];
        for k in 0..4 {
            let rigid_body = RigidBodyBuilder::dynamic().position(positions[k]);
            handles[k] = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            colliders.insert_with_parent(collider, handles[k], bodies);
        }

        // Setup four impulse_joints.
        let x = Vector::x_axis();
        let z = Vector::z_axis();
        let revs = [
            RevoluteJointBuilder::new(z).local_anchor2(point![0.0, 0.0, -shift]),
            RevoluteJointBuilder::new(x).local_anchor2(point![-shift, 0.0, 0.0]),
            RevoluteJointBuilder::new(z).local_anchor2(point![0.0, 0.0, -shift]),
            RevoluteJointBuilder::new(x).local_anchor2(point![shift, 0.0, 0.0]),
        ];

        if use_articulations {
            multibody_joints.insert(curr_parent, handles[0], revs[0], true);
            multibody_joints.insert(handles[0], handles[1], revs[1], true);
            multibody_joints.insert(handles[1], handles[2], revs[2], true);
            multibody_joints.insert(handles[2], handles[3], revs[3], true);
        } else {
            impulse_joints.insert(curr_parent, handles[0], revs[0], true);
            impulse_joints.insert(handles[0], handles[1], revs[1], true);
            impulse_joints.insert(handles[1], handles[2], revs[2], true);
            impulse_joints.insert(handles[2], handles[3], revs[3], true);
        }

        curr_parent = handles[3];
    }
}

fn create_revolute_joints_with_limits(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    use_articulations: bool,
) {
    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(origin.coords));

    let platform1 = bodies.insert(RigidBodyBuilder::dynamic().translation(origin.coords));
    colliders.insert_with_parent(ColliderBuilder::cuboid(4.0, 0.2, 2.0), platform1, bodies);

    let shift = vector![0.0, 0.0, 6.0];
    let platform2 = bodies.insert(RigidBodyBuilder::dynamic().translation(origin.coords + shift));
    colliders.insert_with_parent(ColliderBuilder::cuboid(4.0, 0.2, 2.0), platform2, bodies);

    let z = Vector::z_axis();
    let joint1 = RevoluteJointBuilder::new(z).limits([-0.2, 0.2]);
    // let joint1 = GenericJointBuilder::new(JointAxesMask::X | JointAxesMask::Y | JointAxesMask::Z)
    //     .local_axis1(z)
    //     .local_axis2(z)
    //     .limits(JointAxis::AngX, [-0.2, 0.2])
    //     .limits(JointAxis::AngY, [0.0, 0.4])
    //     .limits(JointAxis::AngZ, [0.0, 0.4])
    //     .coupled_axes(JointAxesMask::ANG_Y | JointAxesMask::ANG_Z);

    if use_articulations {
        multibody_joints.insert(ground, platform1, joint1, true);
    } else {
        impulse_joints.insert(ground, platform1, joint1, true);
    }

    let joint2 = RevoluteJointBuilder::new(z)
        .local_anchor2(-Point::from(shift))
        .limits([-0.2, 0.2]);

    // let joint2 = GenericJointBuilder::new(JointAxesMask::X | JointAxesMask::Y | JointAxesMask::Z)
    //     .local_axis1(z)
    //     .local_axis2(z)
    //     .local_anchor2(-Point::from(shift))
    //     .limits(JointAxis::AngX, [-0.2, 0.2])
    //     .limits(JointAxis::AngY, [0.0, 0.4])
    //     .limits(JointAxis::AngZ, [0.0, 0.4])
    //     .coupled_axes(JointAxesMask::ANG_Y | JointAxesMask::ANG_Z);

    if use_articulations {
        multibody_joints.insert(platform1, platform2, joint2, true);
    } else {
        impulse_joints.insert(platform1, platform2, joint2, true);
    }

    // Let’s add a couple of cuboids that will fall on the platforms, triggering the joint limits.
    let cuboid_body1 = bodies
        .insert(RigidBodyBuilder::dynamic().translation(origin.coords + vector![-2.0, 4.0, 0.0]));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.6, 0.6, 0.6).friction(1.0),
        cuboid_body1,
        bodies,
    );

    let cuboid_body2 = bodies.insert(
        RigidBodyBuilder::dynamic().translation(origin.coords + shift + vector![2.0, 16.0, 0.0]),
    );
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.6, 0.6, 0.6).friction(1.0),
        cuboid_body2,
        bodies,
    );
}

fn create_fixed_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for i in 0..num {
        for k in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            // NOTE: the num - 2 test is to avoid two consecutive
            // fixed bodies. Because physx will crash if we add
            // a joint between these.
            let status = if i == 0 && (k % 4 == 0 && k != num - 2 || k == num - 1) {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(vector![
                origin.x + fk * shift,
                origin.y,
                origin.z + fi * shift
            ]);
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::ball(rad);
            colliders.insert_with_parent(collider, child_handle, bodies);

            // Vertical joint.
            if i > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = FixedJointBuilder::new().local_anchor2(point![0.0, 0.0, -shift]);

                if use_articulations {
                    multibody_joints.insert(parent_handle, child_handle, joint, true);
                } else {
                    impulse_joints.insert(parent_handle, child_handle, joint, true);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - 1;
                let parent_handle = body_handles[parent_index];
                let joint = FixedJointBuilder::new().local_anchor2(point![-shift, 0.0, 0.0]);
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }

            body_handles.push(child_handle);
        }
    }
}

fn create_spherical_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 1.0;

    let mut body_handles = Vec::new();

    for k in 0..num {
        for i in 0..num {
            let fk = k as f32;
            let fi = i as f32;

            let status = if i == 0 && (k % 4 == 0 || k == num - 1) {
                RigidBodyType::Fixed
            } else {
                RigidBodyType::Dynamic
            };

            let rigid_body = RigidBodyBuilder::new(status).translation(vector![
                fk * shift,
                0.0,
                fi * shift * 2.0
            ]);
            let child_handle = bodies.insert(rigid_body);
            let collider = ColliderBuilder::capsule_z(rad * 1.25, rad);
            colliders.insert_with_parent(collider, child_handle, bodies);

            // Vertical joint.
            if i > 0 {
                let parent_handle = *body_handles.last().unwrap();
                let joint =
                    SphericalJointBuilder::new().local_anchor2(point![0.0, 0.0, -shift * 2.0]);

                if use_articulations {
                    multibody_joints.insert(parent_handle, child_handle, joint, true);
                } else {
                    impulse_joints.insert(parent_handle, child_handle, joint, true);
                }
            }

            // Horizontal joint.
            if k > 0 {
                let parent_index = body_handles.len() - num;
                let parent_handle = body_handles[parent_index];
                let joint = SphericalJointBuilder::new().local_anchor2(point![-shift, 0.0, 0.0]);
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }

            body_handles.push(child_handle);
        }
    }
}

fn create_spherical_joints_with_limits(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    use_articulations: bool,
) {
    let shift = vector![0.0, 0.0, 3.0];

    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(origin.coords));

    let ball1 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(origin.coords + shift)
            .linvel(vector![20.0, 20.0, 0.0]),
    );
    colliders.insert_with_parent(ColliderBuilder::cuboid(1.0, 1.0, 1.0), ball1, bodies);

    let ball2 = bodies.insert(RigidBodyBuilder::dynamic().translation(origin.coords + shift * 2.0));
    colliders.insert_with_parent(ColliderBuilder::cuboid(1.0, 1.0, 1.0), ball2, bodies);

    let joint1 = SphericalJointBuilder::new()
        .local_anchor2(Point::from(-shift))
        .limits(JointAxis::LinX, [-0.2, 0.2])
        .limits(JointAxis::LinY, [-0.2, 0.2]);

    let joint2 = SphericalJointBuilder::new()
        .local_anchor2(Point::from(-shift))
        .limits(JointAxis::LinX, [-0.3, 0.3])
        .limits(JointAxis::LinY, [-0.3, 0.3]);

    if use_articulations {
        multibody_joints.insert(ground, ball1, joint1, true);
        multibody_joints.insert(ball1, ball2, joint2, true);
    } else {
        impulse_joints.insert(ground, ball1, joint1, true);
        impulse_joints.insert(ball1, ball2, joint2, true);
    }
}

fn create_actuated_revolute_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    // We will reuse this base configuration for all the impulse_joints here.
    let z = Vector::z_axis();
    let joint_template = RevoluteJointBuilder::new(z).local_anchor2(point![0.0, 0.0, -shift]);

    let mut parent_handle = RigidBodyHandle::invalid();

    for i in 0..num {
        let fi = i as f32;

        // NOTE: the num - 2 test is to avoid two consecutive
        // fixed bodies. Because physx will crash if we add
        // a joint between these.
        let status = if i == 0 {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        };

        let shifty = (i >= 1) as u32 as f32 * -2.0;

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(vector![origin.x, origin.y + shifty, origin.z + fi * shift])
            // .rotation(Vector3::new(0.0, fi * 1.1, 0.0))
            ;

        let child_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(rad * 2.0, rad * 6.0 / (fi + 1.0), rad);
        colliders.insert_with_parent(collider, child_handle, bodies);

        if i > 0 {
            let mut joint = joint_template.motor_model(MotorModel::AccelerationBased);

            if i % 3 == 1 {
                joint = joint.motor_velocity(-20.0, 100.0);
            } else if i == num - 1 {
                let stiffness = 200.0;
                let damping = 100.0;
                joint = joint.motor_position(std::f32::consts::FRAC_PI_2, stiffness, damping);
            }

            if i == 1 {
                joint = joint
                    .local_anchor2(point![0.0, 2.0, -shift])
                    .motor_velocity(-2.0, 1000.0);
            }

            if use_articulations {
                multibody_joints.insert(parent_handle, child_handle, joint, true);
            } else {
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }
        }

        parent_handle = child_handle;
    }
}

fn create_actuated_spherical_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    // We will reuse this base configuration for all the impulse_joints here.
    let joint_template = SphericalJointBuilder::new().local_anchor1(point![0.0, 0.0, shift]);

    let mut parent_handle = RigidBodyHandle::invalid();

    for i in 0..num {
        let fi = i as f32;

        // NOTE: the num - 2 test is to avoid two consecutive
        // fixed bodies. Because physx will crash if we add
        // a joint between these.
        let status = if i == 0 {
            RigidBodyType::Fixed
        } else {
            RigidBodyType::Dynamic
        };

        let rigid_body = RigidBodyBuilder::new(status)
            .translation(vector![origin.x, origin.y, origin.z + fi * shift])
            // .rotation(Vector3::new(0.0, fi * 1.1, 0.0))
            ;

        let child_handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::capsule_y(rad * 2.0 / (fi + 1.0), rad);
        colliders.insert_with_parent(collider, child_handle, bodies);

        if i > 0 {
            let mut joint = joint_template;

            if i == 1 {
                joint = joint
                    .motor_velocity(JointAxis::AngX, 0.0, 0.1)
                    .motor_velocity(JointAxis::AngY, 0.5, 0.1)
                    .motor_velocity(JointAxis::AngZ, -2.0, 0.1);
            } else if i == num - 1 {
                let stiffness = 0.2;
                let damping = 1.0;
                joint = joint
                    .motor_position(JointAxis::AngX, 0.0, stiffness, damping)
                    .motor_position(JointAxis::AngY, 1.0, stiffness, damping)
                    .motor_position(
                        JointAxis::AngZ,
                        std::f32::consts::FRAC_PI_2,
                        stiffness,
                        damping,
                    );
            }

            if use_articulations {
                multibody_joints.insert(parent_handle, child_handle, joint, true);
            } else {
                impulse_joints.insert(parent_handle, child_handle, joint, true);
            }
        }

        parent_handle = child_handle;
    }
}

fn do_init_world(testbed: &mut Testbed, use_articulations: bool) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    create_prismatic_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![20.0, 5.0, 0.0],
        4,
        use_articulations,
    );
    create_actuated_prismatic_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![25.0, 5.0, 0.0],
        4,
        use_articulations,
    );
    create_revolute_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![20.0, 0.0, 0.0],
        3,
        use_articulations,
    );
    create_revolute_joints_with_limits(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![34.0, 0.0, 0.0],
        use_articulations,
    );
    create_fixed_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![0.0, 10.0, 0.0],
        10,
        use_articulations,
    );
    create_actuated_revolute_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![20.0, 10.0, 0.0],
        6,
        use_articulations,
    );
    create_actuated_spherical_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![13.0, 10.0, 0.0],
        3,
        use_articulations,
    );
    create_spherical_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        15,
        use_articulations,
    );
    create_spherical_joints_with_limits(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![-5.0, 0.0, 0.0],
        use_articulations,
    );
    create_coupled_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![0.0, 20.0, 0.0],
        use_articulations,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![15.0, 5.0, 42.0], point![13.0, 1.0, 1.0]);
}

pub fn init_world_with_joints(testbed: &mut Testbed) {
    do_init_world(testbed, false)
}

pub fn init_world_with_articulations(testbed: &mut Testbed) {
    do_init_world(testbed, true)
}
