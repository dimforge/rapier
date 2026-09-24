use crate::*;

fn encoded<T: serde::Serialize>(value: &T) -> Vec<u8> {
    bincode::serialize(value).unwrap()
}

#[test]
fn pod_defaults_match_native_configuration() {
    unsafe {
        for kind in [
            RPR_DYNAMIC,
            RPR_FIXED,
            RPR_KINEMATIC_POSITION_BASED,
            RPR_KINEMATIC_VELOCITY_BASED,
        ] {
            let desc = match kind {
                RPR_DYNAMIC => rpr_dynamic_rigid_body_desc(),
                RPR_FIXED => rpr_fixed_rigid_body_desc(),
                RPR_KINEMATIC_POSITION_BASED => rpr_kinematic_position_based_rigid_body_desc(),
                _ => rpr_kinematic_velocity_based_rigid_body_desc(),
            };
            let actual = desc.raw().unwrap().build();
            let expected = RigidBodyBuilder::new(body_type(kind).unwrap()).build();
            assert_eq!(encoded(&actual), encoded(&expected));
        }
        let desc = RprColliderDesc::default();
        assert_eq!(
            encoded(&desc.raw().unwrap().build()),
            encoded(&ColliderBuilder::ball(0.5).build())
        );
        let native = IntegrationParameters::default();
        let desc = RprIntegrationParameters::from(native);
        assert_eq!(encoded(&desc.raw().unwrap()), encoded(&native));
        let native = SoftBodyMaterial::default();
        let desc = RprSoftBodyMaterial::from(native);
        assert_eq!(encoded(&desc.raw().unwrap()), encoded(&native));
        let native = GenericJoint::new(JointAxesMask::empty());
        assert_eq!(
            encoded(&RprJointDesc::from(native).raw().unwrap()),
            encoded(&native)
        );
    }
}

fn check_soft_recipe(desc: RprSoftBodyDesc, native: SoftBodyBuilder) {
    let actual = unsafe { desc.raw().unwrap() };
    // Includes topology, generated radii, material and collider template defaults.
    assert_eq!(format!("{actual:?}"), format!("{native:?}"));
}

#[test]
fn pod_soft_recipes_preserve_generator_defaults() {
    let positions: [RprVector; 2] = [Vector::ZERO.into(), Vector::X.into()];
    check_soft_recipe(
        RprSoftBodyDesc {
            positions: RprVectorView {
                data: (positions.as_ptr()).cast(),
                count: 2,
            },
            ..Default::default()
        },
        SoftBodyBuilder::new(vec![Vector::ZERO, Vector::X]),
    );
    check_soft_recipe(
        RprSoftBodyDesc {
            kind: RPR_SOFT_DESC_ROPE,
            a: Vector::ZERO.into(),
            b: Vector::X.into(),
            nx: 5,
            ..Default::default()
        },
        SoftBodyBuilder::rope(Vector::ZERO, Vector::X, 5),
    );
    #[cfg(feature = "dim2")]
    check_soft_recipe(
        RprSoftBodyDesc {
            kind: RPR_SOFT_DESC_GRID,
            ..Default::default()
        },
        SoftBodyBuilder::grid(Vector::ZERO, Vector::ONE, 2, 2),
    );
    #[cfg(feature = "dim3")]
    {
        check_soft_recipe(
            RprSoftBodyDesc {
                kind: RPR_SOFT_DESC_CLOTH,
                ..Default::default()
            },
            SoftBodyBuilder::cloth(Vector::ZERO, Vector::X, Vector::Y, 2, 2),
        );
        check_soft_recipe(
            RprSoftBodyDesc {
                kind: RPR_SOFT_DESC_CUBOID,
                ..Default::default()
            },
            SoftBodyBuilder::cuboid(Vector::ZERO, Vector::ONE, 2, 2, 2),
        );
    }
    let points: Vec<RprVector> = vec![Vector::ZERO.into(), Vector::X.into(), Vector::Y.into()];
    #[cfg(feature = "dim3")]
    let indices = [0, 1, 2];
    #[cfg(feature = "dim2")]
    let indices = [0, 1, 1, 2, 2, 0];
    let desc = RprSoftBodyDesc {
        kind: RPR_SOFT_DESC_SURFACE,
        positions: RprVectorView {
            data: (points.as_ptr()).cast(),
            count: 3,
        },
        surface: RprSurfaceElementView {
            data: (indices.as_ptr()).cast(),
            count: indices.len() / rapier::math::DIM,
        },
        ..Default::default()
    };
    #[cfg(feature = "dim3")]
    let native =
        SoftBodyBuilder::trimesh(vec![Vector::ZERO, Vector::X, Vector::Y], vec![[0, 1, 2]])
            .unwrap();
    #[cfg(feature = "dim2")]
    let native = SoftBodyBuilder::polyline(
        vec![Vector::ZERO, Vector::X, Vector::Y],
        Some(vec![[0, 1], [1, 2], [2, 0]]),
    )
    .unwrap();
    check_soft_recipe(desc, native);
}

#[test]
fn pod_insertion_copies_inputs_and_rejects_invalid_data_atomically() {
    unsafe {
        let mut world = RprWorld::new(RprPhysicsWorld(PhysicsWorld::default()));
        let body = rpr_dynamic_rigid_body_desc();
        let mut collider = RprColliderDesc::default();
        collider.shape.radius = -1.0;

        let body_handle = rpr_insert_rigid_body(&mut world, &body);
        assert_eq!(rpr_last_status(), RPR_OK);
        let failed = rpr_insert_collider(body_handle, &collider);
        assert_eq!(rpr_last_status(), RPR_INVALID_ARGUMENT);
        // Separate calls leave the successfully inserted body intact on collider failure.
        assert_eq!((*world.read().unwrap().raw()).0.bodies.len(), 1);
        assert_eq!((*world.read().unwrap().raw()).0.colliders.len(), 0);
        assert_eq!(failed, RprColliderHandle::default());
        assert_eq!(rpr_rigid_body_validate_handle(body_handle), RPR_OK);
        let mut points: [RprVector; 2] = [Vector::ZERO.into(), Vector::X.into()];
        collider.shape.kind = RPR_SHAPE_DESC_POLYLINE;
        collider.shape.vertices = RprVectorView {
            data: points.as_ptr(),
            count: 2,
        };
        let ch = rpr_insert_collider_without_parent(&mut world, &collider);
        assert_eq!(rpr_last_status(), RPR_OK);
        points[1] = (Vector::X * 100.0).into();
        assert_eq!(
            (&(*world.read().unwrap().raw()).0.colliders)[ch.raw()]
                .shape()
                .as_polyline()
                .unwrap()
                .vertices()[1],
            Vector::X
        );
        let desc = RprSoftBodyDesc {
            positions: RprVectorView {
                data: (points.as_ptr()).cast(),
                count: 2,
            },
            ..Default::default()
        };
        let copied = desc.raw().unwrap();
        points[1] = Vector::ZERO.into();
        assert_eq!(copied.positions[1], Vector::X * 100.0);
        assert_eq!(points[1].x, 0.0);
        let mut data = RprIntegrationParameters::from(IntegrationParameters::default());
        let mut params = NativeIntegrationParameters(IntegrationParameters::default());
        let before = encoded(&params.0);
        data.dt = Real::NAN;
        assert_eq!(
            native_integration_parameters_set_data(&mut params, &data),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(encoded(&params.0), before);
    }
}

#[test]
fn pod_shape_recipes_match_native_shapes() {
    unsafe {
        let mut desc = RprShapeDesc::default();
        let check = |desc: &RprShapeDesc, expected: SharedShape| {
            assert_eq!(encoded(&desc.raw().unwrap()), encoded(&expected));
        };
        check(&desc, SharedShape::ball(0.5));
        desc.kind = RPR_SHAPE_DESC_CUBOID;
        desc.a = Vector::ONE.into();
        #[cfg(feature = "dim2")]
        check(&desc, SharedShape::cuboid(1.0, 1.0));
        #[cfg(feature = "dim3")]
        check(&desc, SharedShape::cuboid(1.0, 1.0, 1.0));
        desc.kind = RPR_SHAPE_DESC_ROUND_CUBOID;
        #[cfg(feature = "dim2")]
        check(&desc, SharedShape::round_cuboid(1.0, 1.0, 0.5));
        #[cfg(feature = "dim3")]
        check(&desc, SharedShape::round_cuboid(1.0, 1.0, 1.0, 0.5));
        desc.a = Vector::ZERO.into();
        desc.b = Vector::X.into();
        desc.c = Vector::Y.into();
        desc.kind = RPR_SHAPE_DESC_CAPSULE;
        check(&desc, SharedShape::capsule(Vector::ZERO, Vector::X, 0.5));
        desc.kind = RPR_SHAPE_DESC_SEGMENT;
        check(&desc, SharedShape::segment(Vector::ZERO, Vector::X));
        desc.kind = RPR_SHAPE_DESC_TRIANGLE;
        check(
            &desc,
            SharedShape::triangle(Vector::ZERO, Vector::X, Vector::Y),
        );
        desc.kind = RPR_SHAPE_DESC_HALFSPACE;
        desc.a = Vector::Y.into();
        check(&desc, SharedShape::halfspace(Vector::Y));
        #[cfg(feature = "dim3")]
        {
            desc.kind = RPR_SHAPE_DESC_CYLINDER;
            check(&desc, SharedShape::cylinder(0.5, 0.5));
            desc.kind = RPR_SHAPE_DESC_CONE;
            check(&desc, SharedShape::cone(0.5, 0.5));
        }
        let child = RprCompoundShapeDesc {
            pose: Pose::IDENTITY.into(),
            shape: RprShapeDesc::default(),
        };
        desc.kind = RPR_SHAPE_DESC_COMPOUND;
        desc.children = RprCompoundShapeView {
            data: &child,
            count: 1,
        };
        check(
            &desc,
            SharedShape::compound(vec![(Pose::IDENTITY, SharedShape::ball(0.5))]),
        );
        // Cycles and unsupported tags are rejected before native construction.
        let mut cycle = child;
        cycle.shape.kind = RPR_SHAPE_DESC_COMPOUND;
        cycle.shape.children = RprCompoundShapeView {
            data: std::ptr::addr_of!(cycle),
            count: 1,
        };
        assert!(cycle.shape.raw().is_err());
        desc.kind = u32::MAX;
        assert!(desc.raw().is_err());
        desc.kind = RPR_SHAPE_DESC_HEIGHTFIELD;
        let heights = [0.0, 1.0, 0.5, 0.0];
        desc.heights = RprRealView {
            data: heights.as_ptr(),
            count: if cfg!(feature = "dim2") { 2 } else { 4 },
        };
        desc.rows = 2;
        #[cfg(feature = "dim2")]
        {
            desc.columns = 1;
            check(
                &desc,
                SharedShape::heightfield(heights[..2].to_vec(), Vector::ONE),
            );
        }
        #[cfg(feature = "dim3")]
        {
            desc.columns = 2;
            check(
                &desc,
                SharedShape::heightfield(
                    rapier::parry::utils::Array2::new(2, 2, heights.to_vec()),
                    Vector::ONE,
                ),
            );
        }
    }
}

#[test]
fn procedural_description_constructors_match_rust() {
    #[cfg(feature = "dim2")]
    {
        let desc = rpr_disk_soft_body_desc(Vector::Y.into(), 1.0, 12);
        check_soft_recipe(desc, SoftBodyBuilder::disk(Vector::Y, 1.0, 12));
    }
    #[cfg(feature = "dim3")]
    {
        let desc = rpr_sphere_soft_body_desc(Vector::Y.into(), 1.0, 1);
        check_soft_recipe(desc, SoftBodyBuilder::sphere(Vector::Y, 1.0, 1));
        let desc =
            rpr_cloth_tube_soft_body_desc(Vector::ZERO.into(), Vector::Y.into(), 1.0, 0.5, 8, 3);
        check_soft_recipe(
            desc,
            SoftBodyBuilder::cloth_tube(Vector::ZERO, Vector::Y, 1.0, 0.5, 8, 3),
        );
    }
    let mut desc = rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), 3);
    desc.totalMass = RprOptionalReal {
        enabled: 1,
        value: 6.0,
    };
    desc.translation = Vector::Y.into();
    check_soft_recipe(
        desc,
        SoftBodyBuilder::rope(Vector::ZERO, Vector::X, 3)
            .mass(6.0)
            .translated(Vector::Y),
    );
}
#[test]
fn joint_descriptions_allow_one_sided_limits() {
    let mut desc = rpr_default_joint_desc();
    desc.limitAxes = 1;
    desc.limits[0].min = -Real::INFINITY;
    desc.limits[0].max = 2.0;
    assert!(desc.raw().is_ok());
    desc.limits[0].min = Real::NAN;
    assert!(desc.raw().is_err());
}

#[test]
fn value_constructors_match_native_values_and_defer_validation() {
    unsafe {
        let collider = rpr_cuboid_collider_desc(Vector::splat(2.0).into());
        #[cfg(feature = "dim2")]
        let native = ColliderBuilder::cuboid(2.0, 2.0);
        #[cfg(feature = "dim3")]
        let native = ColliderBuilder::cuboid(2.0, 2.0, 2.0);
        assert_eq!(
            encoded(&collider.raw().unwrap().build()),
            encoded(&native.build())
        );
        assert_eq!(
            encoded(&rpr_fixed_joint_desc().raw().unwrap()),
            encoded(&GenericJoint::from(FixedJointBuilder::new().build()))
        );
        let axis = (Vector::X + Vector::Y).normalize();
        let mut actual = rpr_prismatic_joint_desc((axis * 3.0).into()).raw().unwrap();
        let native = GenericJoint::from(PrismaticJointBuilder::new(axis).build());
        // 2D frames round-trip through an angle; permit trigonometric rounding.
        assert!((actual.local_axis1() - native.local_axis1()).length() < Real::EPSILON * 8.0);
        assert!((actual.local_axis2() - native.local_axis2()).length() < Real::EPSILON * 8.0);
        actual.local_frame1.rotation = native.local_frame1.rotation;
        actual.local_frame2.rotation = native.local_frame2.rotation;
        assert_eq!(encoded(&actual), encoded(&native));
        #[cfg(feature = "dim2")]
        assert_eq!(
            encoded(&rpr_revolute_joint_desc().raw().unwrap()),
            encoded(&GenericJoint::from(RevoluteJointBuilder::new().build()))
        );
        #[cfg(feature = "dim3")]
        assert_eq!(
            encoded(&rpr_revolute_joint_desc(Vector::X.into()).raw().unwrap()),
            encoded(&GenericJoint::from(
                RevoluteJointBuilder::new(Vector::X).build()
            ))
        );
        let softness = RprSpringCoefficients {
            natural_frequency: 12.0,
            damping_ratio: 0.3,
        };
        assert_eq!(
            encoded(&rpr_uniform_soft_body_material(softness).raw().unwrap()),
            encoded(&SoftBodyMaterial::uniform(softness.raw().unwrap()))
        );

        // Invalid values can be edited before insertion; constructors do not report errors.
        let mut invalid = rpr_ball_collider_desc(-1.0);
        assert_eq!(invalid.shape.radius, -1.0);
        assert!(invalid.raw().is_err());
        invalid.shape.radius = 1.0;
        assert!(invalid.raw().is_ok());
        assert!(
            rpr_cuboid_collider_desc(Vector::splat(-1.0).into())
                .raw()
                .is_err()
        );
        assert!(rpr_prismatic_joint_desc(Vector::ZERO.into()).raw().is_err());
        assert!(
            rpr_prismatic_joint_desc(Vector::splat(Real::NAN).into())
                .raw()
                .is_err()
        );
        assert!(
            rpr_rope_soft_body_desc(Vector::ZERO.into(), Vector::X.into(), 0)
                .raw()
                .is_err()
        );
    }
}
