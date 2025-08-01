use obj::raw::object::Polygon;
use rapier_testbed3d::KeyCode;
use rapier_testbed3d::Testbed;
use rapier3d::parry::bounding_volume;
use rapier3d::parry::transformation::voxelization::FillMode;
use rapier3d::prelude::*;
use std::fs::File;
use std::io::BufReader;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Voxel geometry type selection.
     */

    let settings = testbed.example_settings_mut();

    let falling_objects = settings.get_or_set_string(
        "Falling objects",
        5, // Defaults to Mixed.
        vec![
            "Ball".to_string(),
            "Cuboid".to_string(),
            "Cylinder".to_string(),
            "Cone".to_string(),
            "Capsule".to_string(),
            "Mixed".to_string(),
        ],
    );

    let voxel_size_y = settings.get_or_set_f32("Voxel size y", 1.0, 0.5..=2.0);
    let voxel_size = Vector::new(1.0, voxel_size_y, 1.0);
    let test_ccd = settings.get_or_set_bool("Test CCD", false);

    // TODO: give a better placement to the objs.
    // settings.get_or_set_bool("Load .obj", false);
    let load_obj = false;

    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create a bowl for the ground
     */
    /*
     * Create the convex decompositions.
     */
    if load_obj {
        let geoms = models();
        let ngeoms = geoms.len();
        let width = (ngeoms as f32).sqrt() as usize;
        let num_duplications = 1; // 4;
        let shift = 7.0f32;

        for (igeom, obj_path) in geoms.into_iter().enumerate() {
            let deltas = Isometry::identity();

            let mut shapes = Vec::new();
            println!("Parsing and decomposing: {obj_path}");

            let input = BufReader::new(File::open(obj_path).unwrap());

            if let Ok(model) = obj::raw::parse_obj(input) {
                let mut vertices: Vec<_> = model
                    .positions
                    .iter()
                    .map(|v| point![v.0, v.1, v.2])
                    .collect();
                let indices: Vec<_> = model
                    .polygons
                    .into_iter()
                    .flat_map(|p| match p {
                        Polygon::P(idx) => idx.into_iter(),
                        Polygon::PT(idx) => {
                            Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter()
                        }
                        Polygon::PN(idx) => {
                            Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter()
                        }
                        Polygon::PTN(idx) => {
                            Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter()
                        }
                    })
                    .collect();

                // Compute the size of the model, to scale it and have similar size for everything.
                let aabb =
                    bounding_volume::details::point_cloud_aabb(&deltas, vertices.iter().copied());
                let center = aabb.center();
                let diag = (aabb.maxs - aabb.mins).norm();

                vertices
                    .iter_mut()
                    .for_each(|p| *p = (*p - center.coords) * 6.0 / diag);

                let indices: Vec<_> = indices
                    .chunks(3)
                    .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
                    .collect();

                let decomposed_shape =
                    SharedShape::voxelized_mesh(&vertices, &indices, 0.1, FillMode::default());

                shapes.push(decomposed_shape);

                for k in 1..num_duplications + 1 {
                    let x = (igeom % width) as f32 * shift - 3.0;
                    let y = (igeom / width) as f32 * shift + 4.0;
                    let z = k as f32 * shift - 3.0;

                    let body = RigidBodyBuilder::fixed().translation(vector![x, y, z]);
                    let handle = bodies.insert(body);

                    for shape in &shapes {
                        let collider = ColliderBuilder::new(shape.clone());
                        colliders.insert_with_parent(collider, handle, &mut bodies);
                    }
                }
            }
        }
    }

    /*
     * Create a voxelized wavy floor.
     */
    let mut samples = vec![];
    let n = 200;
    for i in 0..n {
        for j in 0..n {
            let y = (i as f32 / n as f32 * 10.0).sin().clamp(-0.8, 0.8)
                * (j as f32 / n as f32 * 10.0).cos().clamp(-0.8, 0.8)
                * 16.0;

            samples.push(point![i as f32, y * voxel_size_y, j as f32]);

            if i == 0 || i == n - 1 || j == 0 || j == n - 1 {
                // Create walls so the object at the edge don’t fall into the infinite void.
                for k in 0..4 {
                    samples.push(point![i as f32, (y + k as f32) * voxel_size_y, j as f32]);
                }
            }
        }
    }
    let collider = ColliderBuilder::voxels_from_points(voxel_size, &samples).build();
    let floor_aabb = collider.compute_aabb();
    colliders.insert(collider);

    /*
     * Some dynamic primitives.
     */
    let nik = 30;
    let extents = floor_aabb.extents() * 0.75;
    let margin = (floor_aabb.extents() - extents) / 2.0;
    let ball_radius = 0.5;
    for i in 0..nik {
        for j in 0..5 {
            for k in 0..nik {
                let mut rb = RigidBodyBuilder::dynamic().translation(vector![
                    floor_aabb.mins.x + margin.x + i as f32 * extents.x / nik as f32,
                    floor_aabb.maxs.y + j as f32 * 2.0,
                    floor_aabb.mins.z + margin.z + k as f32 * extents.z / nik as f32,
                ]);
                if test_ccd {
                    rb = rb.linvel(vector![0.0, -1000.0, 0.0]).ccd_enabled(true);
                }
                let rb_handle = bodies.insert(rb);

                let falling_objects = if falling_objects == 5 {
                    j % 5
                } else {
                    falling_objects
                };

                let co = match falling_objects {
                    0 => ColliderBuilder::ball(ball_radius),
                    1 => ColliderBuilder::cuboid(ball_radius, ball_radius, ball_radius),
                    2 => ColliderBuilder::cylinder(ball_radius, ball_radius),
                    3 => ColliderBuilder::cone(ball_radius, ball_radius),
                    4 => ColliderBuilder::capsule_y(ball_radius, ball_radius),
                    _ => unreachable!(),
                };
                colliders.insert_with_parent(co, rb_handle, &mut bodies);
            }
        }
    }

    // Add callback for handling voxels edition, and highlighting the voxel
    // pointed at by the mouse. We spawn two fake colliders that don’t interact
    // with anything. They are used as gizmos to indicate where the ray hits on voxels
    // by highlighting the voxel and drawing a small ball at the intersection.
    let hit_indicator_handle =
        colliders.insert(ColliderBuilder::ball(0.1).collision_groups(InteractionGroups::none()));
    let hit_highlight_handle = colliders.insert(
        ColliderBuilder::cuboid(0.51, 0.51, 0.51).collision_groups(InteractionGroups::none()),
    );
    testbed.set_initial_collider_color(hit_indicator_handle, [0.5, 0.5, 0.1]);
    testbed.set_initial_collider_color(hit_highlight_handle, [0.1, 0.5, 0.1]);

    testbed.add_callback(move |graphics, physics, _, _| {
        let Some(graphics) = graphics else { return };
        let Some((mouse_orig, mouse_dir)) = graphics.mouse().ray else {
            return;
        };

        let ray = Ray::new(mouse_orig, mouse_dir);
        let filter = QueryFilter {
            predicate: Some(&|_, co: &Collider| co.shape().as_voxels().is_some()),
            ..Default::default()
        };
        let query_pipeline = physics.broad_phase.as_query_pipeline(
            physics.narrow_phase.query_dispatcher(),
            &physics.bodies,
            &physics.colliders,
            filter,
        );

        if let Some((handle, hit)) = query_pipeline.cast_ray_and_get_normal(&ray, Real::MAX, true) {
            // Highlight the voxel.
            let hit_collider = &physics.colliders[handle];
            let hit_local_normal = hit_collider
                .position()
                .inverse_transform_vector(&hit.normal);
            let voxels = hit_collider.shape().as_voxels().unwrap();
            let FeatureId::Face(id) = hit.feature else {
                unreachable!()
            };
            let voxel_key = voxels.voxel_at_id(id);
            let voxel_center = hit_collider.position() * voxels.voxel_center(voxel_key);
            let voxel_size = voxels.voxel_size();
            let hit_highlight = physics.colliders.get_mut(hit_highlight_handle).unwrap();
            hit_highlight.set_translation(voxel_center.coords);
            hit_highlight
                .shape_mut()
                .as_cuboid_mut()
                .unwrap()
                .half_extents = voxel_size / 2.0 + Vector::repeat(0.001);
            graphics.update_collider(hit_highlight_handle, &physics.colliders);

            // Show the hit point.
            let hit_pt = ray.point_at(hit.time_of_impact);
            let hit_indicator = physics.colliders.get_mut(hit_indicator_handle).unwrap();
            hit_indicator.set_translation(hit_pt.coords);
            hit_indicator.shape_mut().as_ball_mut().unwrap().radius = voxel_size.norm() / 3.5;
            graphics.update_collider(hit_indicator_handle, &physics.colliders);

            // If a relevant key was pressed, edit the shape.
            if graphics.keys().pressed(KeyCode::Space) {
                let removal_mode = graphics.keys().pressed(KeyCode::ShiftLeft);
                let voxels = physics
                    .colliders
                    .get_mut(handle)
                    .unwrap()
                    .shape_mut()
                    .as_voxels_mut()
                    .unwrap();
                let mut affected_key = voxel_key;

                if !removal_mode {
                    let imax = hit_local_normal.iamax();
                    if hit_local_normal[imax] >= 0.0 {
                        affected_key[imax] += 1;
                    } else {
                        affected_key[imax] -= 1;
                    }
                }

                voxels.set_voxel(affected_key, !removal_mode);
                graphics.update_collider(handle, &physics.colliders);
            }
        } else {
            // When there is no hit, move the indicators behind the camera.
            let behind_camera = mouse_orig - mouse_dir * 1000.0;
            let hit_indicator = physics.colliders.get_mut(hit_indicator_handle).unwrap();
            hit_indicator.set_translation(behind_camera.coords);
            let hit_highlight = physics.colliders.get_mut(hit_highlight_handle).unwrap();
            hit_highlight.set_translation(behind_camera.coords);
        }
    });

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}

fn models() -> Vec<String> {
    vec![
        // "assets/3d/camel_decimated.obj".to_string(),
        "assets/3d/chair.obj".to_string(),
        "assets/3d/cup_decimated.obj".to_string(),
        "assets/3d/dilo_decimated.obj".to_string(),
        "assets/3d/feline_decimated.obj".to_string(),
        "assets/3d/genus3_decimated.obj".to_string(),
        "assets/3d/hand2_decimated.obj".to_string(),
        "assets/3d/hand_decimated.obj".to_string(),
        "assets/3d/hornbug.obj".to_string(),
        "assets/3d/octopus_decimated.obj".to_string(),
        "assets/3d/rabbit_decimated.obj".to_string(),
        // "assets/3d/rust_logo.obj".to_string(),
        "assets/3d/rust_logo_simplified.obj".to_string(),
        "assets/3d/screwdriver_decimated.obj".to_string(),
        "assets/3d/table.obj".to_string(),
        "assets/3d/tstTorusModel.obj".to_string(),
        // "assets/3d/tstTorusModel2.obj".to_string(),
        // "assets/3d/tstTorusModel3.obj".to_string(),
    ]
}
