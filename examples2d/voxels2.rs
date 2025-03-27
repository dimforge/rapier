use rapier2d::parry::bounding_volume;
use rapier2d::parry::transformation::voxelization::FillMode;
use rapier2d::prelude::*;
use rapier_testbed2d::Testbed;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

const VOXEL_SIZE: Real = 0.1; // 0.25;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Voxel geometry type selection.
     */
    // TODO: make the testbed support custom enums (or at least a list of option from strings and
    //       associated constants).
    let settings = testbed.example_settings_mut();
    let geometry_mode = settings.get_or_set_string(
        "Voxels mode",
        0,
        vec!["PseudoBall".to_string(), "PseudoCube".to_string()],
    );

    let primitive_geometry = if geometry_mode == 0 {
        VoxelPrimitiveGeometry::PseudoBall
    } else {
        VoxelPrimitiveGeometry::PseudoCube
    };

    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create the convex decompositions.
     */
    // let geoms = models();
    // let ngeoms = geoms.len();
    // let width = (ngeoms as f32).sqrt() as usize;
    // let num_duplications = 1; // 4;
    // let shift = 7.0f32;
    //
    // for (igeom, obj_path) in geoms.into_iter().enumerate() {
    //     let deltas = Isometry::identity();
    //
    //     let mut shapes = Vec::new();
    //     println!("Parsing and decomposing: {}", obj_path);
    //
    //     let input = BufReader::new(File::open(obj_path).unwrap());
    //
    //     if let Ok(model) = obj::raw::parse_obj(input) {
    //         let mut vertices: Vec<_> = model
    //             .positions
    //             .iter()
    //             .map(|v| point![v.0, v.1, v.2])
    //             .collect();
    //         let indices: Vec<_> = model
    //             .polygons
    //             .into_iter()
    //             .flat_map(|p| match p {
    //                 Polygon::P(idx) => idx.into_iter(),
    //                 Polygon::PT(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
    //                 Polygon::PN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
    //                 Polygon::PTN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
    //             })
    //             .collect();
    //
    //         // Compute the size of the model, to scale it and have similar size for everything.
    //         let aabb = bounding_volume::details::point_cloud_aabb(&deltas, &vertices);
    //         let center = aabb.center();
    //         let diag = (aabb.maxs - aabb.mins).norm();
    //
    //         vertices
    //             .iter_mut()
    //             .for_each(|p| *p = (*p - center.coords) * 6.0 / diag);
    //
    //         let indices: Vec<_> = indices
    //             .chunks(3)
    //             .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
    //             .collect();
    //
    //         let decomposed_shape =
    //             SharedShape::voxelized_mesh(&vertices, &indices, VOXEL_SIZE, FillMode::default());
    //         shapes.push(decomposed_shape);
    //
    //         for k in 1..num_duplications + 1 {
    //             let x = (igeom % width) as f32 * shift - 3.0;
    //             let y = (igeom / width) as f32 * shift + 4.0;
    //             let z = k as f32 * shift - 3.0;
    //
    //             let body = RigidBodyBuilder::fixed().translation(vector![x, y, z]);
    //             let handle = bodies.insert(body);
    //
    //             for shape in &shapes {
    //                 let collider = ColliderBuilder::new(shape.clone());
    //                 colliders.insert_with_parent(collider, handle, &mut bodies);
    //             }
    //         }
    //     }
    // }

    for i in 0..30 {
        for j in 0..5 {
            let rb = RigidBodyBuilder::dynamic()
                .translation(vector![i as f32 * 2.0, 20.0 + j as f32 * 2.0]);
            let rb_handle = bodies.insert(rb);
            let co = ColliderBuilder::ball(1.0);
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
    let rb = bodies.insert(RigidBodyBuilder::fixed());
    let shape = SharedShape::voxelized_mesh(
        primitive_geometry,
        &polyline,
        &indices,
        0.2,
        FillMode::default(),
    );

    colliders.insert_with_parent(ColliderBuilder::new(shape), rb, &mut bodies);

    /*
     * Load .vox file.
     */
    let path = "/Users/sebcrozet/Downloads/droid_one.vox";

    for i in 0..3 {
        for shape in voxels_from_dot_vox(primitive_geometry, path, 1.0) {
            let rb = bodies.insert(
                RigidBodyBuilder::fixed()
                    .rotation(-3.14 / 2.0)
                    .translation(vector![0.0, 10.0 + 50.0 * i as f32]),
            );
            colliders.insert_with_parent(ColliderBuilder::new(shape), rb, &mut bodies);
        }
    }

    /*
     * Floor
     */
    colliders.insert(ColliderBuilder::cuboid(100.0, 1.0).translation(vector![0.0, -40.0]));

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![0.0, 20.0], 17.0);
}

fn voxels_from_dot_vox(
    primitive_geometry: VoxelPrimitiveGeometry,
    path: impl AsRef<Path>,
    voxel_size: Real,
) -> Vec<SharedShape> {
    let data = dot_vox::load(path.as_ref().to_str().unwrap()).unwrap();
    data.models
        .iter()
        .map(|model| {
            let centers: Vec<_> = model
                .voxels
                .iter()
                .map(|v| Point::new(v.y as f32, v.z as f32) * voxel_size) // FIXME: avoid duplicates
                .collect();
            SharedShape::voxels(primitive_geometry, &centers, voxel_size)
        })
        .collect()
}
