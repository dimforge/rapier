use obj::raw::object::Polygon;
use rapier3d::parry::bounding_volume;
use rapier3d::parry::transformation::voxelization::FillMode;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

const VOXEL_SIZE: Real = 0.1; // 0.25;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * Voxel geometry type selection.
     */

    let settings = testbed.example_settings_mut();

    let geometry_mode = settings.get_or_set_string(
        "Voxels mode",
        0,
        vec![
            "PseudoBall".to_string(),
            "PseudoCube".to_string(),
            "TriMesh".to_string(),
        ],
    );
    let load_dot_vox = settings.get_or_set_bool("Load .vox", false);
    let load_obj = settings.get_or_set_bool("Load .obj", false);
    let load_big_world = settings.get_or_set_bool("Load big world", false);

    let primitive_geometry = if geometry_mode == 0 {
        VoxelPrimitiveGeometry::PseudoBall
    } else {
        VoxelPrimitiveGeometry::PseudoCube
    };
    let use_mesh = geometry_mode == 2;

    let voxels_to_mesh = |shape: &mut SharedShape| {
        let (vtx, idx) = shape.as_voxels().unwrap().to_trimesh();
        SharedShape::trimesh_with_flags(vtx, idx, TriMeshFlags::all()).unwrap()
    };

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
    // {
    //     let obj_path = "assets/3d/bowl.obj";
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
    //         let aabb = bounding_volume::details::local_point_cloud_aabb(&vertices);
    //         let center = aabb.center();
    //         let diag = (aabb.maxs - aabb.mins).norm();
    //
    //         vertices
    //             .iter_mut()
    //             .for_each(|p| *p = (*p - center.coords) * 0.2);
    //
    //         let indices: Vec<_> = indices
    //             .chunks(3)
    //             .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
    //             .collect();
    //
    //         let voxels =
    //             SharedShape::voxelized_mesh(&vertices, &indices, VOXEL_SIZE, FillMode::default());
    //         let collider = ColliderBuilder::new(voxels);
    //         colliders.insert(collider);
    //     }
    // }

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
            println!("Parsing and decomposing: {}", obj_path);

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
                let aabb = bounding_volume::details::point_cloud_aabb(&deltas, &vertices);
                let center = aabb.center();
                let diag = (aabb.maxs - aabb.mins).norm();

                vertices
                    .iter_mut()
                    .for_each(|p| *p = (*p - center.coords) * 6.0 / diag);

                let indices: Vec<_> = indices
                    .chunks(3)
                    .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
                    .collect();

                let mut decomposed_shape = SharedShape::voxelized_mesh(
                    primitive_geometry,
                    &vertices,
                    &indices,
                    VOXEL_SIZE,
                    FillMode::default(),
                );

                if use_mesh {
                    decomposed_shape = voxels_to_mesh(&mut decomposed_shape);
                }

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
     * Load Hytopia map.
     */
    let path = if load_big_world {
        "/Users/sebcrozet/work/hytopia/hytopia-main/sdk/examples/big-world/assets/map.json"
    } else {
        "/Users/sebcrozet/work/hytopia/hytopia-main/sdk/examples/zombies-fps/assets/maps/terrain.json"
    };

    let mut shape = voxels_from_hytopia(primitive_geometry, path);
    if use_mesh {
        shape = voxels_to_mesh(&mut shape);
    }
    let floor_aabb = shape.compute_local_aabb();
    colliders.insert(ColliderBuilder::new(shape));

    /*
     * Load .vox file.
     */
    if load_dot_vox {
        let path = "/Users/sebcrozet/Downloads/droid_one.vox";
        for mut shape in voxels_from_dot_vox(path, 1.0, primitive_geometry) {
            if use_mesh {
                shape = voxels_to_mesh(&mut shape);
            }

            colliders.insert(
                ColliderBuilder::new(shape)
                    .rotation(vector![-3.14 / 2.0, 0.0, 0.0])
                    .translation(vector![0.0, 10.0, 0.0]),
            );
        }
    }

    /*
     * Some dynamic primitives.
     */
    let nik = 30;
    let extents = floor_aabb.extents() * 0.75;
    let margin = (floor_aabb.extents() - extents) / 2.0;
    let ball_radius = if load_big_world { 1.0 } else { 0.5 };
    for i in 0..nik {
        for j in 0..5 {
            for k in 0..nik {
                let rb = RigidBodyBuilder::dynamic().translation(vector![
                    floor_aabb.mins.x + margin.x + i as f32 * extents.x / nik as f32,
                    floor_aabb.maxs.y + j as f32 * 2.0,
                    floor_aabb.mins.z + margin.z + k as f32 * extents.z / nik as f32,
                ]);
                let rb_handle = bodies.insert(rb);
                let co = ColliderBuilder::ball(ball_radius);
                colliders.insert_with_parent(co, rb_handle, &mut bodies);
            }
        }
    }

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

#[derive(serde::Serialize, serde::Deserialize)]
struct HytopiaBlock {
    coords: String,
    id: usize,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct HytopiaMap {
    blocks: serde_json::Value,
}

fn voxels_from_hytopia(
    primitive_geometry: VoxelPrimitiveGeometry,
    path: impl AsRef<Path>,
) -> SharedShape {
    let map_str = std::fs::read_to_string(&path).unwrap();
    let map: HytopiaMap = serde_json::from_str(&map_str).unwrap();

    let serde_json::Value::Object(fields) = map.blocks else {
        panic!("Incorrect map format.")
    };

    let voxels: Vec<_> = fields
        .keys()
        .map(|coords| {
            let coords: Vec<_> = coords.split_terminator(",").collect();
            let x = coords[0].parse::<f32>().unwrap();
            let y = coords[1].parse::<f32>().unwrap();
            let z = coords[2].parse::<f32>().unwrap();
            Point::new(x, y, z)
        })
        .collect();

    SharedShape::voxels(primitive_geometry, &voxels, 1.0)
}

fn voxels_from_dot_vox(
    path: impl AsRef<Path>,
    voxel_size: Real,
    primitive_geometry: VoxelPrimitiveGeometry,
) -> Vec<SharedShape> {
    let data = dot_vox::load(path.as_ref().to_str().unwrap()).unwrap();
    data.models
        .iter()
        .map(|model| {
            let centers: Vec<_> = model
                .voxels
                .iter()
                .map(|v| Point::new(v.x as f32, v.y as f32, v.z as f32) * voxel_size)
                .collect();
            SharedShape::voxels(primitive_geometry, &centers, voxel_size)
        })
        .collect()
}
