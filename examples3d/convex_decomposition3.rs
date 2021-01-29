use kiss3d::loader::obj;
use na::{Point3, Translation3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, SharedShape};
use rapier3d::parry::bounding_volume::{self, BoundingVolume};
use rapier_testbed3d::Testbed;
use std::path::Path;

/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let ground_size = 50.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert(collider, handle, &mut bodies);

    /*
     * Create the convex decompositions.
     */
    let geoms = models();
    let ngeoms = geoms.len();
    let width = (ngeoms as f32).sqrt() as usize;
    let num_duplications = 4;
    let shift = 5.0f32;

    for (igeom, obj_path) in geoms.into_iter().enumerate() {
        let deltas = na::one();
        let mtl_path = Path::new("");

        let mut shapes = Vec::new();
        println!("Parsing and decomposing: {}", obj_path);
        let obj = obj::parse_file(&Path::new(&obj_path), &mtl_path, "");

        if let Ok(model) = obj {
            let meshes: Vec<_> = model
                .into_iter()
                .map(|mesh| mesh.1.to_trimesh().unwrap())
                .collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let mut aabb =
                bounding_volume::details::point_cloud_aabb(&deltas, &meshes[0].coords[..]);

            for mesh in meshes[1..].iter() {
                aabb.merge(&bounding_volume::details::point_cloud_aabb(
                    &deltas,
                    &mesh.coords[..],
                ));
            }

            let center = aabb.center().coords;
            let diag = (aabb.maxs - aabb.mins).norm();

            for mut trimesh in meshes.into_iter() {
                trimesh.translate_by(&Translation3::from(-center));
                trimesh.scale_by_scalar(6.0 / diag);

                let vertices = trimesh.coords;
                let indices: Vec<_> = trimesh
                    .indices
                    .unwrap_unified()
                    .into_iter()
                    .map(|idx| [idx.x, idx.y, idx.z])
                    .collect();

                let decomposed_shape = SharedShape::convex_decomposition(&vertices, &indices);
                shapes.push(decomposed_shape);
            }

            // let compound = SharedShape::compound(compound_parts);

            for k in 1..num_duplications + 1 {
                let x = (igeom % width) as f32 * shift;
                let y = (igeom / width) as f32 * shift + 4.0;
                let z = k as f32 * shift;

                let body = RigidBodyBuilder::new_dynamic().translation(x, y, z).build();
                let handle = bodies.insert(body);

                for shape in &shapes {
                    let collider = ColliderBuilder::new(shape.clone()).build();
                    colliders.insert(collider, handle, &mut bodies);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(100.0, 100.0, 100.0), Point3::origin());
}

fn models() -> Vec<String> {
    vec![
        "media/models/camel_decimated.obj".to_string(),
        "media/models/chair.obj".to_string(),
        "media/models/cup_decimated.obj".to_string(),
        "media/models/dilo_decimated.obj".to_string(),
        "media/models/feline_decimated.obj".to_string(),
        "media/models/genus3_decimated.obj".to_string(),
        "media/models/hand2_decimated.obj".to_string(),
        "media/models/hand_decimated.obj".to_string(),
        "media/models/hornbug.obj".to_string(),
        "media/models/octopus_decimated.obj".to_string(),
        "media/models/rabbit_decimated.obj".to_string(),
        "media/models/rust_logo.obj".to_string(),
        "media/models/rust_logo_simplified.obj".to_string(),
        "media/models/screwdriver_decimated.obj".to_string(),
        "media/models/table.obj".to_string(),
        "media/models/tstTorusModel.obj".to_string(),
        // "media/models/tstTorusModel2.obj".to_string(),
        // "media/models/tstTorusModel3.obj".to_string(),
    ]
}
