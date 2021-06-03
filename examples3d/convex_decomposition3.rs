use obj::raw::object::Polygon;
use rapier3d::parry::bounding_volume;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use std::fs::File;
use std::io::BufReader;

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
        .translation(vector![0.0, -ground_height, 0.0])
        .build();
    let handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size).build();
    colliders.insert_with_parent(collider, handle, &mut bodies);

    /*
     * Create the convex decompositions.
     */
    let geoms = models();
    let ngeoms = geoms.len();
    let width = (ngeoms as f32).sqrt() as usize;
    let num_duplications = 4;
    let shift = 5.0f32;

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
            use std::iter::FromIterator;
            let indices: Vec<_> = model
                .polygons
                .into_iter()
                .flat_map(|p| match p {
                    Polygon::P(idx) => idx.into_iter(),
                    Polygon::PT(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PTN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
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

            let decomposed_shape = SharedShape::convex_decomposition(&vertices, &indices);
            shapes.push(decomposed_shape);

            // let compound = SharedShape::compound(compound_parts);

            for k in 1..num_duplications + 1 {
                let x = (igeom % width) as f32 * shift;
                let y = (igeom / width) as f32 * shift + 4.0;
                let z = k as f32 * shift;

                let body = RigidBodyBuilder::new_dynamic()
                    .translation(vector![x, y, z])
                    .build();
                let handle = bodies.insert(body);

                for shape in &shapes {
                    let collider = ColliderBuilder::new(shape.clone()).build();
                    colliders.insert_with_parent(collider, handle, &mut bodies);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
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
        // "media/models/rust_logo.obj".to_string(),
        "media/models/rust_logo_simplified.obj".to_string(),
        "media/models/screwdriver_decimated.obj".to_string(),
        "media/models/table.obj".to_string(),
        "media/models/tstTorusModel.obj".to_string(),
        // "media/models/tstTorusModel2.obj".to_string(),
        // "media/models/tstTorusModel3.obj".to_string(),
    ]
}
