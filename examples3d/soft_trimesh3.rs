//! The dynamic trimeshes demo's models as volumetric soft bodies: the lattice mesher fills each
//! boundary mesh with corotational tetrahedral cells and the model is worn as the drawn skin. The
//! settings panel picks the cover smoothing, crust, subdivision and FEM variants.

use obj::raw::object::Polygon;
use rapier_testbed3d::TestbedViewer;
use rapier_testbed3d::settings::StringDisplayMode;
use rapier3d::parry::bounding_volume;
use rapier3d::prelude::*;
use std::fs::File;
use std::io::BufReader;

/// How a model is filled with cells; the settings panel edits one of these.
struct Meshing {
    /// The size of the elements (the boundary refines below it, see `cover_subdivisions`).
    cell_size: Real,
    /// Whether only the model's surface is covered: a hollow shell that takes the model as
    /// it is, open meshes included.
    crust: bool,
    /// Shrink-wrap iterations flattening the cover's staircase (cover and crust).
    cover_smoothing: u32,
    /// How close to the model the wrap may pull the boundary, as a fraction of the local
    /// cell size: the standoff floor of the smoothed cover.
    cover_guard: Real,
    /// Halvings below the cell size for cells crossing the model's boundary (cover and crust).
    cover_subdivisions: u32,
}

impl Default for Meshing {
    fn default() -> Self {
        Self {
            cell_size: 2.0,
            crust: false,
            cover_smoothing: 20,
            cover_guard: 0.15,
            cover_subdivisions: 1,
        }
    }
}

impl Meshing {
    /// The size of the cells on the model's boundary: the cell size, halved per subdivision.
    fn boundary_cell(&self) -> Real {
        self.cell_size / (1 << self.cover_subdivisions) as Real
    }
}

const BODY_COLOR: [f32; 4] = [0.85, 0.35, 0.3, 1.0];

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * Example settings.
     */
    let settings = viewer.example_settings_mut();

    // Overwritten with the real count once the bodies are built; registered here so the
    // readout sits at the top of the panel.
    settings.set_label("Tetrahedra", "...");

    let mut model_options = vec!["All models".to_string()];
    model_options.extend(models().iter().map(|path| {
        path.rsplit('/')
            .next()
            .unwrap_or(path)
            .trim_end_matches(".obj")
            .to_string()
    }));
    let selected_model =
        settings.get_or_set_string_with("Model", 0, model_options, StringDisplayMode::List);

    let defaults = Meshing::default();
    let meshing = Meshing {
        cell_size: settings.get_or_set_f32("Cell size", defaults.cell_size, 0.4..=4.0),
        crust: settings.get_or_set_bool("Crust (surface shell only)", defaults.crust),
        cover_smoothing: settings.get_or_set_u32(
            "Cover smoothing",
            defaults.cover_smoothing,
            0..=50,
        ),
        cover_guard: settings.get_or_set_f32("Cover guard", defaults.cover_guard, 0.02..=0.5),
        cover_subdivisions: settings.get_or_set_u32(
            "Cover subdivision",
            defaults.cover_subdivisions,
            0..=3,
        ),
    };
    // Drop the stale `Enclosure` entry left over from an earlier three-way choice.
    settings.remove("Enclosure");

    let wear_skin = settings.get_or_set_bool("Wear the model as a skin", true);
    let skin_collision = if wear_skin {
        settings.get_or_set_bool("Skin collisions", false)
    } else {
        settings.remove("Skin collisions");
        false
    };
    let cell_model = settings.get_or_set_string(
        "Cell model",
        0,
        vec![
            "Corotational".to_string(),
            "Neo-Hookean".to_string(),
            "Volume".to_string(),
        ],
    );
    let young_modulus = if cell_model == 2 {
        // The volume cells have no elastic modulus to tune.
        settings.remove("Young modulus");
        1.0e5
    } else {
        settings.get_or_set_f32("Young modulus", 1.0e5, 1.0e4..=1.0e6)
    };
    let self_contacts = settings.get_or_set_bool("Self contacts", false);

    let mut world = PhysicsWorld::new();

    /*
     * Wavy ground (as in the dynamic trimeshes demo).
     */
    let nsubdivs = 100;
    let heights = Array2::from_fn(nsubdivs + 1, nsubdivs + 1, |i, j| {
        -(i as f32 * 40.0 / (nsubdivs as f32) / 2.0).cos()
            - (j as f32 * 40.0 / (nsubdivs as f32) / 2.0).cos()
    });
    let heightfield = HeightField::new(heights, Vector::new(100.0, 2.0, 100.0));
    let mut trimesh = TriMesh::from(heightfield);
    let _ = trimesh.set_flags(TriMeshFlags::FIX_INTERNAL_EDGES);
    world.insert_collider(ColliderBuilder::new(SharedShape::new(trimesh)), None);

    /*
     * The models, as volumetric soft bodies.
     */
    let geoms: Vec<String> = models()
        .into_iter()
        .enumerate()
        .filter(|(igeom, _)| selected_model == 0 || *igeom + 1 == selected_model)
        .map(|(_, path)| path)
        .collect();

    let ngeoms = geoms.len();
    let width = (ngeoms as f32).sqrt().ceil().max(1.0) as usize;
    let shift_y = 8.0f32;
    let shift_xz = 9.0f32;
    let mut total_cells = 0;
    let mut total_bodies = 0;

    for (igeom, obj_path) in geoms.into_iter().enumerate() {
        let Some((vertices, indices)) = load_obj(&obj_path) else {
            continue;
        };
        let Some(filled) = fill_lattice(&vertices, &indices, &meshing) else {
            continue;
        };

        let x = (igeom % width) as f32 * shift_xz - (width - 1) as f32 * shift_xz / 2.0;
        let y = (igeom / width) as f32 * shift_y + 7.0;

        // The skin has to be worn before the body is moved: `translated` moves the
        // particles and whatever skin is already there, together.
        let filled = if wear_skin {
            filled
                .skin(vertices.clone(), indices.clone())
                .skin_collision(skin_collision)
        } else {
            filled
        };

        let model = filled
            .translated(Vector::new(x, y, 0.0))
            .cell_model(match cell_model {
                1 => SoftBodyCellModel::NeoHookean,
                2 => SoftBodyCellModel::Volume,
                _ => SoftBodyCellModel::Corotational,
            })
            .material(SoftBodyMaterial {
                young_modulus,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.5,
                // The models sway on their feet (solver compliance, undamped by the material
                // constraints): damp their deformation.
                deformation_damping: 2.5,
                ..Default::default()
            })
            .particle_mass(0.05)
            // A quarter of the boundary cell size: the subdivision is what sets the
            // boundary's resolution, so the contact skin follows it.
            .particle_radius(meshing.boundary_cell() * 0.25)
            .self_contacts(self_contacts)
            .surface_collider(ColliderBuilder::ball(0.1).friction(0.6));

        total_cells += model.cells.len();
        total_bodies += 1;
        let handle = world.insert_soft_body(model);
        viewer.set_initial_soft_body_color(handle, BODY_COLOR.into());
    }

    viewer.example_settings_mut().set_label(
        "Tetrahedra",
        format!("{total_cells} in {total_bodies} bodies"),
    );

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(60.0, 40.0, 60.0), Vec3::new(0.0, 5.0, 0.0));

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();
        }
    }
    Ok(())
}

/// Loads an OBJ file as a shared-vertex triangle mesh, centered and scaled to a 10-unit
/// diagonal (as in the dynamic trimeshes demo).
fn load_obj(path: &str) -> Option<(Vec<Vector>, Vec<[u32; 3]>)> {
    let input = BufReader::new(File::open(path).ok()?);
    let model = obj::raw::parse_obj(input).ok()?;
    let mut vertices: Vec<Vector> = model
        .positions
        .iter()
        .map(|v| Vector::new(v.0, v.1, v.2))
        .collect();
    let flat: Vec<usize> = model
        .polygons
        .into_iter()
        .flat_map(|p| match p {
            Polygon::P(idx) => idx.into_iter(),
            Polygon::PT(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
            Polygon::PN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
            Polygon::PTN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
        })
        .collect();
    let aabb =
        bounding_volume::details::point_cloud_aabb(&Pose::IDENTITY, vertices.iter().copied());
    let center = aabb.center();
    let diag = (aabb.maxs - aabb.mins).length();
    vertices
        .iter_mut()
        .for_each(|p| *p = (*p - Vector::new(center.x, center.y, center.z)) * 10.0 / diag);
    let indices: Vec<[u32; 3]> = flat
        .chunks(3)
        .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
        .collect();
    Some((vertices, indices))
}

/// The lattice mesher's parameters for the demo's settings.
fn lattice_params(meshing: &Meshing) -> rapier3d::parry::transformation::VolumeMeshParameters {
    use rapier3d::parry::transformation::MeshEnclosure;

    let mut params = rapier3d::parry::transformation::VolumeMeshParameters::new(meshing.cell_size);
    params.enclosure = if meshing.crust {
        MeshEnclosure::Crust
    } else {
        MeshEnclosure::Cover
    };
    params.cover_smoothing = meshing.cover_smoothing;
    params.cover_guard = meshing.cover_guard;
    params.cover_subdivisions = meshing.cover_subdivisions;
    params
}

/// Fills a model with the lattice mesher; the crust enclosure takes a model the lattice refuses
/// (not closed) as it is. Shared by the demo and the test that checks it, so they cannot drift.
fn fill_lattice(
    vertices: &[Vector],
    indices: &[[u32; 3]],
    meshing: &Meshing,
) -> Option<SoftBodyBuilder> {
    SoftBodyBuilder::volumetric_with(vertices, indices, &lattice_params(meshing))
}

/// The models of the dynamic trimeshes demo. `hornbug.obj` is not closed: it only gets
/// cells through the crust enclosure.
const OPEN_MODEL: &str = "assets/3d/hornbug.obj";

fn models() -> Vec<String> {
    vec![
        "assets/3d/camel_decimated.obj".to_string(),
        "assets/3d/chair.obj".to_string(),
        "assets/3d/cup_decimated.obj".to_string(),
        "assets/3d/dilo_decimated.obj".to_string(),
        "assets/3d/tstTorusModel2.obj".to_string(),
        "assets/3d/feline_decimated.obj".to_string(),
        "assets/3d/genus3_decimated.obj".to_string(),
        "assets/3d/tstTorusModel.obj".to_string(),
        "assets/3d/octopus_decimated.obj".to_string(),
        "assets/3d/rabbit_decimated.obj".to_string(),
        "assets/3d/rust_logo_simplified.obj".to_string(),
        "assets/3d/screwdriver_decimated.obj".to_string(),
        "assets/3d/table.obj".to_string(),
        "assets/3d/tstTorusModel3.obj".to_string(),
        OPEN_MODEL.to_string(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    /// The connected components of a cell mesh, largest first.
    fn components(positions: &[Vector], cells: &[[u32; 4]]) -> Vec<usize> {
        let mut parent: Vec<usize> = (0..positions.len()).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for cell in cells {
            for k in 1..4 {
                let (a, b) = (
                    find(&mut parent, cell[0] as usize),
                    find(&mut parent, cell[k] as usize),
                );
                parent[a] = b;
            }
        }
        let mut sizes: std::collections::HashMap<usize, usize> = Default::default();
        for cell in cells {
            *sizes
                .entry(find(&mut parent, cell[0] as usize))
                .or_insert(0) += 1;
        }
        let mut sizes: Vec<usize> = sizes.into_values().collect();
        sizes.sort_unstable_by(|a, b| b.cmp(a));
        sizes
    }

    /// The volume of a cell, positive when it is the right way round.
    fn cell_volume(positions: &[Vector], cell: [u32; 4]) -> Real {
        let [a, b, c, d] = cell.map(|index| positions[index as usize]);
        (b - a).cross(c - a).dot(d - a) / 6.0
    }

    /// Every closed model fills whole with the demo's default lattice settings (smoothed
    /// cover) and the grid settles on the ground without exploding; the open model is
    /// refused by default and shelled by the crust.
    #[test]
    fn demo_models_are_filled_and_settle() {
        let mut world = PhysicsWorld::new();
        world.insert_collider(
            ColliderBuilder::cuboid(100.0, 1.0, 100.0).translation(Vector::new(0.0, -1.0, 0.0)),
            None,
        );

        let meshing = Meshing::default();
        let geoms = models();
        let ngeoms = geoms.len();
        let width = (ngeoms as f32).sqrt() as usize;
        let mut cells = 0;
        let mut handles = Vec::new();
        let mut labels: Vec<String> = Vec::new();
        let build = Instant::now();

        for (igeom, obj_path) in geoms.into_iter().enumerate() {
            let path = format!("{}/../{obj_path}", env!("CARGO_MANIFEST_DIR"));
            let (vertices, indices) = load_obj(&path).unwrap_or_else(|| panic!("{path} missing"));

            let filled = fill_lattice(&vertices, &indices, &meshing);
            if obj_path == OPEN_MODEL {
                // The open model is refused by default; the crust takes it.
                assert!(filled.is_none(), "{obj_path} was expected to be refused");

                let crusted = fill_lattice(
                    &vertices,
                    &indices,
                    &Meshing {
                        crust: true,
                        ..Meshing::default()
                    },
                )
                .unwrap_or_else(|| panic!("{obj_path} could not be crusted"));
                assert!(
                    crusted
                        .cells
                        .iter()
                        .all(|cell| cell_volume(&crusted.positions, *cell) > 0.0)
                );
                println!("{obj_path:44} crust {:>5} cells", crusted.cells.len());
                continue;
            }

            let filled = filled.unwrap_or_else(|| panic!("{path} could not be filled"));

            // A model that comes out in pieces falls apart on screen. The lattice mesher
            // thickens the shape until it does not, so every closed model comes out whole.
            let parts = components(&filled.positions, &filled.cells);
            assert_eq!(
                parts.len(),
                1,
                "{obj_path} came out in {} pieces",
                parts.len()
            );
            println!("{obj_path:44} {:>5} cells", filled.cells.len());

            let filled = filled.skin(vertices.clone(), indices.clone());
            cells += filled.cells.len();

            // The model is worn as a skin, so nothing of it is lost to the cells'
            // resolution.
            let skin = filled.skin.as_ref().expect("skinned");
            assert_eq!(skin.0.len(), vertices.len());

            let offset = Vector::new(
                (igeom % width) as f32 * 9.0 - width as f32 * 9.0 / 2.0,
                (igeom / width) as f32 * 8.0 + 7.0,
                0.0,
            );
            labels.push(obj_path.clone());
            handles.push(
                world.insert_soft_body(
                    filled
                        .translated(offset)
                        .cell_model(SoftBodyCellModel::Corotational)
                        .material(SoftBodyMaterial {
                            young_modulus: 1.0e5,
                            poisson_ratio: 0.35,
                            elastic_damping_ratio: 0.5,
                            deformation_damping: 2.5,
                            ..Default::default()
                        })
                        .particle_mass(0.05)
                        .particle_radius(meshing.boundary_cell() * 0.25)
                        .surface_collider(ColliderBuilder::ball(0.1).friction(0.6)),
                ),
            );
        }

        println!(
            "{} bodies over {ngeoms} models, {cells} cells, built in {:?}",
            handles.len(),
            build.elapsed()
        );

        for _ in 0..600 {
            world.step();
        }

        for (handle, label) in handles.iter().zip(&labels) {
            let handle = *handle;
            let body = &world.soft_bodies[handle];
            let skin = body
                .meshes()
                .find(|mesh| mesh.is_skinned())
                .expect("skinned");
            assert!(
                skin.vertices().iter().all(|v| v.is_finite()),
                "{label}: the skin went non-finite"
            );
            let lowest = skin.vertices().iter().map(|v| v.y).fold(f32::MAX, f32::min);
            assert!(
                lowest < 7.0,
                "{label}: the skin did not come down with the cells"
            );

            let max_v = body
                .particles()
                .iter()
                .map(|p| p.velocity().length())
                .fold(0.0, f32::max);
            assert!(
                max_v.is_finite() && max_v < 20.0,
                "{label} exploded: {max_v} m/s"
            );
            let min_y = body
                .particle_positions()
                .map(|p| p.y)
                .fold(f32::MAX, f32::min);
            assert!(min_y > -0.5, "{label} sank through the ground: {min_y}");
        }
    }
}
