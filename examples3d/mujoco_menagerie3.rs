use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use kiss3d::color::Color;
use rapier_testbed3d::{RenderMaterial, Testbed, settings::StringDisplayMode};
use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot, MjcfRobotHandles};

/// Roots the scene-picker walks. Each entry is expected to contain `<robot>/scene*.xml` one level
/// deep. We recommend simply cloning `https://github.com/google-deepmind/mujoco_menagerie` into
/// the same parent directory as the Rapier repository.
const SCENE_ROOTS: &[&str] = &["../mujoco_menagerie"];
static LAST_FRAMED_SCENE: AtomicUsize = AtomicUsize::new(usize::MAX);

pub fn init_world(testbed: &mut Testbed) {
    let scenes = discover_scenes(SCENE_ROOTS);
    let labels: Vec<String> = scenes
        .iter()
        .map(|(p, root)| scene_label(p, root))
        .collect();

    // Default to unitree_a1 if it's there, otherwise pick the first scene.
    let default_idx = scenes
        .iter()
        .position(|(p, _)| p.to_string_lossy().contains("unitree_a1"))
        .unwrap_or(0);

    // Insert the checkboxes FIRST so they appear above the (taller) scene
    // list in the Example Settings panel — IndexMap preserves insertion
    // order on the first init_world call after the panel is fresh.
    let use_multibody = testbed
        .example_settings_mut()
        .get_or_set_bool("Use multibody joints", true);
    let render_colliders = testbed
        .example_settings_mut()
        .get_or_set_bool("Render colliders", false);
    let render_visual_meshes = testbed
        .example_settings_mut()
        .get_or_set_bool("Render visual meshes", true);
    // Some MJCF files (apptronik_apollo for example) declare every geom
    // with `contype=conaffinity=0`, so even their "collision class"
    // capsules end up in the visual channel. This toggle lets you hide
    // those non-mesh visual geoms while keeping the .obj-derived ones.
    let render_visual_primitives = testbed
        .example_settings_mut()
        .get_or_set_bool("Render visual primitives", false);
    let disable_collisions = testbed
        .example_settings_mut()
        .get_or_set_bool("Disable collisions", true);
    // Drives every actuator each frame with a zero control input — MuJoCo's
    // "actuation" flag. For affine `<general>`/`<position>` actuators (a
    // position servo) this holds each joint at its neutral pose, e.g. keeping
    // the flybody legs spread instead of letting them retract. Only effective
    // on the multibody path (controls are applied to multibody joints).
    let enable_controls = testbed
        .example_settings_mut()
        .get_or_set_bool("Enable joint controls", false);
    // MJCF `<joint stiffness>` passive springs (integrated implicitly on the
    // multibody path). Unchecking removes them — useful to see a model without
    // its return springs (e.g. robotiq's gripper preload, cassie's leg springs).
    let enable_springs = testbed
        .example_settings_mut()
        .get_or_set_bool("Enable joint springs", true);
    let selected = testbed.example_settings_mut().get_or_set_string_with(
        "Scene",
        default_idx,
        labels,
        StringDisplayMode::List,
    );

    if scenes.is_empty() {
        testbed.example_settings_mut().set_label("NO MODEL FOUND", "Consider cloning `google-deepmind/mujoco_menagerie`\ninto the same parent directory as the rapier repo.")
    }

    let mut world = PhysicsWorld::new();
    let mut loaded = None;
    let mut mb_handles = None;
    if let Some((path, _)) = scenes.get(selected) {
        match load_into_world(
            path,
            &mut world,
            use_multibody,
            disable_collisions,
            enable_springs,
        ) {
            Ok((robot, body_handles, mb)) => {
                loaded = Some((robot, body_handles));
                mb_handles = mb;
            }
            Err(e) => eprintln!("Failed to load `{}`: {e}", path.display()),
        }
        add_floor(&mut world);
    }
    testbed.set_physics_world(world);

    // "Enable joint controls" → drive the model's actuators every frame with a
    // zero control vector (hold the neutral pose). Re-registered on each
    // `init_world` (the testbed clears callbacks when a setting toggles), so
    // ticking the checkbox turns actuation on/off. Multibody path only.
    if enable_controls && let Some(handles) = mb_handles {
        let ctrl = vec![0.0 as Real; handles.actuators.len()];
        testbed.add_callback(move |_, physics, _, _| {
            handles.apply_controls_multibody(
                &mut physics.bodies,
                &mut physics.multibody_joints,
                &ctrl,
            );
        });
    }
    // MJCF is Z-up by default — keep the camera convention consistent
    // with the model so orbit controls feel right.
    testbed.set_up_axis(Vec3::Z);

    if !use_multibody {
        testbed.integration_parameters_mut().dt = 1.0 / 240.0;
        testbed.integration_parameters_mut().num_solver_iterations = 12;
    } else {
        testbed.integration_parameters_mut().num_internal_pgs_iterations = 4;
    }

    // Refit the camera only on actual scene changes, otherwise a
    // checkbox toggle in the Example Settings panel would snap the
    // camera out from under the user every time.
    if LAST_FRAMED_SCENE.swap(selected, Ordering::Relaxed) != selected {
        testbed.request_frame_all();
    }

    if let Some((robot, body_handles)) = loaded
        && render_visual_meshes
    {
        register_visual_meshes(testbed, &robot, &body_handles, render_visual_primitives);
    }

    testbed.set_colliders_visible(render_colliders);
    testbed.set_body_render_meshes_visible(render_visual_meshes);
}

fn loader_options() -> MjcfLoaderOptions {
    // No `make_roots_fixed` argument tweaks: bodies that are jointed to the
    // world need to remain Dynamic so the multibody insertion path can
    // chain them as non-root links. Bodies welded to the world (no joint)
    // get forced to Fixed automatically by the loader.
    MjcfLoaderOptions {
        skip_plane_geoms: true,
        make_roots_fixed: false,
        // Visual `<geom>` elements (contype=conaffinity=0) are surfaced
        // as `MjcfBody::visual_meshes` instead of becoming colliders.
        // We hand them to the testbed below via `add_body_render_mesh`.
        create_colliders_from_visual_shapes: false,
        // The collider blueprint applies to every collider the loader
        // builds. Density 0 (mass comes from <inertial>) plus an empty
        // collision group keeps non-physics geometry from interfering
        // with contacts.
        collider_blueprint: ColliderBuilder::ball(0.5).density(0.0),
        ..Default::default()
    }
}

fn add_floor(world: &mut PhysicsWorld) {
    if world.colliders.is_empty() {
        return;
    }

    let mut total_aabb = Aabb::new_invalid();
    for (_, collider) in world.all_colliders() {
        total_aabb.merge(&collider.compute_aabb());
    }

    let mut floor_hext = total_aabb.half_extents();
    let mut floor_center = total_aabb.center();
    floor_hext.x *= 10.0;
    floor_hext.y *= 10.0;
    floor_center.z -= floor_hext.z;
    floor_hext.z = 0.2;
    floor_center.z -= 0.2;
    world.insert_collider(
        ColliderBuilder::cuboid(floor_hext.x, floor_hext.y, floor_hext.z).translation(floor_center),
        None,
    );
}

/// Loads the MJCF at `path` into `world` using either the multibody or
/// the impulse-joint insertion path, and returns the source robot
/// together with the rapier body handle of each `MjcfBody` (`None` for
/// bodies that weren't inserted — typically the implicit world body when
/// no joint references it). The third element is the full multibody handle
/// set (`Some` only on the multibody path), kept so the caller can drive the
/// model's actuators via `apply_controls_multibody`.
type MultibodyHandles = MjcfRobotHandles<Option<MultibodyJointHandle>>;
fn load_into_world(
    path: &Path,
    world: &mut PhysicsWorld,
    use_multibody: bool,
    disable_collisions: bool,
    enable_springs: bool,
) -> Result<
    (MjcfRobot, Vec<Option<RigidBodyHandle>>, Option<MultibodyHandles>),
    Box<dyn std::error::Error>,
> {
    let (mut robot, model) = MjcfRobot::from_file(path, loader_options())?;

    if disable_collisions {
        for link in &mut robot.bodies {
            for collider in &mut link.colliders {
                collider.set_collision_groups(InteractionGroups::new(
                    Group::GROUP_1,
                    Group::GROUP_2,
                    Default::default(),
                ));
            }
        }
    }

    // MJCF specifies gravity as a 3-vector — typically (0, 0, -9.81)
    // since MJCF is Z-up by default. The testbed uses Z-up too (see
    // `set_up_axis` in `init_world`), so we lock the direction to the
    // up-axis (-Z = "down") and keep only the magnitude from the model.
    // This keeps physics and rendering aligned even if a stray model
    // declares gravity along a non-Z axis.
    let gx = model.option.gravity[0] as f32;
    let gy = model.option.gravity[1] as f32;
    let gz = model.option.gravity[2] as f32;
    let gravity_mag = (gx * gx + gy * gy + gz * gz).sqrt();
    world.gravity = Vector::new(0.0, 0.0, -gravity_mag);

    let mut mb_options = if disable_collisions {
        MjcfMultibodyOptions::DISABLE_SELF_CONTACTS
    } else {
        MjcfMultibodyOptions::default()
    };
    // `<joint stiffness>` springs are integrated implicitly by default;
    // unchecking the toggle strips them so you can see the model without its
    // passive return springs (and contrast against the implicit-spring fix).
    if !enable_springs {
        mb_options |= MjcfMultibodyOptions::SKIP_JOINT_SPRINGS;
    }

    let (body_handles, mb_handles) = if use_multibody {
        let handles = robot.clone().insert_using_multibody_joints(
            &mut world.bodies,
            &mut world.colliders,
            &mut world.multibody_joints,
            &mut world.impulse_joints,
            mb_options,
        );
        let body_handles = handles
            .bodies
            .iter()
            .map(|b| b.as_ref().map(|h| h.body))
            .collect();
        (body_handles, Some(handles))
    } else {
        let handles = robot.clone().insert_using_impulse_joints(
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
        );
        let body_handles = handles
            .bodies
            .into_iter()
            .map(|b| b.map(|h| h.body))
            .collect();
        (body_handles, None)
    };

    Ok((robot, body_handles, mb_handles))
}

/// For each MJCF body that has visual meshes, register them against the
/// matching rapier body so the testbed renders them. RGBA is taken from
/// the loader's resolved color (geom rgba → material rgba), falling
/// back to white when a texture is present (so the texture displays
/// in its native colors) or to a neutral grey otherwise. UVs and the
/// texture file path flow through to the testbed when both are
/// available.
fn register_visual_meshes(
    testbed: &mut Testbed,
    robot: &MjcfRobot,
    body_handles: &[Option<RigidBodyHandle>],
    include_primitives: bool,
) {
    let untextured_fallback = Color::new(0.7, 0.7, 0.75, 1.0);
    let textured_fallback = Color::new(1.0, 1.0, 1.0, 1.0);
    for (i, handle) in body_handles.iter().enumerate() {
        let Some(handle) = handle else { continue };
        let Some(mjcf_body) = robot.bodies.get(i) else {
            continue;
        };
        for vm in &mjcf_body.visual_meshes {
            // `MjcfBody::visual_meshes` covers every geom with
            // `contype=conaffinity=0`, regardless of whether it came
            // from a `<geom type="mesh">` or a primitive (capsule,
            // box, sphere, …). Some MJCFs lean on the primitive
            // variety as collision approximations they don't want
            // active in physics — let the user opt out of those.
            let is_mesh = matches!(
                vm.shape.shape_type(),
                rapier3d::geometry::ShapeType::TriMesh
            );
            if !is_mesh && !include_primitives {
                continue;
            }
            let textured = vm.texture.is_some();
            let color = vm
                .rgba
                .map(|c| Color::new(c[0], c[1], c[2], c[3]))
                .unwrap_or(if textured {
                    textured_fallback
                } else {
                    untextured_fallback
                });
            let material = vm.material.map(|m| RenderMaterial {
                metallic: m.metallic,
                roughness: m.roughness,
                reflectance: m.reflectance,
                emissive: m.emissive,
            });
            testbed.add_body_render_mesh(
                *handle,
                &vm.shape,
                vm.local_pose,
                color,
                vm.uvs.as_deref(),
                vm.normals.as_deref(),
                vm.texture.as_deref(),
                material,
            );
        }
    }
}

/// Walk each root one level deep and collect any `scene*.xml` found in
/// a sub-directory. Each result is paired with the root it came from so
/// the picker label can disambiguate same-named robots across roots.
/// Sorted by full path so the picker is stable.
fn discover_scenes(roots: &[&str]) -> Vec<(PathBuf, PathBuf)> {
    let mut scenes = Vec::new();
    for root in roots {
        let root = PathBuf::from(root);
        let Ok(top) = fs::read_dir(&root) else {
            continue;
        };
        for top_entry in top.flatten() {
            let dir = top_entry.path();
            if !dir.is_dir() {
                continue;
            }
            let Ok(sub) = fs::read_dir(&dir) else {
                continue;
            };
            for sub_entry in sub.flatten() {
                let path = sub_entry.path();
                if !path.is_file() {
                    continue;
                }
                let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
                if name.starts_with("scene") && name.ends_with(".xml") {
                    scenes.push((path, root.clone()));
                }
            }
        }
    }
    scenes.sort();
    scenes
}

/// `<root>/<dir>/<file>` short form, e.g. `menagerie/unitree_a1/scene.xml`,
/// so picks from different roots remain distinguishable.
fn scene_label(path: &Path, root: &Path) -> String {
    let root_name = root.file_name().and_then(|s| s.to_str()).unwrap_or("?");
    let parent = path
        .parent()
        .and_then(|p| p.file_name())
        .and_then(|s| s.to_str())
        .unwrap_or("?");
    let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("?.xml");
    format!("{root_name}/{parent}/{name}")
}
