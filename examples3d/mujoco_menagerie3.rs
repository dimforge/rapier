use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use kiss3d::color::Color;
use rapier_testbed3d::{RenderMaterial, TestbedViewer, settings::StringDisplayMode};
use rapier3d::prelude::*;
use rapier3d_mjcf::mjcf_rs::extras::Keyframe;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot, MjcfRobotHandles};

/// Roots the scene-picker walks. Each entry is expected to contain `<robot>/scene*.xml` one level
/// deep. We recommend simply cloning `https://github.com/google-deepmind/mujoco_menagerie` into
/// the same parent directory as the Rapier repository.
const SCENE_ROOTS: &[&str] = &["../mujoco_menagerie"];
static LAST_FRAMED_SCENE: AtomicUsize = AtomicUsize::new(usize::MAX);

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
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
    let use_multibody = viewer
        .example_settings_mut()
        .get_or_set_bool("Use multibody joints", true);
    let render_colliders = viewer
        .example_settings_mut()
        .get_or_set_bool("Render colliders", false);
    let render_visual_meshes = viewer
        .example_settings_mut()
        .get_or_set_bool("Render visual meshes", true);
    // Some MJCF files (apptronik_apollo for example) declare every geom
    // with `contype=conaffinity=0`, so even their "collision class"
    // capsules end up in the visual channel. This toggle lets you hide
    // those non-mesh visual geoms while keeping the .obj-derived ones.
    let render_visual_primitives = viewer
        .example_settings_mut()
        .get_or_set_bool("Render visual primitives", false);
    let disable_collisions = viewer
        .example_settings_mut()
        .get_or_set_bool("Disable collisions", true);
    // Drives every actuator each frame with a zero control input — MuJoCo's
    // "actuation" flag. For affine `<general>`/`<position>` actuators (a
    // position servo) this holds each joint at its neutral pose, e.g. keeping
    // the flybody legs spread instead of letting them retract. Only effective
    // on the multibody path (controls are applied to multibody joints).
    let enable_controls = viewer
        .example_settings_mut()
        .get_or_set_bool("Enable joint controls", true);
    // Uniformly scales actuator strength (gains + force limits) when driving the
    // model. Read live by the controls callback, so it's a non-restart setting
    // (below). Turn it down to make a servo-driven move between keyframes ease in
    // instead of snapping for models whose actuators are otherwise strong enough
    // to arrive almost instantly.
    viewer
        .example_settings_mut()
        .get_or_set_f32("Actuator strength", 1.0, 0.02..=2.0);
    // MJCF `<joint stiffness>` passive springs (integrated implicitly on the
    // multibody path). Unchecking removes them — useful to see a model without
    // its return springs (e.g. robotiq's gripper preload, cassie's leg springs).
    let enable_springs = viewer
        .example_settings_mut()
        .get_or_set_bool("Enable joint springs", true);
    // The keyframe to apply right after loading, so the robot starts in a
    // declared pose (e.g. a quadruped standing instead of collapsed at the
    // all-zeros configuration). The real option list is filled once the model
    // is loaded (it depends on which keyframes the model declares); reserve the
    // slot here so it renders above the taller scene list.
    viewer
        .example_settings_mut()
        .get_or_set_string("Keyframe", 0, vec!["(none)".to_string()]);
    // When the actuators drive toward the keyframe (multibody + controls on),
    // changing the keyframe is a *live* retarget — the per-step callback reads
    // the new selection and the joints move there. So don't restart the sim on
    // a keyframe change in that mode. Without controls there's nothing to track
    // the target, so the keyframe is applied by reloading (a restart) instead.
    let keyframe_is_live = use_multibody && enable_controls;
    viewer
        .example_settings_mut()
        .set_restart_on_change("Keyframe", !keyframe_is_live);
    // The controls callback reads "Actuator strength" live each step, so don't
    // restart the sim when the slider moves.
    viewer
        .example_settings_mut()
        .set_restart_on_change("Actuator strength", false);
    let selected = viewer.example_settings_mut().get_or_set_string_with(
        "Scene",
        default_idx,
        labels,
        StringDisplayMode::List,
    );
    // `init_world` re-runs on every settings change, not just scene changes.
    // Detect an actual scene change so the keyframe picker resets to the new
    // model's default (and the camera reframes) only then.
    let scene_changed = LAST_FRAMED_SCENE.swap(selected, Ordering::Relaxed) != selected;

    if scenes.is_empty() {
        viewer.example_settings_mut().set_label("NO MODEL FOUND", "Consider cloning `google-deepmind/mujoco_menagerie`\ninto the same parent directory as the rapier repo.")
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
            viewer,
            scene_changed,
        ) {
            Ok((robot, body_handles, mb)) => {
                loaded = Some((robot, body_handles));
                mb_handles = mb;
            }
            Err(e) => eprintln!("Failed to load `{}`: {e}", path.display()),
        }
        add_floor(&mut world);
    }
    viewer.set_world(&mut world);

    // "Enable joint controls" → keep the model at the *currently selected*
    // keyframe every step. Re-registered on each `init_world` (the testbed
    // clears callbacks when a setting toggles). Multibody only.
    //
    // The callback reads the live "Keyframe" selection from the settings each
    // step. Because that setting is marked non-restart (above), switching it
    // doesn't rebuild the world — instead, on a change the callback applies the
    // new keyframe directly (an instant, full-pose update of every joint, no
    // sim reset) and then the actuators hold it. Applying the pose rather than
    // only retargeting the servos matters because some models' actuators are
    // too weak to drag their joints across (e.g. the shadow hand's fingers
    // barely move under servo force alone). Each keyframe's hold control is its
    // `ctrl` (or a target derived from its `qpos`), precomputed here; "(none)"
    // falls back to zeros (hold the neutral pose).
    let mut controls_state = if enable_controls
        && let Some(handles) = mb_handles
        && let Some(robot) = loaded.as_ref().map(|(robot, _)| robot.clone())
    {
        let per_keyframe_ctrl: Vec<Vec<Real>> = robot
            .keyframes
            .iter()
            .map(|k| robot.keyframe_controls(k))
            .collect();
        let neutral = vec![0.0_f32; handles.actuators.len()];
        // Seed with the keyframe already applied at load so we don't re-apply
        // it on the first step.
        let last_selected = viewer
            .example_settings_mut()
            .get_string_id("Keyframe")
            .unwrap_or(0);
        Some((handles, robot, per_keyframe_ctrl, neutral, last_selected))
    } else {
        None
    };
    // MJCF is Z-up by default — keep the camera convention consistent
    // with the model so orbit controls feel right.
    viewer.set_up_axis(Vec3::Z);

    if !use_multibody {
        world.integration_parameters.dt = 1.0 / 240.0;
        world.integration_parameters.num_solver_iterations = 12;
    } else {
        world.integration_parameters.num_internal_pgs_iterations = 4;
    }

    // Refit the camera only on actual scene changes, otherwise a
    // checkbox toggle in the Example Settings panel would snap the
    // camera out from under the user every time.
    if scene_changed {
        viewer.request_frame_all();
    }

    if let Some((robot, body_handles)) = loaded
        && render_visual_meshes
    {
        register_visual_meshes(viewer, &robot, &body_handles, render_visual_primitives);
    }

    viewer.set_colliders_visible(render_colliders);
    viewer.set_body_render_meshes_visible(render_visual_meshes);

    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            world.step();

            if let Some((handles, robot, per_keyframe_ctrl, neutral, last_selected)) =
                controls_state.as_mut()
            {
                let settings = viewer.example_settings();
                // Selection index 0 is "(none)"; index `i + 1` is keyframe `i`.
                let selected = settings.get_string_id("Keyframe").unwrap_or(*last_selected);
                // Live actuator-strength scale (1.0 = as authored; lower = softer).
                let gain: Real = settings.get_f32("Actuator strength").unwrap_or(1.0);
                let keyframe = selected.checked_sub(1).and_then(|i| robot.keyframes.get(i));
                let _ = keyframe;
                if selected != *last_selected {
                    // if let Some(key) = keyframe {
                    //     handles.apply_keyframe(
                    //         &mut world.bodies,
                    //         &mut world.multibody_joints,
                    //         &robot,
                    //         key,
                    //     );
                    // }
                    *last_selected = selected;
                }
                let ctrl = selected
                    .checked_sub(1)
                    .and_then(|i| per_keyframe_ctrl.get(i))
                    .map(|v| v.as_slice())
                    .unwrap_or(neutral.as_slice());
                handles.apply_controls_multibody_scaled(
                    &mut world.bodies,
                    &mut world.multibody_joints,
                    ctrl,
                    gain,
                );
            }
        }
    }
    Ok(())
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

/// Merge the keyframes from a sibling `keyframes.xml` (next to the scene file)
/// into `robot`, skipping any whose name is already present.
///
/// This follows the menagerie convention where a model's keyframes live in a
/// separate `keyframes.xml` meant to be `<include>`d by the user. Their `qpos`
/// are authored against the bare model, so they line up as long as the scene
/// only *appends* extra free bodies (props to manipulate) after the robot —
/// those land at the tail of `qpos` and are simply left unset by the keyframe.
fn merge_sibling_keyframes(robot: &mut MjcfRobot, scene_path: &Path) {
    let Some(kf_path) = scene_path.parent().map(|d| d.join("keyframes.xml")) else {
        return;
    };
    if !kf_path.exists() {
        return;
    }
    match MjcfRobot::from_file(&kf_path, loader_options()) {
        Ok((kf_robot, _)) => {
            let existing: std::collections::HashSet<String> = robot
                .keyframes
                .iter()
                .filter_map(|k| k.name.clone())
                .collect();
            for k in kf_robot.keyframes {
                if k.name.as_ref().is_none_or(|n| !existing.contains(n)) {
                    robot.keyframes.push(k);
                }
            }
        }
        Err(e) => eprintln!(
            "Failed to load sibling keyframes `{}`: {e}",
            kf_path.display()
        ),
    }
}

/// Fill the "Keyframe" picker with this model's keyframes (prefixed with a
/// "(none)" entry) and return the keyframe the user selected, if any.
///
/// The option list depends on the loaded model, so it is rewritten on every
/// load. On an actual scene change the selection resets to the model's default
/// (its `home` keyframe, else its first); otherwise the user's pick is
/// preserved by name across re-inits (toggling another setting re-runs
/// `init_world`, and a stale index could point at a different keyframe).
fn configure_keyframe_setting(
    viewer: &mut TestbedViewer,
    robot: &MjcfRobot,
    scene_changed: bool,
) -> Option<Keyframe> {
    // Index 0 is "(none)"; entry `i + 1` is `robot.keyframes[i]`.
    let mut names = vec!["(none)".to_string()];
    for (i, k) in robot.keyframes.iter().enumerate() {
        names.push(k.name.clone().unwrap_or_else(|| format!("key {i}")));
    }

    // Default: `home` if present, else the first keyframe, else "(none)".
    let default_idx = names
        .iter()
        .position(|n| n == "home")
        .unwrap_or(if robot.keyframes.is_empty() { 0 } else { 1 });

    let selected_idx = if scene_changed {
        default_idx
    } else {
        // Preserve the prior selection by name (the index may have shifted).
        viewer
            .example_settings_mut()
            .get_string("Keyframe")
            .and_then(|prev| names.iter().position(|n| n == prev))
            .unwrap_or(default_idx)
    };

    viewer
        .example_settings_mut()
        .set_string("Keyframe", selected_idx, names);

    selected_idx
        .checked_sub(1)
        .and_then(|i| robot.keyframes.get(i))
        .cloned()
}

fn load_into_world(
    path: &Path,
    world: &mut PhysicsWorld,
    use_multibody: bool,
    disable_collisions: bool,
    enable_springs: bool,
    viewer: &mut TestbedViewer,
    scene_changed: bool,
) -> Result<
    (
        MjcfRobot,
        Vec<Option<RigidBodyHandle>>,
        Option<MultibodyHandles>,
    ),
    Box<dyn std::error::Error>,
> {
    let (mut robot, model) = MjcfRobot::from_file(path, loader_options())?;
    // Some menagerie models (e.g. shadow_hand) ship their keyframes in a
    // standalone `keyframes.xml` that the scene file doesn't `<include>`, so
    // they'd otherwise never reach the picker. Pull in a sibling `keyframes.xml`
    // when present.
    merge_sibling_keyframes(&mut robot, path);

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

    // Now that the model's keyframes are known, populate the "Keyframe" picker
    // and resolve the user's selection (`None` ⇒ "(none)" ⇒ don't apply one).
    let keyframe = configure_keyframe_setting(viewer, &robot, scene_changed);

    let (body_handles, mb_handles) = if use_multibody {
        let handles = robot.clone().insert_using_multibody_joints(
            &mut world.bodies,
            &mut world.colliders,
            &mut world.multibody_joints,
            &mut world.impulse_joints,
            mb_options,
        );
        if let Some(key) = &keyframe {
            handles.apply_keyframe(&mut world.bodies, &mut world.multibody_joints, &robot, key);
        }
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
        if let Some(key) = &keyframe {
            handles.apply_keyframe(&mut world.bodies, &robot, key);
        }
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
    viewer: &mut TestbedViewer,
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
            viewer.add_body_render_mesh(
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
