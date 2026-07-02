//! `<material>` elements inherit attributes from their `<default class=…>`
//! block (MuJoCo class-default resolution), and the loader maps the result to
//! PBR shading parameters.

use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRenderMaterial, MjcfRobot};

fn first_material(xml: &str) -> MjcfRenderMaterial {
    let opts = MjcfLoaderOptions {
        create_colliders_from_visual_shapes: false,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    robot
        .bodies
        .iter()
        .flat_map(|b| b.visual_meshes.iter())
        .find_map(|vm| vm.material)
        .expect("a visual mesh with a resolved material")
}

const MODEL: &str = r#"
<mujoco>
  <default>
    <default class="shiny">
      <material specular="0.9" shininess="0.8" metallic="1"/>
    </default>
  </default>
  <asset>
    <material name="from_class" class="shiny" rgba="1 0 0 1"/>
    <material name="overrides" class="shiny" shininess="0.1"/>
    <material name="plain" rgba="0 1 0 1"/>
  </asset>
  <worldbody>
    <body>
      <geom name="g_class" type="box" size="0.1 0.1 0.1" material="from_class"
            contype="0" conaffinity="0"/>
    </body>
    <body>
      <geom name="g_over" type="box" size="0.1 0.1 0.1" material="overrides"
            contype="0" conaffinity="0"/>
    </body>
    <body>
      <geom name="g_plain" type="box" size="0.1 0.1 0.1" material="plain"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

fn material_named(xml: &str, geom: &str) -> MjcfRenderMaterial {
    // The three geoms map 1:1 onto the three bodies, in declaration order.
    let opts = MjcfLoaderOptions {
        create_colliders_from_visual_shapes: false,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    let idx = match geom {
        "g_class" => 0,
        "g_over" => 1,
        "g_plain" => 2,
        _ => unreachable!(),
    };
    robot
        .bodies
        .iter()
        .filter(|b| !b.visual_meshes.is_empty())
        .nth(idx)
        .and_then(|b| b.visual_meshes.first())
        .and_then(|vm| vm.material)
        .expect("resolved material")
}

#[test]
fn material_inherits_class_defaults() {
    // `from_class` sets only rgba; specular/shininess/metallic come from the
    // `shiny` default class: metallic 1, reflectance 0.9, roughness 1 − 0.8.
    let m = material_named(MODEL, "g_class");
    assert!((m.metallic - 1.0).abs() < 1e-6, "metallic = {}", m.metallic);
    assert!(
        (m.reflectance - 0.9).abs() < 1e-6,
        "reflectance = {}",
        m.reflectance
    );
    assert!(
        (m.roughness - 0.2).abs() < 1e-6,
        "roughness = {} (expected 1 − shininess(0.8))",
        m.roughness
    );
}

#[test]
fn material_own_attribute_overrides_class_default() {
    // `overrides` keeps the class metallic/specular but sets its own shininess,
    // so roughness = 1 − 0.1, not 1 − 0.8.
    let m = material_named(MODEL, "g_over");
    assert!((m.metallic - 1.0).abs() < 1e-6, "metallic = {}", m.metallic);
    assert!(
        (m.roughness - 0.9).abs() < 1e-6,
        "own shininess(0.1) should win over class: roughness = {}",
        m.roughness
    );
}

#[test]
fn material_without_class_uses_mujoco_defaults() {
    // `plain` has no class: MuJoCo defaults specular/shininess 0.5, dielectric.
    let m = material_named(MODEL, "g_plain");
    assert!((m.metallic - 0.0).abs() < 1e-6, "metallic = {}", m.metallic);
    assert!(
        (m.reflectance - 0.5).abs() < 1e-6,
        "reflectance = {}",
        m.reflectance
    );
    assert!(
        (m.roughness - 0.5).abs() < 1e-6,
        "default shininess(0.5) → roughness = {}",
        m.roughness
    );
}

#[test]
fn smoke_first_material_resolves() {
    let _ = first_material(MODEL);
}
