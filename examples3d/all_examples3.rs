#![allow(dead_code)]
#![allow(clippy::type_complexity)]

use rapier_testbed3d::{Example, TestbedApp};

mod utils;

mod ccd3;
mod character_controller3;
mod collision_groups3;
mod compound3;
mod convex_decomposition3;
mod convex_polyhedron3;
mod damping3;
mod debug_add_remove_collider3;
mod debug_articulations3;
mod debug_balls3;
mod debug_big_colliders3;
mod debug_boxes3;
mod debug_chain_high_mass_ratio3;
mod debug_cube_high_mass_ratio3;
mod debug_cylinder3;
mod debug_deserialize3;
mod debug_disabled3;
mod debug_dynamic_collider_add3;
mod debug_friction3;
mod debug_infinite_fall3;
mod debug_internal_edges3;
mod debug_long_chain3;
mod debug_multibody_ang_motor_pos3;
mod debug_pop3;
mod debug_prismatic3;
mod debug_rollback3;
mod debug_shape_modification3;
mod debug_sleeping_kinematic3;
mod debug_thin_cube_on_mesh3;
mod debug_triangle3;
mod debug_trimesh3;
mod debug_two_cubes3;
mod domino3;
mod dynamic_trimesh3;
mod fountain3;
mod gyroscopic3;
mod heightfield3;
mod inverse_kinematics3;
mod joint_motor_position3;
mod joints3;
mod keva3;
mod locked_rotations3;
mod newton_cradle3;
mod one_way_platforms3;
mod platform3;
mod primitives3;
mod restitution3;
mod rope_joints3;
mod sensor3;
mod spring_joints3;
mod stress_tests;
mod trimesh3;
mod urdf3;
mod vehicle_controller3;
mod vehicle_joints3;
mod voxels3;

#[kiss3d::main]
pub async fn main() {
    const COLLISIONS: &str = "Collisions";
    const DYNAMICS: &str = "Dynamics";
    const COMPLEX: &str = "Complex Shapes";
    const JOINTS: &str = "Joints";
    const CONTROLS: &str = "Controls";
    const DEBUG: &str = "Debug";
    const ROBOTICS: &str = "Robotics";

    let mut builders: Vec<Example> = vec![
        // ── Collisions ──────────────────────────────────────────────────────────
        Example::new(COLLISIONS, "Fountain", fountain3::init_world),
        Example::new(COLLISIONS, "Primitives", primitives3::init_world),
        Example::new(COLLISIONS, "Keva tower", keva3::init_world),
        Example::new(COLLISIONS, "Newton cradle", newton_cradle3::init_world),
        Example::new(COLLISIONS, "Domino", domino3::init_world),
        Example::new(COLLISIONS, "Platform", platform3::init_world),
        Example::new(COLLISIONS, "Sensor", sensor3::init_world),
        Example::new(COLLISIONS, "Compound", compound3::init_world),
        #[cfg(not(target_arch = "wasm32"))]
        Example::new(
            COLLISIONS,
            "Convex decomposition",
            convex_decomposition3::init_world,
        ),
        Example::new(
            COLLISIONS,
            "Convex polyhedron",
            convex_polyhedron3::init_world,
        ),
        Example::new(COLLISIONS, "TriMesh", trimesh3::init_world),
        #[cfg(not(target_arch = "wasm32"))]
        Example::new(
            COLLISIONS,
            "Dynamic trimeshes",
            dynamic_trimesh3::init_world,
        ),
        Example::new(COLLISIONS, "Heightfield", heightfield3::init_world),
        Example::new(COLLISIONS, "Voxels", voxels3::init_world),
        Example::new(
            COLLISIONS,
            "Collision groups",
            collision_groups3::init_world,
        ),
        Example::new(
            COLLISIONS,
            "One-way platforms",
            one_way_platforms3::init_world,
        ),
        // ── Dynamics ─────────────────────────────────────────────────────────
        Example::new(DYNAMICS, "Locked rotations", locked_rotations3::init_world),
        Example::new(DYNAMICS, "Restitution", restitution3::init_world),
        Example::new(DYNAMICS, "Damping", damping3::init_world),
        Example::new(DYNAMICS, "Gyroscopic", gyroscopic3::init_world),
        Example::new(DYNAMICS, "CCD", ccd3::init_world),
        // ── Joints ─────────────────────────────────────────────────────────
        Example::new(JOINTS, "Impulse Joints", joints3::init_world_with_joints),
        Example::new(
            JOINTS,
            "Multibody Joints",
            joints3::init_world_with_articulations,
        ),
        Example::new(JOINTS, "Rope Joints", rope_joints3::init_world),
        Example::new(JOINTS, "Spring Joints", spring_joints3::init_world),
        Example::new(
            JOINTS,
            "Joint Motor Position",
            joint_motor_position3::init_world,
        ),
        Example::new(
            JOINTS,
            "Inverse kinematics",
            inverse_kinematics3::init_world,
        ),
        // ── Controls ─────────────────────────────────────────────────────
        Example::new(
            CONTROLS,
            "Character controller",
            character_controller3::init_world,
        ),
        Example::new(
            CONTROLS,
            "Vehicle controller",
            vehicle_controller3::init_world,
        ),
        Example::new(CONTROLS, "Vehicle joints", vehicle_joints3::init_world),
        // ── Robotics ───────────────────────────────────────────────────────
        #[cfg(not(target_arch = "wasm32"))]
        Example::new(ROBOTICS, "URDF", urdf3::init_world),
        // ── Debug ──────────────────────────────────────────────────────────
        Example::new(DEBUG, "Multibody joints", debug_articulations3::init_world),
        Example::new(
            DEBUG,
            "Add/rm collider",
            debug_add_remove_collider3::init_world,
        ),
        Example::new(DEBUG, "Big colliders", debug_big_colliders3::init_world),
        Example::new(DEBUG, "Boxes", debug_boxes3::init_world),
        Example::new(DEBUG, "Balls", debug_balls3::init_world),
        Example::new(DEBUG, "Disabled", debug_disabled3::init_world),
        Example::new(DEBUG, "Two cubes", debug_two_cubes3::init_world),
        Example::new(DEBUG, "Pop", debug_pop3::init_world),
        Example::new(
            DEBUG,
            "Dyn. collider add",
            debug_dynamic_collider_add3::init_world,
        ),
        Example::new(DEBUG, "Friction", debug_friction3::init_world),
        Example::new(DEBUG, "Internal edges", debug_internal_edges3::init_world),
        Example::new(DEBUG, "Long chain", debug_long_chain3::init_world),
        Example::new(
            DEBUG,
            "High mass ratio: chain",
            debug_chain_high_mass_ratio3::init_world,
        ),
        Example::new(
            DEBUG,
            "High mass ratio: cube",
            debug_cube_high_mass_ratio3::init_world,
        ),
        Example::new(DEBUG, "Triangle", debug_triangle3::init_world),
        Example::new(DEBUG, "Trimesh", debug_trimesh3::init_world),
        Example::new(DEBUG, "Thin cube", debug_thin_cube_on_mesh3::init_world),
        Example::new(DEBUG, "Cylinder", debug_cylinder3::init_world),
        Example::new(DEBUG, "Infinite fall", debug_infinite_fall3::init_world),
        Example::new(DEBUG, "Prismatic", debug_prismatic3::init_world),
        Example::new(DEBUG, "Rollback", debug_rollback3::init_world),
        Example::new(
            DEBUG,
            "Shape modification",
            debug_shape_modification3::init_world,
        ),
        Example::new(
            DEBUG,
            "Sleeping kinematics",
            debug_sleeping_kinematic3::init_world,
        ),
        #[cfg(not(target_arch = "wasm32"))]
        Example::new(DEBUG, "Deserialize", debug_deserialize3::init_world),
        Example::new(
            DEBUG,
            "Multibody ang. motor pos.",
            debug_multibody_ang_motor_pos3::init_world,
        ),
    ];
    let mut benches = stress_tests::builders();
    builders.append(&mut benches);

    let testbed = TestbedApp::from_builders(builders);
    testbed.run().await
}
