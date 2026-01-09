use rapier_testbed2d::Example;

mod balls2;
mod boxes2;
mod capsules2;
mod convex_polygons2;
mod heightfield2;
mod joint_ball2;
mod joint_fixed2;
mod joint_prismatic2;
mod pyramid2;
mod vertical_stacks2;

pub fn builders() -> Vec<Example> {
    const STRESS: &str = "Stress Tests";

    vec![
        Example::new(STRESS, "Balls", balls2::init_world),
        Example::new(STRESS, "Boxes", boxes2::init_world),
        Example::new(STRESS, "Capsules", capsules2::init_world),
        Example::new(STRESS, "Convex polygons", convex_polygons2::init_world),
        Example::new(STRESS, "Heightfield", heightfield2::init_world),
        Example::new(STRESS, "Pyramid", pyramid2::init_world),
        Example::new(STRESS, "Verticals stacks", vertical_stacks2::init_world),
        Example::new(STRESS, "(Stress test) joint ball", joint_ball2::init_world),
        Example::new(
            STRESS,
            "(Stress test) joint fixed",
            joint_fixed2::init_world,
        ),
        Example::new(
            STRESS,
            "(Stress test) joint prismatic",
            joint_prismatic2::init_world,
        ),
    ]
}
