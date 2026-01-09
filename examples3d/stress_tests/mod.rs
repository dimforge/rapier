use rapier_testbed3d::Example;

mod balls3;
mod boxes3;
mod capsules3;
mod ccd3;
mod compound3;
mod convex_polyhedron3;
mod heightfield3;
mod joint_ball3;
mod joint_fixed3;
mod joint_prismatic3;
mod joint_revolute3;
mod keva3;
mod many_kinematics3;
mod many_pyramids3;
mod many_sleep3;
mod many_static3;
mod pyramid3;
mod ray_cast3;
mod stacks3;
mod trimesh3;

pub fn builders() -> Vec<Example> {
    const STRESS: &str = "Stress Tests";

    vec![
        Example::new(STRESS, "Balls", balls3::init_world),
        Example::new(STRESS, "Boxes", boxes3::init_world),
        Example::new(STRESS, "Capsules", capsules3::init_world),
        Example::new(STRESS, "CCD", ccd3::init_world),
        Example::new(STRESS, "Compound", compound3::init_world),
        Example::new(STRESS, "Convex polyhedron", convex_polyhedron3::init_world),
        Example::new(STRESS, "Many kinematics", many_kinematics3::init_world),
        Example::new(STRESS, "Many static", many_static3::init_world),
        Example::new(STRESS, "Many sleep", many_sleep3::init_world),
        Example::new(STRESS, "Heightfield", heightfield3::init_world),
        Example::new(STRESS, "Stacks", stacks3::init_world),
        Example::new(STRESS, "Pyramid", pyramid3::init_world),
        Example::new(STRESS, "Trimesh", trimesh3::init_world),
        Example::new(STRESS, "ImpulseJoint ball", joint_ball3::init_world),
        Example::new(STRESS, "ImpulseJoint fixed", joint_fixed3::init_world),
        Example::new(STRESS, "ImpulseJoint revolute", joint_revolute3::init_world),
        Example::new(
            STRESS,
            "ImpulseJoint prismatic",
            joint_prismatic3::init_world,
        ),
        Example::new(STRESS, "Many pyramids", many_pyramids3::init_world),
        Example::new(STRESS, "Keva tower", keva3::init_world),
        Example::new(STRESS, "Ray cast", ray_cast3::init_world),
    ]
}
