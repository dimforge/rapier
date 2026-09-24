//! Every impulse-joint type attached to soft bodies through their cluster proxies: revolute,
//! spherical, fixed, prismatic, rope, spring and generic joints between jellies, cloths and rigid
//! or kinematic bodies, plus a same-soft-body hinge, a multibody arm and a kinematic cluster.

use rapier_testbed3d::TestbedViewer;
use rapier3d::prelude::*;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(18.0, 0.1, 18.0),
    );

    let jelly = |center: Vector, half: Vector, young: Real| {
        SoftBodyBuilder::cuboid(center, half, 4, 4, 4)
            .cell_model(SoftBodyCellModel::Corotational)
            .material(SoftBodyMaterial {
                young_modulus: young,
                poisson_ratio: 0.35,
                elastic_damping_ratio: 0.8,
                ..Default::default()
            })
            .particle_mass(0.08)
            .particle_radius(0.06)
            .surface_collider(ColliderBuilder::ball(0.06).friction(0.6))
    };

    /*
     * Revolute + velocity motor: a jelly spun about Y at its whole-body frame.
     */
    let spinner = world.insert_soft_body(jelly(
        Vector::new(-6.0, 1.6, -4.0),
        Vector::splat(0.5),
        8.0e3,
    ));
    let spinner_root = world.soft_bodies[spinner].root_body();
    let com = world.soft_bodies[spinner].center_of_mass();
    let pivot = world.insert_body(RigidBodyBuilder::fixed().translation(com));
    world.insert_impulse_joint(
        pivot,
        spinner_root,
        RevoluteJointBuilder::new(Vector::Y).motor_velocity(1.5, 60.0),
    );

    /*
     * Spherical: a cloth's corner cluster follows a circling kinematic mover (two-way: the
     * joint feels the cloth's weight).
     */
    let cloth = world.insert_soft_body(SoftBodyBuilder::cloth(
        Vector::new(0.0, 2.6, 2.5),
        Vector::X * 0.16,
        Vector::Z * 0.16,
        12,
        12,
    ));
    let corner = world
        .add_soft_body_cluster(cloth, &[0, 1, 12])
        .expect("corner cluster");
    let corner_proxy = world.soft_bodies[cloth].cluster_proxy(corner).unwrap();
    let corner_pos = world.bodies[corner_proxy].position().translation;
    let mover =
        world.insert_body(RigidBodyBuilder::kinematic_position_based().translation(corner_pos));
    world.insert_impulse_joint(mover, corner_proxy, SphericalJointBuilder::new());

    /*
     * Fixed: a rigid plate welded flat onto a jelly's top-face cluster; the full-rank face
     * cluster lets the weld hold the plate's orientation as the jelly wobbles.
     */
    let wobbler = world.insert_soft_body(jelly(
        Vector::new(-2.5, 0.61, -4.0),
        Vector::splat(0.6),
        2.5e3,
    ));
    // The top face of a 4x4x4 particle cuboid: the 16 particles of the highest layer.
    let top_face: Vec<u32> = {
        let sb = &world.soft_bodies[wobbler];
        let max_y = sb
            .particle_positions()
            .map(|p| p.y)
            .fold(0.0, |a: Real, b| a.max(b));
        (0..sb.num_particles() as u32)
            .filter(|&v| (sb.particle_position(v as usize).y - max_y).abs() < 1.0e-3)
            .collect()
    };
    let top_cluster = world
        .add_soft_body_cluster(wobbler, &top_face)
        .expect("top-face cluster");
    let top_proxy = world.soft_bodies[wobbler]
        .cluster_proxy(top_cluster)
        .unwrap();
    let top_pos = world.bodies[top_proxy].position().translation;
    let (plate, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(top_pos + Vector::new(0.0, 0.12, 0.0)),
        ColliderBuilder::cuboid(0.7, 0.06, 0.7).density(0.4),
    );
    world.insert_impulse_joint(
        plate,
        top_proxy,
        FixedJointBuilder::new().local_anchor1(Vector::new(0.0, -0.12, 0.0)),
    );
    // A small crate dropped on the plate to make it wobble.
    world.insert(
        RigidBodyBuilder::dynamic().translation(top_pos + Vector::new(0.3, 1.4, 0.2)),
        ColliderBuilder::cuboid(0.15, 0.15, 0.15).density(1.5),
    );

    /*
     * Prismatic + limits + position motor: a jelly shuttled along a rail between two stops
     * (the motor target oscillates in the render loop).
     */
    let shuttle = world.insert_soft_body(jelly(
        Vector::new(2.0, 0.85, -4.0),
        Vector::splat(0.4),
        6.0e3,
    ));
    let shuttle_root = world.soft_bodies[shuttle].root_body();
    let shuttle_com = world.soft_bodies[shuttle].center_of_mass();
    let rail = world.insert_body(RigidBodyBuilder::fixed().translation(shuttle_com));
    let rail_joint = world.insert_impulse_joint(
        rail,
        shuttle_root,
        PrismaticJointBuilder::new(Vector::X)
            .limits([-1.8, 1.8])
            .motor_position(0.0, 40.0, 8.0),
    );

    /*
     * Rope: two jellies chained over a ledge, one dragging the other.
     */
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(-1.0, 1.0, -9.0)),
        ColliderBuilder::cuboid(1.6, 1.0, 1.2),
    );
    let anchor_jelly = world.insert_soft_body(jelly(
        Vector::new(-1.0, 2.6, -9.0),
        Vector::splat(0.5),
        1.2e4,
    ));
    let hanging_jelly = world.insert_soft_body(jelly(
        Vector::new(1.6, 2.6, -9.0),
        Vector::splat(0.5),
        1.2e4,
    ));
    world.insert_impulse_joint(
        world.soft_bodies[anchor_jelly].root_body(),
        world.soft_bodies[hanging_jelly].root_body(),
        RopeJointBuilder::new(2.2),
    );

    /*
     * Spring: a bungee jelly bouncing under a gantry.
     */
    let bungee = world.insert_soft_body(jelly(
        Vector::new(5.5, 3.2, 2.5),
        Vector::splat(0.45),
        6.0e3,
    ));
    let gantry =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(5.5, 5.5, 2.5)));
    world.insert_impulse_joint(
        gantry,
        world.soft_bodies[bungee].root_body(),
        SpringJointBuilder::new(1.2, 25.0, 1.5),
    );

    /*
     * Generic: a jelly bead on a pole, translations locked to the pole's Y axis (limited along
     * it) and every rotation free: a 3D pin-slot joint spelled as a generic joint.
     */
    let pole_base = Vector::new(8.5, 0.0, -4.0);
    // The pole is visual only: the bead's particles would otherwise rest on it.
    world.insert(
        RigidBodyBuilder::fixed().translation(pole_base + Vector::new(0.0, 2.5, 0.0)),
        ColliderBuilder::cylinder(2.5, 0.05).collision_groups(InteractionGroups::none()),
    );
    let bead = world.insert_soft_body(jelly(
        pole_base + Vector::new(0.0, 4.2, 0.0),
        Vector::splat(0.35),
        8.0e3,
    ));
    let pole = world
        .insert_body(RigidBodyBuilder::fixed().translation(pole_base + Vector::new(0.0, 2.5, 0.0)));
    world.insert_impulse_joint(
        pole,
        world.soft_bodies[bead].root_body(),
        GenericJointBuilder::new(JointAxesMask::LIN_X | JointAxesMask::LIN_Z)
            .local_axis1(Vector::Y)
            .local_axis2(Vector::Y)
            .limits(JointAxis::LinY, [-1.8, 1.8]),
    );

    /*
     * Same-soft-body joint: one soft bar whose two disjoint half clusters are hinged by a
     * motorized revolute at its middle, so the body folds and flaps at its own hinge.
     */
    let bar = world.insert_soft_body(
        SoftBodyBuilder::cuboid(
            Vector::new(2.5, 2.2, 5.5),
            Vector::new(1.0, 0.22, 0.4),
            7,
            3,
            3,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 2.0e4,
            poisson_ratio: 0.35,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .particle_mass(0.08)
        .particle_radius(0.06)
        .surface_collider(ColliderBuilder::ball(0.06)),
    );
    let (left_half, right_half): (Vec<u32>, Vec<u32>) = {
        let sb = &world.soft_bodies[bar];
        let mid = sb.center_of_mass().x;
        let mut left = Vec::new();
        let mut right = Vec::new();
        for v in 0..sb.num_particles() as u32 {
            if sb.particle_position(v as usize).x < mid - 1.0e-3 {
                left.push(v);
            } else if sb.particle_position(v as usize).x > mid + 1.0e-3 {
                right.push(v);
            }
        }
        (left, right)
    };
    let left_cluster = world
        .add_soft_body_cluster(bar, &left_half)
        .expect("left half");
    let right_cluster = world
        .add_soft_body_cluster(bar, &right_half)
        .expect("right half");
    let left_proxy = world.soft_bodies[bar].cluster_proxy(left_cluster).unwrap();
    let right_proxy = world.soft_bodies[bar].cluster_proxy(right_cluster).unwrap();
    let bar_com = world.soft_bodies[bar].center_of_mass();
    let left_pos = world.bodies[left_proxy].position().translation;
    let right_pos = world.bodies[right_proxy].position().translation;
    // Hold the bar's left half in the air, then flap the right half about the middle hinge.
    let bar_anchor = world.insert_body(RigidBodyBuilder::fixed().translation(left_pos));
    world.insert_impulse_joint(bar_anchor, left_proxy, FixedJointBuilder::new());
    let flap_joint = world.insert_impulse_joint(
        left_proxy,
        right_proxy,
        RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(bar_com - left_pos)
            .local_anchor2(bar_com - right_pos)
            .motor_position(0.0, 80.0, 10.0),
    );

    /*
     * Multibody + rigid on one soft body: a jelly hanging from a two-link multibody arm (the
     * generic joint path) while a rope joint ties the same jelly to a rigid crate on the ground.
     */
    let arm_root =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(7.0, 5.0, 6.0)));
    let (link1, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(8.2, 5.0, 6.0)),
        ColliderBuilder::capsule_x(0.5, 0.08).density(2.0),
    );
    world
        .multibody_joints
        .insert(
            arm_root,
            link1,
            RevoluteJointBuilder::new(Vector::Z)
                .local_anchor1(Vector::ZERO)
                .local_anchor2(Vector::new(-1.2, 0.0, 0.0)),
            true,
        )
        .expect("arm joint 1");
    let (link2, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(9.4, 5.0, 6.0)),
        ColliderBuilder::capsule_x(0.5, 0.08).density(2.0),
    );
    world
        .multibody_joints
        .insert(
            link1,
            link2,
            RevoluteJointBuilder::new(Vector::Z)
                .local_anchor1(Vector::new(0.6, 0.0, 0.0))
                .local_anchor2(Vector::new(-0.6, 0.0, 0.0)),
            true,
        )
        .expect("arm joint 2");
    let pendulum = world.insert_soft_body(jelly(
        Vector::new(10.2, 4.2, 6.0),
        Vector::splat(0.5),
        5.0e3,
    ));
    let pendulum_root = world.soft_bodies[pendulum].root_body();
    world.insert_impulse_joint(
        link2,
        pendulum_root,
        SphericalJointBuilder::new()
            .local_anchor1(Vector::new(0.7, 0.0, 0.0))
            .local_anchor2(Vector::new(0.0, 0.6, 0.0)),
    );
    let (crate_body, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(10.2, 0.3, 6.0)),
        ColliderBuilder::cuboid(0.3, 0.3, 0.3).density(0.5),
    );
    world.insert_impulse_joint(pendulum_root, crate_body, RopeJointBuilder::new(3.2));

    /*
     * Kinematic cluster (no joint): a banner whose pinned top-edge cluster is waved rigidly.
     */
    let (banner_width, banner_height) = (14, 10);
    let banner = world.insert_soft_body(SoftBodyBuilder::cloth(
        Vector::new(-8.0, 3.2, 4.0),
        Vector::X * 0.18,
        Vector::Y * -0.18,
        banner_width,
        banner_height,
    ));
    // Cloth particles are column-major: particle `i * banner_height` is at the top of column `i`.
    let top_edge: Vec<u32> = (0..banner_width)
        .map(|i| (i * banner_height) as u32)
        .collect();
    let banner_grip = world
        .add_soft_body_cluster(banner, &top_edge)
        .expect("banner grip cluster");
    world.soft_bodies[banner].set_cluster_pinned(banner_grip, true);
    let grip_home = world.soft_bodies[banner]
        .cluster_proxy(banner_grip)
        .map(|proxy| world.bodies[proxy].position().translation)
        .unwrap_or_default();

    viewer.set_world(&mut world);
    viewer.look_at(Vec3::new(13.0, 9.0, 15.0), Vec3::new(0.0, 1.5, 0.0));

    let mut t: Real = 0.0;
    while viewer.render_frame(&mut world).await {
        if viewer.simulating() {
            t += world.integration_parameters.dt;
            // The spherical joint's kinematic mover circles.
            let target = corner_pos
                + Vector::new(
                    1.2 * (0.8 * t).sin(),
                    0.4 * (1.3 * t).sin(),
                    1.2 * (0.8 * t).cos() - 1.2,
                );
            world.bodies[mover].set_next_kinematic_translation(target);
            // The prismatic motor shuttles between its stops.
            if let Some(j) = world.impulse_joints.get_mut(rail_joint, true) {
                j.data
                    .set_motor_position(JointAxis::LinX, 1.5 * (0.6 * t).sin(), 40.0, 8.0);
            }
            // The same-body hinge flaps.
            if let Some(j) = world.impulse_joints.get_mut(flap_joint, true) {
                j.data
                    .set_motor_position(JointAxis::AngX, 0.8 * (1.4 * t).sin(), 80.0, 10.0);
            }
            // The banner's grip waves.
            let grip_pose = Pose::from_parts(
                grip_home + Vector::new(1.5 * (0.7 * t).sin(), 0.2 * (1.9 * t).sin(), 0.0),
                Rotation::from_axis_angle(Vector::X, 0.35 * (1.1 * t).sin()),
            );
            world.soft_bodies[banner].set_cluster_kinematic_target(banner_grip, grip_pose);
            world.step();
        }
    }
    Ok(())
}
