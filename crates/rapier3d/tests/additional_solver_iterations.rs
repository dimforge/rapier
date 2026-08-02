//! `RigidBody::additional_solver_iterations` semantics: the body's whole connected
//! component runs that many extra substeps (smaller per-group dt); other components
//! keep the base substep count.

use rapier3d::prelude::*;

struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    ccd: CCDSolver,
}

impl World {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd: CCDSolver::new(),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &IntegrationParameters::default(),
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }
}

/// A heavy cube resting on a light cube on the ground (high mass ratio through
/// contacts), with extra iterations requested on the heavy body.
fn build_heavy_stack(world: &mut World, extra_iters: usize) -> (RigidBodyHandle, RigidBodyHandle) {
    let ground = world.bodies.insert(RigidBodyBuilder::fixed());
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
        ground,
        &mut world.bodies,
    );

    let light = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).density(1.0),
        light,
        &mut world.bodies,
    );

    let heavy = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.5, 0.0))
            .additional_solver_iterations(extra_iters),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).density(200.0),
        heavy,
        &mut world.bodies,
    );

    (light, heavy)
}

/// A rope of impulse joints hanging from a fixed anchor with a heavy weight at
/// the end (high mass ratio through joints), extra iterations on the weight.
fn build_heavy_chain(world: &mut World, extra_iters: usize) -> RigidBodyHandle {
    let mut prev = world.bodies.insert(RigidBodyBuilder::fixed());
    let num_links = 6;

    for i in 0..num_links {
        let is_last = i == num_links - 1;
        let mut builder =
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, -(i as Real + 1.0), 0.0));
        if is_last {
            builder = builder.additional_solver_iterations(extra_iters);
        }
        let link = world.bodies.insert(builder);
        world.colliders.insert_with_parent(
            ColliderBuilder::ball(0.4).density(if is_last { 100.0 } else { 1.0 }),
            link,
            &mut world.bodies,
        );

        let joint = SphericalJointBuilder::new()
            .local_anchor1(Vector::new(0.0, -0.5, 0.0))
            .local_anchor2(Vector::new(0.0, 0.5, 0.0));
        world.impulse_joints.insert(prev, link, joint, true);
        prev = link;
    }

    prev
}

#[test]
fn heavy_stack_stays_stable_with_extra_iterations() {
    let mut world = World::new();
    let (light, heavy) = build_heavy_stack(&mut world, 16);

    for _ in 0..300 {
        world.step();
    }

    let light_pos = world.bodies[light].translation();
    let heavy_pos = world.bodies[heavy].translation();
    assert!(
        light_pos.y > 0.3 && light_pos.y < 0.7,
        "light box crushed or launched: y = {}",
        light_pos.y
    );
    assert!(
        heavy_pos.y > 1.2 && heavy_pos.y < 1.8,
        "heavy box sank or launched: y = {}",
        heavy_pos.y
    );
    assert!(
        world.bodies[heavy].linvel().length() < 0.1,
        "heavy box still moving: {:?}",
        world.bodies[heavy].linvel()
    );
}

#[test]
fn heavy_chain_stays_stable_with_extra_iterations() {
    let mut world = World::new();
    let end = build_heavy_chain(&mut world, 16);

    for _ in 0..300 {
        world.step();
    }

    let end_pos = world.bodies[end].translation();
    assert!(
        end_pos.y.is_finite() && end_pos.length() < 20.0,
        "chain exploded: end at {end_pos:?}"
    );
    // The rope is 6 links of length 1: the end must hang around y = -6, with
    // limited joint stretch despite the 100x mass ratio.
    assert!(
        end_pos.y > -7.5 && end_pos.y < -4.5,
        "chain over-stretched or bunched: end y = {}",
        end_pos.y
    );
}

/// The extra iterations must actually run: with them enabled the trajectory of
/// a not-yet-settled high-mass-ratio stack differs from the plain solve, while
/// two identical runs stay bitwise identical (determinism).
#[test]
fn extra_iterations_take_effect_and_are_deterministic() {
    let run = |extra: usize| {
        let mut world = World::new();
        let (light, heavy) = build_heavy_stack(&mut world, extra);
        // Drop the heavy cube from higher up so the early steps are dynamic.
        world.bodies[heavy].set_translation(Vector::new(0.1, 3.0, 0.0), true);
        for _ in 0..60 {
            world.step();
        }
        (
            world.bodies[light].translation(),
            world.bodies[heavy].translation(),
        )
    };

    let plain = run(0);
    let extra1 = run(8);
    let extra2 = run(8);

    assert_eq!(extra1, extra2, "extra iterations broke determinism");
    assert_ne!(
        plain, extra1,
        "additional_solver_iterations had no effect on the solve"
    );
}
