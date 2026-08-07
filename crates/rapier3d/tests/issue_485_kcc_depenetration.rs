//! Regression test for #485: `KinematicCharacterController::move_shape` must not let
//! obstacles pass through the character when `desired_translation` is zero — a
//! depenetration pass now pushes it out of overlapping colliders even when it isn't moving.

use rapier3d::control::KinematicCharacterController;
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
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            Vector::ZERO,
            &IntegrationParameters::default(),
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut CCDSolver::new(),
            &(),
            &(),
        );
    }
}

/// A kinematic wall moved into a stationary character (zero desired translation) must push
/// the character in front of it instead of clipping through it.
#[test]
fn kinematic_platform_pushes_idle_character() {
    let mut world = World::new();
    let params = IntegrationParameters::default();

    // Kinematic wall moving along +x, starting behind the character.
    let wall_start = -3.0;
    let wall = world.bodies.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(wall_start, 0.0, 0.0)),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 2.0, 2.0),
        wall,
        &mut world.bodies,
    );

    // Stationary character at the origin.
    let character = world
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let character_collider = ColliderBuilder::ball(0.5).build();
    world
        .colliders
        .insert_with_parent(character_collider.clone(), character, &mut world.bodies);

    let controller = KinematicCharacterController::default();
    let wall_speed = 2.0;

    for i in 0..240 {
        let wall_x = wall_start + wall_speed * params.dt * (i + 1) as Real;
        world.bodies[wall].set_next_kinematic_translation(Vector::new(wall_x, 0.0, 0.0));
        world.step();

        let filter = QueryFilter::new().exclude_rigid_body(character);
        let queries = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            filter,
        );

        // The character does not try to move at all: zero desired translation.
        let movement = controller.move_shape(
            params.dt,
            &queries,
            character_collider.shape(),
            world.bodies[character].position(),
            Vector::ZERO,
            |_| {},
        );

        let new_pos = world.bodies[character].translation() + movement.translation;
        world.bodies[character].set_next_kinematic_translation(new_pos);
    }

    let wall_x = world.bodies[wall].translation().x;
    let character_x = world.bodies[character].translation().x;

    // The wall travelled way past the character's starting position…
    assert!(wall_x > 4.0, "unexpected wall position: {wall_x}");
    // … so the character must have been pushed in front of it: its surface
    // (character_x - 0.5) must not be inside or behind the wall's leading face
    // (wall_x + 0.5). A small tolerance accounts for the one-frame lag between
    // the wall's motion and the controller update.
    assert!(
        character_x - 0.5 >= wall_x + 0.5 - 0.05,
        "character clipped into/behind the wall: character_x = {character_x}, wall_x = {wall_x}"
    );
}

/// A character that starts overlapping a fixed collider must get pushed out of it by
/// `move_shape` even with a zero desired translation.
#[test]
fn overlapping_character_is_depenetrated() {
    let mut world = World::new();
    let params = IntegrationParameters::default();

    // Fixed box centered at the origin, half-extents (1, 1, 1).
    let obstacle = world.bodies.insert(RigidBodyBuilder::fixed());
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(1.0, 1.0, 1.0),
        obstacle,
        &mut world.bodies,
    );

    // One step so the broad-phase knows about the collider.
    world.step();

    // Ball of radius 0.5 spawned inside the box, closest to its +x face.
    let character_shape = ColliderBuilder::ball(0.5).build();
    let mut character_pos = Pose::from_translation(Vector::new(0.8, 0.3, 0.0));
    let controller = KinematicCharacterController::default();

    // The depenetration push is capped per call, so let a few calls run (as a game
    // loop would).
    for _ in 0..20 {
        let queries = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            QueryFilter::default(),
        );
        let movement = controller.move_shape(
            params.dt,
            &queries,
            character_shape.shape(),
            &character_pos,
            Vector::ZERO,
            |_| {},
        );
        character_pos = Pose::from_translation(movement.translation) * character_pos;
    }

    // The ball must have been expelled through the +x face: center at x >= 1.5 (touching).
    assert!(
        character_pos.translation.x >= 1.5 - 1.0e-3,
        "character still overlaps the box: pos = {:?}",
        character_pos.translation
    );
}
