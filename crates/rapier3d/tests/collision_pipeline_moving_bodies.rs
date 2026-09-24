//! `CollisionPipeline::step` with non-fixed bodies: the pipeline doesn't register bodies in
//! the island manager, which used to trip the island-membership debug assertion as soon as a
//! kinematic or dynamic body started touching another collider.

use rapier3d::prelude::*;

struct CollisionWorld {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    pipeline: CollisionPipeline,
}

impl CollisionWorld {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            pipeline: CollisionPipeline::new(),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            IntegrationParameters::default().prediction_distance(),
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &(),
            &(),
        );
    }

    fn touching(&self, co1: ColliderHandle, co2: ColliderHandle) -> bool {
        self.narrow_phase
            .contact_pair(co1, co2)
            .is_some_and(|pair| pair.has_any_active_contact())
    }
}

fn check_moving_body_touches(body_type: RigidBodyType, other_type: Option<RigidBodyType>) {
    let mut world = CollisionWorld::new();
    // Kinematic-fixed and kinematic-kinematic contacts are disabled by default.
    let all_types = ActiveCollisionTypes::all();

    let other = match other_type {
        Some(ty) => {
            let body = world.bodies.insert(RigidBodyBuilder::new(ty));
            world.colliders.insert_with_parent(
                ColliderBuilder::cuboid(1.0, 1.0, 1.0).active_collision_types(all_types),
                body,
                &mut world.bodies,
            )
        }
        None => world
            .colliders
            .insert(ColliderBuilder::cuboid(1.0, 1.0, 1.0).active_collision_types(all_types)),
    };

    let body = world
        .bodies
        .insert(RigidBodyBuilder::new(body_type).translation(Vector::new(0.0, 5.0, 0.0)));
    let co = world.colliders.insert_with_parent(
        ColliderBuilder::ball(0.5).active_collision_types(all_types),
        body,
        &mut world.bodies,
    );

    world.step();
    assert!(!world.touching(co, other));

    // Move the body into the other collider: the begin-touch transition must not panic.
    world.bodies[body].set_translation(Vector::new(0.0, 1.2, 0.0), true);
    world.step();
    assert!(world.touching(co, other));
    world.step();
    assert!(world.touching(co, other));

    // And move it away again (end-touch transition).
    world.bodies[body].set_translation(Vector::new(0.0, 5.0, 0.0), true);
    world.step();
    assert!(!world.touching(co, other));

    // Removing the bodies with the pipeline's island manager must work too.
    world.bodies.remove(
        body,
        &mut world.islands,
        &mut world.colliders,
        &mut ImpulseJointSet::new(),
        &mut MultibodyJointSet::new(),
        &mut SoftBodySet::new(),
        true,
    );
    world.step();
}

#[test]
fn kinematic_body_touching_fixed_collider() {
    check_moving_body_touches(RigidBodyType::KinematicPositionBased, None);
    check_moving_body_touches(
        RigidBodyType::KinematicPositionBased,
        Some(RigidBodyType::Fixed),
    );
    check_moving_body_touches(RigidBodyType::KinematicVelocityBased, None);
}

#[test]
fn dynamic_bodies_touching() {
    check_moving_body_touches(RigidBodyType::Dynamic, Some(RigidBodyType::Dynamic));
    check_moving_body_touches(RigidBodyType::Dynamic, None);
    check_moving_body_touches(
        RigidBodyType::KinematicPositionBased,
        Some(RigidBodyType::Dynamic),
    );
}
