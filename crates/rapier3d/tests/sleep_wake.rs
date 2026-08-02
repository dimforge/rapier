use rapier3d::prelude::*;

/// A body woken by an impulse (colliders barely moved) takes the contact-recycling
/// path, which must repair the per-pair solver hint count-cleared at sleep time —
/// otherwise the woken body falls through the ground (contacts never reach the solver).
#[test]
fn woken_body_is_supported_by_recycled_contacts() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();
    let gravity = Vector::Y * -9.81;

    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
        ground,
        &mut bodies,
    );

    let cube = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.6, 0.0)));
    colliders.insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), cube, &mut bodies);

    let step = |bodies: &mut RigidBodySet,
                colliders: &mut ColliderSet,
                islands: &mut IslandManager,
                bf: &mut BroadPhaseBvh,
                nf: &mut NarrowPhase,
                impulse_joints: &mut ImpulseJointSet,
                multibody_joints: &mut MultibodyJointSet,
                ccd: &mut CCDSolver,
                pipeline: &mut PhysicsPipeline| {
        pipeline.step(
            gravity,
            &params,
            islands,
            bf,
            nf,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            ccd,
            &(),
            &(),
        );
    };

    // Let the cube settle and fall asleep.
    let mut slept = false;
    for _ in 0..400 {
        step(
            &mut bodies,
            &mut colliders,
            &mut islands,
            &mut bf,
            &mut nf,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &mut pipeline,
        );
        if bodies[cube].is_sleeping() {
            slept = true;
            break;
        }
    }
    assert!(slept, "the cube never fell asleep");

    // Wake it with an impulse only: its colliders don't move, so the contact pair
    // must go through the recycling path (which repairs the count-cleared hint).
    bodies[cube].apply_impulse(Vector::new(0.5, 0.0, 0.0), true);
    assert!(!bodies[cube].is_sleeping());

    for _ in 0..120 {
        step(
            &mut bodies,
            &mut colliders,
            &mut islands,
            &mut bf,
            &mut nf,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &mut pipeline,
        );
        let y = bodies[cube].translation().y;
        assert!(
            y > 0.4,
            "woken cube sank into the ground (y = {y}): its contacts were not solved"
        );
    }
}

/// Minimal world harness for the partial-island sleep tests below.
struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    pipeline: PhysicsPipeline,
    bf: BroadPhaseBvh,
    nf: NarrowPhase,
    islands: IslandManager,
    ccd: CCDSolver,
    params: IntegrationParameters,
}

impl World {
    fn new() -> Self {
        let mut world = Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            pipeline: PhysicsPipeline::new(),
            bf: BroadPhaseBvh::new(),
            nf: NarrowPhase::new(),
            islands: IslandManager::new(),
            ccd: CCDSolver::new(),
            params: IntegrationParameters::default(),
        };
        let ground = world
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
        world.colliders.insert_with_parent(
            ColliderBuilder::cuboid(50.0, 0.5, 50.0),
            ground,
            &mut world.bodies,
        );
        world
    }

    fn add_cube(&mut self, builder: RigidBodyBuilder) -> RigidBodyHandle {
        let handle = self.bodies.insert(builder);
        self.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
            handle,
            &mut self.bodies,
        );
        handle
    }

    fn step(&mut self) {
        self.pipeline.step(
            Vector::Y * -9.81,
            &self.params,
            &mut self.islands,
            &mut self.bf,
            &mut self.nf,
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

/// Whole-island sleep (partial-island sleep was removed): a row
/// of touching cubes ending at a never-sleeping body is one component, so nothing
/// in it may sleep; a distant independent cube must still sleep on its own.
#[test]
fn non_sleeping_neighbor_keeps_touching_row_awake() {
    let mut w = World::new();

    // A row of touching cubes: c[0] can never sleep.
    let n = 8;
    let mut row = Vec::new();
    for i in 0..n {
        let builder = RigidBodyBuilder::dynamic()
            .translation(Vector::new(i as f32, 0.5, 0.0))
            .can_sleep(i != 0);
        row.push(w.add_cube(builder));
    }
    // A distant cube, disconnected from the row: its own island must sleep.
    let lone = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(30.0, 0.5, 0.0)));

    for _ in 0..400 {
        w.step();
    }

    for (i, handle) in row.iter().enumerate() {
        assert!(
            !w.bodies[*handle].is_sleeping(),
            "row cube {i} must stay awake: its island contains a non-sleeping body"
        );
    }
    assert!(
        w.bodies[lone].is_sleeping(),
        "the disconnected cube must sleep on its own"
    );

    // Everybody is still resting in place.
    for (i, handle) in row.iter().enumerate() {
        let pos = w.bodies[*handle].translation();
        assert!(
            (pos.y - 0.5).abs() < 0.1 && (pos.x - i as f32).abs() < 0.1,
            "row cube {i} drifted to {pos:?}"
        );
    }
}

/// An impact on a sleeping region must wake it and be resolved physically.
#[test]
fn impact_wakes_sleeping_region() {
    let mut w = World::new();

    let n = 6;
    let mut row = Vec::new();
    for i in 0..n {
        row.push(
            w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(i as f32, 0.5, 0.0))),
        );
    }

    for _ in 0..400 {
        w.step();
    }
    for (i, handle) in row.iter().enumerate() {
        assert!(
            w.bodies[*handle].is_sleeping(),
            "row cube {i} should be asleep before the impact"
        );
    }

    // Throw a fast cube at the end of the row.
    let bullet = w.add_cube(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.5, 0.0))
            .linvel(Vector::new(20.0, 0.0, 0.0)),
    );

    for _ in 0..30 {
        w.step();
    }

    assert!(
        !w.bodies[row[0]].is_sleeping(),
        "the impacted cube must be awake"
    );
    assert!(
        w.bodies[row[0]].linvel().length() > 0.05
            || (w.bodies[row[0]].translation().x - 0.0) > 0.05,
        "the impacted cube must have physically responded to the hit"
    );
    let _ = bullet;
}

/// A joint must never connect an awake body to a sleeping one: if one side of a
/// jointed pair is kept awake (halo of a non-sleeping body), the other side must
/// stay awake with it even if all its own contacts would allow sleeping.
#[test]
fn joint_keeps_both_sides_awake() {
    let mut w = World::new();

    // `mover` can never sleep; `b` touches it (halo); `a` is jointed to `b` but
    // physically separate from everything else; `control` is a free cube that
    // must sleep normally.
    let mover = w.add_cube(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .can_sleep(false),
    );
    let b = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 0.5, 0.0)));
    let a = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(2.5, 0.5, 0.0)));
    let control = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(10.0, 0.5, 0.0)));

    // Fixed joint holding `a` and `b` exactly at their rest poses (no residual
    // force at equilibrium).
    let joint = FixedJointBuilder::new()
        .local_anchor1(Vector::new(1.5, 0.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0, 0.0));
    w.impulse_joints.insert(b, a, joint, true);

    for _ in 0..400 {
        w.step();
    }

    assert!(!w.bodies[mover].is_sleeping());
    assert!(
        !w.bodies[b].is_sleeping(),
        "halo neighbor of the non-sleeping body must stay awake"
    );
    assert!(
        !w.bodies[a].is_sleeping(),
        "a body jointed to an awake body must not sleep"
    );
    assert!(
        w.bodies[control].is_sleeping(),
        "the control cube should sleep normally"
    );

    // The jointed pair must be at rest at its original poses (the joint holds
    // them without fighting).
    assert!((w.bodies[a].translation().x - 2.5).abs() < 0.1);
    assert!((w.bodies[b].translation().x - 1.0).abs() < 0.1);
}

/// Corner-velocity sleep metric: a long beam pivoting below the raw angular
/// threshold still moves its tips fast — it must NOT sleep mid-motion; a small
/// body spinning slightly above it barely moves its surface and SHOULD sleep.
#[test]
fn corner_velocity_sleep_metric() {
    let mut w = World::new();

    // Gravity-free spinners so their velocity stays exactly constant.
    let beam = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0, 0.0))
            .angvel(Vector::new(0.0, 0.0, 0.3)) // below the raw 0.5 rad/s threshold
            .gravity_scale(0.0),
    );
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(10.0, 0.1, 0.1), beam, &mut w.bodies);

    let pebble = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0, 20.0))
            // Above the raw 0.5 rad/s angular threshold, but the surface only
            // moves at ~0.55 × 0.087 ≈ 0.048 m/s — below the 0.05 linear one.
            .angvel(Vector::new(0.0, 0.0, 0.55))
            .gravity_scale(0.0),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.05, 0.05, 0.05),
        pebble,
        &mut w.bodies,
    );

    for _ in 0..200 {
        w.step();
    }

    assert!(
        !w.bodies[beam].is_sleeping(),
        "a slowly pivoting long beam (tips at ~3 m/s) must not sleep mid-motion"
    );
    assert!(
        w.bodies[pebble].is_sleeping(),
        "a small spinner whose surface moves at ~0.05 m/s should be allowed to sleep"
    );
}

/// Frontier drift monitor: a support dragged out at 0.15 m/s (below every velocity
/// wake gate) from under a rider that fell asleep mid-slide must wake it via the
/// relative-pose drift anchor, so it keeps riding instead of being left floating.
#[test]
fn sliding_support_wakes_sleeping_rider() {
    let mut w = World::new();

    // Kinematic pusher, slow enough to stay below every velocity wake gate.
    let pusher = w.bodies.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(-1.55, 0.5, 0.0))
            .linvel(Vector::new(0.15, 0.0, 0.0)),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        pusher,
        &mut w.bodies,
    );

    let support = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));
    let rider = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)));

    let mut rider_slept = false;
    let mut rider_woke_after_sleep = false;
    for _ in 0..400 {
        w.step();
        if w.bodies[rider].is_sleeping() {
            rider_slept = true;
        } else if rider_slept {
            rider_woke_after_sleep = true;
        }
    }

    assert!(
        rider_slept,
        "the rider should fall asleep mid-slide (its velocity is below the sleep threshold)"
    );
    assert!(
        rider_woke_after_sleep,
        "the frontier drift monitor must wake the frozen rider as its support slides away"
    );
    let rider_pos = w.bodies[rider].translation();
    let support_pos = w.bodies[support].translation();
    assert!(
        rider_pos.x > 0.3,
        "the rider must keep being dragged along overall (x = {})",
        rider_pos.x
    );
    assert!(
        (rider_pos.y - 1.5).abs() < 0.2 && (rider_pos.x - support_pos.x).abs() < 0.75,
        "the rider must still ride its support (rider {rider_pos:?}, support {support_pos:?})"
    );
}

/// A slow kinematic body pressing into a sleeping body must wake it instead of
/// tunneling through the frozen "static wall" (contact starts wake on any
/// visible approach; the drift anchor catches even sub-gate intrusions).
#[test]
fn slow_kinematic_wakes_sleeping_body_on_contact() {
    let mut w = World::new();

    let cube = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));

    // Approaches at 0.3 m/s: below the 2x-threshold wake gate (0.8 m/s), so
    // the contact start doesn't wake the cube — the frontier drift anchor
    // stamped on the new pair must, as the wall keeps pressing in.
    let wall = w.bodies.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(-2.5, 0.5, 0.0))
            .linvel(Vector::new(0.3, 0.0, 0.0)),
    );
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), wall, &mut w.bodies);

    // The cube falls asleep long before the wall arrives (~5s away).
    for _ in 0..120 {
        w.step();
    }
    assert!(w.bodies[cube].is_sleeping());

    // Step until well past first contact (the drift anchor needs the wall to
    // actually intrude by a fraction of the pair extent before it fires).
    for _ in 0..320 {
        w.step();
    }

    assert!(
        !w.bodies[cube].is_sleeping(),
        "the slow kinematic wall must wake the sleeping cube on contact"
    );
    assert!(
        w.bodies[cube].translation().x > 0.2,
        "the cube must have been pushed, not tunneled into (x = {})",
        w.bodies[cube].translation().x
    );
}

/// Drum regression: the frontier drift threshold must scale with the *sleeping*
/// body's size, not the pair's — a huge slow platform (rotating-drum wall) must
/// wake a small sleeping cube after sub-cube-size motion, not platform-sized.
#[test]
fn huge_slow_platform_wakes_small_sleeping_body() {
    let mut w = World::new();

    let cube = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));

    // Huge collider sliding in at 0.3 m/s: below the 2x wake gate, so only the
    // drift anchor can wake the cube — a pair-extent-scaled threshold would need
    // ~1.4 units of travel (~4.7s) after contact instead of ~0.09.
    let platform = w.bodies.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(-11.0, 0.5, 0.0))
            .linvel(Vector::new(0.3, 0.0, 0.0)),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.4, 10.0),
        platform,
        &mut w.bodies,
    );

    // The cube sleeps long before the platform arrives (gap 0.5 => ~100 steps).
    for _ in 0..90 {
        w.step();
    }
    assert!(w.bodies[cube].is_sleeping());

    // First contact at ~step 100; with the sleeping-body-scaled threshold the
    // drift anchor fires after ~0.09 units of intrusion (~18 steps).
    for _ in 0..60 {
        w.step();
    }

    assert!(
        !w.bodies[cube].is_sleeping(),
        "the huge slow platform must wake the small sleeping cube shortly after contact"
    );
    assert!(
        w.bodies[cube].translation().x > 0.02,
        "the cube must have been pushed by the platform (x = {})",
        w.bodies[cube].translation().x
    );
}

/// Frontier load-band wake regression (domino-demo stall): the toppling chain is a
/// quasi-static leaning wedge below every velocity gate that accrues no relative-pose
/// drift (drift anchor blind); its sustained normal impulse must wake the sleeper.
#[test]
fn slow_dynamic_intruder_wakes_sleeping_body() {
    let mut w = World::new();

    // A chain of tall dominoes, the first one leaning enough to topple. The
    // standing ones sleep after ~1s (they are isolated islands), long before
    // the slow leaning wave reaches them.
    let n = 10;
    let mut dominoes = Vec::new();
    for i in 0..n {
        let rot = if i == 0 {
            Rotation::from_rotation_z(-0.2)
        } else {
            Rotation::IDENTITY
        };
        let handle = w.bodies.insert(
            RigidBodyBuilder::dynamic()
                .pose(Pose::from_parts(Vector::new(i as f32 * 0.4, 2.0, 0.0), rot)),
        );
        w.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.1, 2.0, 1.0),
            handle,
            &mut w.bodies,
        );
        dominoes.push(handle);
    }

    let tilt = |bodies: &RigidBodySet, h: RigidBodyHandle| {
        let up = bodies[h].rotation() * Vector::Y;
        up.y.clamp(-1.0, 1.0).acos()
    };

    // ~15 simulated seconds: enough for the (slow) wave to cross the chain.
    for _ in 0..900 {
        w.step();
    }

    for (i, handle) in dominoes.iter().enumerate() {
        let t = tilt(&w.bodies, *handle);
        assert!(
            t > 0.5,
            "domino {i} did not fall (tilt = {t}): the chain stalled against a sleeping domino"
        );
    }

    // And the fallen chain must still be able to fall asleep afterwards.
    for _ in 0..600 {
        w.step();
    }
    for (i, handle) in dominoes.iter().enumerate() {
        assert!(
            w.bodies[*handle].is_sleeping(),
            "fallen domino {i} never went back to sleep"
        );
    }
}

/// Domino-spiral variant of the load-band wake: the wedge tip already touches
/// (carrying ~no load) when the standing dominoes fall asleep, so the sleep-time
/// monitor must stay load-armed (unloaded reference) or the ring never falls.
#[test]
fn grazing_wedge_wakes_sleeping_chain() {
    let mut w = World::new();

    // One ring segment of the domino-spiral demo (the region around its first
    // pre-tilted "starter"), reproduced verbatim: dominoes 150..=186 of the
    // spiral, the last one tilted so it leans backward onto the segment.
    let mut curr_angle = 0.0f32;
    let mut curr_rad = 10.0f32;
    let mut dominoes = Vec::new();
    for i in 0..187 {
        let perimeter = 2.0 * std::f32::consts::PI * curr_rad;
        let spacing = 0.4;
        let prev_angle = curr_angle;
        curr_angle += 2.0 * std::f32::consts::PI * spacing / perimeter;
        let (x, z) = curr_angle.sin_cos();
        let two_pi = 2.0 * std::f32::consts::PI;
        let nudged = curr_angle % two_pi < prev_angle % two_pi;
        let tilt = if nudged { 0.2 } else { 0.0 };
        if i >= 150 {
            let rot = Rotation::from_rotation_y(curr_angle);
            let tilt_axis = rot * Vector::Z;
            let tilt_rot = Rotation::from_axis_angle(tilt_axis, tilt);
            let handle = w
                .bodies
                .insert(RigidBodyBuilder::dynamic().pose(Pose::from_parts(
                    Vector::new(x * curr_rad, 2.1, z * curr_rad),
                    tilt_rot * rot,
                )));
            w.colliders.insert_with_parent(
                ColliderBuilder::cuboid(0.1, 2.0, 1.0),
                handle,
                &mut w.bodies,
            );
            dominoes.push(handle);
        }
        curr_rad += 1.5 / perimeter;
    }

    // ~30 simulated seconds: enough for the backward wave to cross the segment.
    for _ in 0..1800 {
        w.step();
    }

    for (i, handle) in dominoes.iter().enumerate() {
        let up = w.bodies[*handle].rotation() * Vector::Y;
        let tilt = up.y.clamp(-1.0, 1.0).acos();
        assert!(
            tilt > 0.5,
            "segment domino {i} did not fall (tilt = {tilt}): the wedge froze against a sleeping domino"
        );
    }
}

/// Closing-velocity wake: an impact below the absolute wake gate must still wake
/// the sleeper *in the contact step* and transfer momentum — otherwise the sleeping
/// side is an infinite-mass wall and the absorbed momentum is destroyed for good.
#[test]
fn sub_gate_impact_wakes_and_transfers_momentum() {
    let mut w = World::new();

    // Frictionless slider setup so the approach speed is controlled exactly.
    let ground_friction0 = w
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 70.0)));
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 10.0).friction(0.0),
        ground_friction0,
        &mut w.bodies,
    );

    let target = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 70.0)));
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).friction(0.0),
        target,
        &mut w.bodies,
    );
    for _ in 0..120 {
        w.step();
    }
    assert!(w.bodies[target].is_sleeping());

    // 0.6 is below the absolute contact-start gate (2x the 0.4 threshold) but
    // well above the closing-velocity gate (0.25x).
    let mover = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.5, 70.0))
            .linvel(Vector::new(0.6, 0.0, 0.0)),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).friction(0.0),
        mover,
        &mut w.bodies,
    );

    // 2.0 of gap at 0.6/s: contact at ~t = 3.33s; run to 4s.
    for _ in 0..240 {
        w.step();
    }

    let target_v = w.bodies[target].linvel().x;
    assert!(
        !w.bodies[target].is_sleeping(),
        "the sub-gate impact never woke the sleeping target"
    );
    assert!(
        target_v > 0.25,
        "the impact was partially absorbed by the sleeping target \
         (target vx = {target_v}, expected ~0.3 as in the awake-vs-awake case)"
    );
}

/// Anchoring rule: an eligible region whose only support is an awake body must NOT
/// sleep partially (frozen mid-air above a live support, it destabilizes on wake);
/// it needs a fixed/sleeping anchor, or its whole component must sleep at once.
#[test]
fn floating_region_does_not_sleep_partially() {
    let mut w = World::new();

    // Column: never-sleeping cube with three stacked on top. `b` is the awake
    // halo; `c`/`d` form an eligible region touching only the halo — it must
    // stay awake instead of freezing mid-air above a live support.
    let mover = w.add_cube(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .can_sleep(false),
    );
    let b = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)));
    let c = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.5, 0.0)));
    let d = w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.5, 0.0)));

    // Control column: fully sleepable, anchored by contact with the fixed
    // ground, so partial-island sleep still applies to it normally.
    let control: Vec<_> = (0..3)
        .map(|i| {
            w.add_cube(RigidBodyBuilder::dynamic().translation(Vector::new(
                10.0,
                0.5 + i as f32,
                0.0,
            )))
        })
        .collect();

    for _ in 0..600 {
        w.step();
    }

    assert!(!w.bodies[mover].is_sleeping());
    for (name, handle) in [("b", b), ("c", c), ("d", d)] {
        assert!(
            !w.bodies[handle].is_sleeping(),
            "cube {name} floats above a live support and must not sleep"
        );
    }
    for (i, handle) in control.iter().enumerate() {
        assert!(
            w.bodies[*handle].is_sleeping(),
            "grounded control cube {i} should be asleep"
        );
    }

    // The awake stack must still be resting in place.
    for (i, handle) in [mover, b, c, d].iter().enumerate() {
        let pos = w.bodies[*handle].translation();
        let expected_y = 0.5 + i as f32;
        assert!(
            (pos.y - expected_y).abs() < 0.1 && pos.x.abs() < 0.1,
            "stacked cube {i} drifted to {pos:?}"
        );
    }
}

/// The whole-island exception to the anchoring rule: an island with no fixed
/// anchor at all (free-floating debris) must still be able to sleep — as a
/// whole. This is the common "settled debris in space" case.
#[test]
fn whole_floating_island_sleeps() {
    let mut w = World::new();

    // Two barely-overlapping gravity-free cubes high in the air: one island,
    // touching nothing fixed and nothing sleeping.
    let a = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0, 0.0))
            .gravity_scale(0.0),
    );
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), a, &mut w.bodies);
    let b = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.999, 30.0, 0.0))
            .gravity_scale(0.0),
    );
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), b, &mut w.bodies);

    for _ in 0..400 {
        w.step();
    }

    assert!(
        w.bodies[a].is_sleeping() && w.bodies[b].is_sleeping(),
        "a fully floating island must still sleep as a whole"
    );
}

/// Sleep metric: a body drifting at 0.3 length-units/s is *moving* and must
/// never sleep mid-motion (the old 0.4 default threshold slept it; the default
/// is now 0.05); a body creeping at 0.03 — below the threshold — may sleep.
#[test]
fn slow_drift_does_not_sleep_mid_motion() {
    let mut w = World::new();

    let drifter = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0, 0.0))
            .linvel(Vector::new(0.3, 0.0, 0.0))
            .gravity_scale(0.0),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        drifter,
        &mut w.bodies,
    );

    let creeper = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 30.0, 20.0))
            .linvel(Vector::new(0.03, 0.0, 0.0))
            .gravity_scale(0.0),
    );
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        creeper,
        &mut w.bodies,
    );

    for _ in 0..400 {
        w.step();
    }

    assert!(
        !w.bodies[drifter].is_sleeping(),
        "a body moving at 0.3 length-units/s must not sleep mid-motion"
    );
    assert!(
        w.bodies[creeper].is_sleeping(),
        "a body creeping below the sleep threshold may sleep"
    );
}
