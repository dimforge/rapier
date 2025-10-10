# Rapier Physics Engine - Complete Codebase Guide

## What is Rapier?

Rapier is a high-performance 2D and 3D physics engine written in Rust by Dimforge. It's designed for games, animation, and robotics applications, offering deterministic simulations, snapshot/restore capabilities, and cross-platform support (including WASM).

**License:** Apache 2.0 (free and open-source)
**Repository:** https://github.com/dimforge/rapier
**Documentation:** https://rapier.rs/docs/
**Crate:** https://crates.io/crates/rapier3d

## Key Features

- **Dual-dimensional support**: Both 2D (`rapier2d`) and 3D (`rapier3d`) with f32 and f64 precision variants
- **Deterministic simulation**: Identical results across different machines (IEEE 754-2008 compliant)
- **Snapshot & restore**: Complete physics state serialization
- **Cross-platform**: Desktop, mobile, web (WASM), consoles
- **Performance-focused**: SIMD optimizations, optional multi-threading, sleeping system
- **Flexible**: Can be used for full physics or just collision detection

## Repository Architecture

The repository uses an unusual structure to share code between 2D/3D versions:

```
rapier/
├── src/                    # Shared source code (2D/3D agnostic)
├── crates/                 # Concrete 2D/3D crate definitions
│   ├── rapier2d/          # 2D f32
│   ├── rapier3d/          # 3D f32
│   ├── rapier2d-f64/      # 2D f64
│   ├── rapier3d-f64/      # 3D f64
│   ├── rapier_testbed2d/  # 2D visual debugger
│   ├── rapier_testbed3d/  # 3D visual debugger
│   └── rapier3d-urdf/     # Robot model loader
├── examples2d/            # 2D demos
├── examples3d/            # 3D demos
├── benchmarks2d/          # 2D performance tests
├── benchmarks3d/          # 3D performance tests
└── src_testbed/           # Testbed source
```

## Core Modules Deep Dive

### 1. `src/dynamics/` - Physics Simulation

Handles movement, forces, and constraints.

#### RigidBody System

**`RigidBody`** (`rigid_body.rs`) - The fundamental physics object
- **Three body types:**
  - `Dynamic`: Fully simulated - responds to forces, gravity, collisions
  - `Fixed`: Never moves - infinite mass (walls, floors, terrain)
  - `Kinematic`: User-controlled movement - pushes but isn't pushed (platforms, doors)

**Key properties:**
- Position/rotation: `translation()`, `rotation()`, `position()`
- Velocity: `linvel()`, `angvel()`, `set_linvel()`, `set_angvel()`
- Mass: `mass()`, `center_of_mass()`, computed from attached colliders
- Forces: `add_force()`, `add_torque()`, `add_force_at_point()` (continuous, cleared each step)
- Impulses: `apply_impulse()`, `apply_torque_impulse()`, `apply_impulse_at_point()` (instant)
- Damping: `linear_damping()`, `angular_damping()` (air resistance, drag)
- Sleeping: `sleep()`, `wake_up()`, `is_sleeping()` (performance optimization)
- CCD: `enable_ccd()`, prevents fast objects tunneling through walls
- Locking: `lock_rotations()`, `lock_translations()`, `set_locked_axes()` (constrain movement)
- Gravity: `gravity_scale()`, `set_gravity_scale()` (0.0 = zero-g, 1.0 = normal, 2.0 = heavy)
- Energy: `kinetic_energy()`, `gravitational_potential_energy()`
- Prediction: `predict_position_using_velocity()`, `predict_position_using_velocity_and_forces()`

**`RigidBodySet`** (`rigid_body_set.rs`) - Collection of all bodies
- Handle-based storage (generational indices prevent use-after-free)
- Methods: `insert()`, `remove()`, `get()`, `get_mut()`, `iter()`, `contains()`
- `propagate_modified_body_positions_to_colliders()` for manual position sync

**`RigidBodyBuilder`** - Builder pattern for creating bodies
- Type constructors: `dynamic()`, `fixed()`, `kinematic_velocity_based()`, `kinematic_position_based()`
- Configuration: `translation()`, `rotation()`, `linvel()`, `angvel()`
- Settings: `gravity_scale()`, `can_sleep()`, `ccd_enabled()`, `linear_damping()`, `angular_damping()`
- Constraints: `locked_axes()`, `lock_rotations()`, `lock_translations()`, `enabled_rotations()`
- Advanced: `dominance_group()`, `additional_mass()`, `enable_gyroscopic_forces()`

#### Joint System

**Joint Types** (all in `src/dynamics/joint/`):

1. **`FixedJoint`**: Welds two bodies rigidly together
2. **`RevoluteJoint`**: Hinge - rotation around one axis (doors, wheels)
3. **`PrismaticJoint`**: Slider - translation along one axis (pistons, elevators)
4. **`SphericalJoint`**: Ball-and-socket - free rotation, fixed position (shoulders, gimbals)
5. **`RopeJoint`**: Maximum distance constraint (ropes, cables, tethers)
6. **`SpringJoint`**: Elastic connection with stiffness/damping (suspension, soft constraints)

**Joint Features:**
- **Motors**: Powered actuation with `set_motor_velocity()`, `set_motor_position()`
  - Velocity control: constant speed rotation/sliding
  - Position control: spring-like movement toward target
  - Max force limits: `set_motor_max_force()`
  - Motor models: `AccelerationBased` (mass-independent) vs `ForceBased` (mass-dependent)
- **Limits**: Restrict range with `set_limits([min, max])`
- **Anchors**: Connection points in each body's local space

**`ImpulseJointSet`** - Collection of all joints
- Methods: `insert()`, `remove()`, `get()`, `get_mut()`, `iter()`
- Queries: `joints_between()`, `attached_joints()`, `attached_enabled_joints()`

#### Integration & Solving

**`IntegrationParameters`** - Controls simulation behavior
- `dt`: Timestep (1/60 for 60 FPS, 1/120 for 120 FPS)
- `num_solver_iterations`: Accuracy vs speed (4 = default, 8-12 = high quality, 1-2 = fast)
- `length_unit`: Scale factor if not using meters (100.0 for pixel-based 2D)
- `contact_natural_frequency`, `contact_damping_ratio`: Contact compliance
- `joint_natural_frequency`, `joint_damping_ratio`: Joint compliance
- Advanced: warmstarting, prediction distance, stabilization iterations

**`IslandManager`** - Sleeping/waking optimization
- Groups connected bodies into "islands" for parallel solving
- Automatically sleeps bodies at rest (huge performance gain)
- Wakes bodies when disturbed by collisions/joints
- Sleep thresholds configurable via `RigidBodyActivation`

**`CCDSolver`** - Prevents tunneling
- Detects fast-moving bodies that might pass through geometry
- Predicts time-of-impact and clamps motion
- Enable per-body with `ccd_enabled(true)`
- Soft-CCD: cheaper predictive variant with `set_soft_ccd_prediction()`

### 2. `src/geometry/` - Collision Detection

#### Collider System

**`Collider`** - Collision shape ("hitbox") attached to bodies
- **Shapes**: Ball, Cuboid, Capsule, Cylinder, Cone, Triangle, Segment, HeightField, TriMesh, Compound, ConvexHull
- **Material properties:**
  - Friction: 0.0 = ice, 0.5 = wood, 1.0 = rubber
  - Restitution: 0.0 = clay (no bounce), 1.0 = perfect elastic, >1.0 = super bouncy
  - Combine rules: Average, Min, Max, Multiply (how to combine when two colliders touch)
- **Mass:** Set via `density()` (kg/m³) or `mass()` (kg directly)
- **Sensors:** `set_sensor(true)` for trigger zones (detect overlaps without physical collision)
- **Position:** `position()`, `translation()`, `rotation()`, `position_wrt_parent()`
- **Groups:** `collision_groups()`, `solver_groups()` for layer-based filtering
- **Events:** `active_events()`, `contact_force_event_threshold()` for collision notifications
- **Shape queries:** `compute_aabb()`, `compute_swept_aabb()`, `volume()`, `mass_properties()`
- **Enabled:** `set_enabled()` to temporarily disable without removal

**`ColliderSet`** - Collection of all colliders
- Methods: `insert()`, `insert_with_parent()`, `remove()`, `set_parent()`
- Access: `get()`, `get_mut()`, `iter()`, `iter_enabled()`
- Invalid handle: `ColliderSet::invalid_handle()`

**`ColliderBuilder`** - Creates colliders with builder pattern
- **Primitive shapes:**
  - `ball(radius)` - Sphere/circle
  - `cuboid(hx, hy, hz)` - Box (half-extents)
  - `capsule_y(half_height, radius)` - Pill shape (best for characters!)
  - `cylinder(half_height, radius)`, `cone(half_height, radius)` (3D only)
- **Complex shapes:**
  - `trimesh(vertices, indices)` - Triangle mesh (slow but accurate)
  - `heightfield(heights, scale)` - Terrain from height grid
  - `convex_hull(points)` - Smallest convex shape containing points
  - `convex_decomposition(vertices, indices)` - Breaks concave mesh into convex pieces
  - `segment(a, b)`, `triangle(a, b, c)` - Simple primitives
- **Configuration:**
  - Material: `friction()`, `restitution()`, `density()`, `mass()`
  - Filtering: `collision_groups()`, `sensor()`
  - Events: `active_events()`, `contact_force_event_threshold()`
  - Position: `translation()`, `rotation()`, `position()`
  - Advanced: `active_hooks()`, `active_collision_types()`, `contact_skin()`

#### Collision Detection Pipeline

**`BroadPhaseBvh`** - First pass: quickly find nearby pairs
- Uses hierarchical BVH tree for spatial indexing
- Filters out distant objects before expensive narrow-phase
- Incrementally updated as objects move
- Optimization strategies: `SubtreeOptimizer` (default) vs `None`

**`NarrowPhase`** - Second pass: compute exact contacts
- Calculates precise contact points, normals, penetration depths
- Builds contact manifolds (groups of contacts sharing properties)
- Managed automatically by PhysicsPipeline
- Access via `contact_graph()` for querying contact pairs

**`ContactPair`** - Detailed collision information
- Methods: `total_impulse()`, `total_impulse_magnitude()`, `max_impulse()`, `find_deepest_contact()`
- Contains multiple `ContactManifold` structures
- Each manifold has contact points with normals, distances, solver data

#### Collision Filtering

**`InteractionGroups`** - Layer-based collision control
- Two components: `memberships` (what groups I'm in) and `filter` (what groups I collide with)
- 32 groups available: `Group::GROUP_1` through `Group::GROUP_32`
- Bidirectional check: A and B collide only if A's memberships overlap B's filter AND vice versa
- Example: Player bullets (group 1) only hit enemies (group 2)

**`ActiveCollisionTypes`** - Filter by body type
- Controls which body type pairs can collide
- Defaults: Dynamic↔Dynamic ✓, Dynamic↔Fixed ✓, Dynamic↔Kinematic ✓, Fixed↔Fixed ✗
- Rarely changed - defaults are correct for most games

**`QueryFilter`** - Filter spatial queries
- Flags: `EXCLUDE_FIXED`, `EXCLUDE_DYNAMIC`, `EXCLUDE_SENSORS`, `ONLY_DYNAMIC`, etc.
- Groups: Filter by collision groups
- Exclusions: `exclude_collider`, `exclude_rigid_body`
- Custom: `predicate` closure for arbitrary filtering

### 3. `src/pipeline/` - Simulation Orchestration

**`PhysicsPipeline`** - The main simulation loop
- **`step()`**: Advances physics by one timestep
  1. Handles user changes (moved bodies, added/removed colliders)
  2. Runs collision detection (broad-phase → narrow-phase)
  3. Builds islands and solves constraints (contacts + joints)
  4. Integrates velocities to update positions
  5. Runs CCD if needed
  6. Generates events
- Reuse same instance for performance (caches data between frames)

**`CollisionPipeline`** - Collision detection without dynamics
- Use when you only need collision detection, not physics simulation
- Lighter weight than PhysicsPipeline

**`QueryPipeline`** - Spatial queries (created from BroadPhase)
- **Raycasting:**
  - `cast_ray()` - First hit along ray
  - `cast_ray_and_get_normal()` - Hit with surface normal
  - `intersect_ray()` - ALL hits along ray
- **Shape casting:**
  - `cast_shape()` - Sweep shape through space (thick raycast)
  - `cast_shape_nonlinear()` - Non-linear motion sweep
- **Point queries:**
  - `project_point()` - Find closest point on any collider
  - `intersect_point()` - Find all colliders containing point
- **AABB queries:**
  - `intersect_aabb_conservative()` - Find colliders in bounding box
- Filtering via `QueryFilter`

**`EventHandler`** trait - Receive physics events
- `handle_collision_event()` - When colliders start/stop touching
- `handle_contact_force_event()` - When contact forces exceed threshold
- Built-in: `ChannelEventCollector` sends events to mpsc channels
- Enable per-collider with `ActiveEvents` flags

**`PhysicsHooks`** trait - Custom collision behavior
- `filter_contact_pair()` - Decide if two colliders should collide
- `filter_intersection_pair()` - Filter sensor intersections
- `modify_solver_contacts()` - Modify contact properties before solving
- Enable per-collider with `ActiveHooks` flags
- Advanced feature - most users should use `InteractionGroups` instead

**`CollisionEvent`** - Collision state changes
- `Started(h1, h2, flags)` - Colliders began touching
- `Stopped(h1, h2, flags)` - Colliders stopped touching
- Methods: `started()`, `stopped()`, `collider1()`, `collider2()`, `sensor()`

### 4. `src/control/` - High-Level Controllers

**`KinematicCharacterController`** - Player/NPC movement
- Handles walking, slopes, stairs, wall sliding, ground snapping
- NOT physics-based - you control movement directly
- Features:
  - `slide`: Slide along walls instead of stopping
  - `autostep`: Automatically climb stairs/small obstacles
  - `max_slope_climb_angle`: Max climbable slope (radians)
  - `min_slope_slide_angle`: When to slide down slopes
  - `snap_to_ground`: Keep grounded on uneven terrain
- Returns `EffectiveCharacterMovement` with `translation` and `grounded` status

**`DynamicRayCastVehicleController`** - Arcade vehicle physics
- Raycast-based suspension (simpler than constraint-based)
- Add wheels with suspension, steering, engine force
- Automatically handles wheel contacts and forces

### 5. `src/data/` - Core Data Structures

**`Arena<T>`** - Handle-based storage
- Generational indices: (index, generation) pair
- Prevents use-after-free: old handles become invalid when slots reused
- Used for: `RigidBodySet`, `ColliderSet`, `ImpulseJointSet`
- Methods: `insert()`, `remove()`, `get()`, `get_mut()`, `iter()`
- Advanced: `get_unknown_gen()` bypasses generation check (unsafe)

**`Graph<N, E>`** - Generic graph structure
- Nodes and edges with associated data
- Used for: interaction graph (bodies/colliders), joint graph
- Enables: "find all joints attached to this body"

**`ModifiedObjects`** - Change tracking
- Flags objects that changed since last frame
- Enables incremental updates (only process changed objects)
- Critical for performance

### 6. `src/counters/` - Profiling

**`Counters`** - Performance measurements
- Tracks time in: collision detection, solver, CCD, island construction
- Subdivided: broad-phase time, narrow-phase time, velocity resolution, etc.
- Access via `physics_pipeline.counters`

## Complete API Reference

### Body Types

```rust
RigidBodyType::Dynamic          // Fully simulated
RigidBodyType::Fixed            // Never moves
RigidBodyType::KinematicVelocityBased    // Velocity control
RigidBodyType::KinematicPositionBased    // Position control
```

### Joint Types

```rust
FixedJoint::new()               // Weld
RevoluteJoint::new(axis)        // Hinge
PrismaticJoint::new(axis)       // Slider
SphericalJoint::new()           // Ball-and-socket (3D only)
RopeJoint::new(max_dist)        // Distance limit
SpringJoint::new(rest, stiff, damp)  // Elastic
```

### Collision Shapes

```rust
// Primitives (fast)
ColliderBuilder::ball(radius)
ColliderBuilder::cuboid(hx, hy, hz)         // Half-extents
ColliderBuilder::capsule_y(half_h, r)       // Best for characters!
ColliderBuilder::cylinder(half_h, r)        // 3D only
ColliderBuilder::cone(half_h, r)            // 3D only

// Complex (slower)
ColliderBuilder::trimesh(verts, indices)    // Arbitrary mesh
ColliderBuilder::heightfield(heights, scale) // Terrain
ColliderBuilder::convex_hull(points)        // Convex wrap
ColliderBuilder::convex_decomposition(v, i) // Auto-split concave
```

### Events & Hooks

```rust
// Event flags
ActiveEvents::COLLISION_EVENTS           // Start/stop touching
ActiveEvents::CONTACT_FORCE_EVENTS       // Force threshold exceeded

// Hook flags
ActiveHooks::FILTER_CONTACT_PAIRS        // Custom collision filtering
ActiveHooks::FILTER_INTERSECTION_PAIR    // Custom sensor filtering
ActiveHooks::MODIFY_SOLVER_CONTACTS      // Modify contact properties
```

### Filtering

```rust
// Collision groups (layer system)
let groups = InteractionGroups::new(
    Group::GROUP_1,                      // I'm in group 1
    Group::GROUP_2 | Group::GROUP_3      // I collide with 2 and 3
);

// Query filters
QueryFilter::default()
QueryFilter::only_dynamic()              // Ignore static geometry
QueryFilter::exclude_sensors()           // Only solid shapes
```

### Locked Axes

```rust
LockedAxes::ROTATION_LOCKED              // Can't rotate at all
LockedAxes::TRANSLATION_LOCKED           // Can't translate at all
LockedAxes::TRANSLATION_LOCKED_Z         // Lock one axis (2D in 3D)
LockedAxes::ROTATION_LOCKED_X | LockedAxes::ROTATION_LOCKED_Y  // Combine
```

## Advanced Concepts

### Sleeping System

Bodies automatically sleep when at rest (velocities below threshold for 2 seconds). Sleeping bodies:
- Skipped in collision detection and simulation
- Auto-wake when hit or joint-connected to moving body
- Configured via `RigidBodyActivation`:
  - `normalized_linear_threshold`: Linear velocity threshold (default 0.4)
  - `angular_threshold`: Angular velocity threshold (default 0.5)
  - `time_until_sleep`: How long to be still before sleeping (default 2.0s)
- Disable with `can_sleep(false)` or `RigidBodyActivation::cannot_sleep()`

### CCD (Continuous Collision Detection)

Prevents "tunneling" where fast objects pass through thin walls:
- **Hard CCD**: Shape-casting with substeps (expensive but accurate)
- **Soft CCD**: Predictive contacts (cheaper, good for medium-speed objects)
- Enable: `RigidBodyBuilder::ccd_enabled(true)` or `body.enable_ccd(true)`
- Soft: `set_soft_ccd_prediction(distance)`
- Active when velocity exceeds auto-computed threshold

### Mass Properties

Total mass = collider masses + additional mass:
- Collider mass: `density × volume` or set directly
- Additional mass: `set_additional_mass()` adds to total
- Auto-computed: mass, center of mass, angular inertia tensor
- Manual recompute: `recompute_mass_properties_from_colliders()`

### Dominance Groups

Bodies with higher dominance push lower ones but not vice versa:
- Range: `i8::MIN` to `i8::MAX`
- Default: 0 (all equal)
- Rarely needed - use for "heavy objects should always win" scenarios

### Contact Skin

Small margin around colliders (keeps objects slightly apart):
- Improves performance and stability
- Might create small visual gaps
- Set via `ColliderBuilder::contact_skin(thickness)`

### Motor Models

How motor spring constants scale with mass:
- **`MotorModel::AccelerationBased`** (default): Auto-scales with mass, easier to tune
- **`MotorModel::ForceBased`**: Absolute forces, mass-dependent behavior

## Common Usage Patterns

### Creating a Dynamic Object

```rust
let body = RigidBodyBuilder::dynamic()
    .translation(vector![0.0, 10.0, 0.0])
    .linvel(vector![1.0, 0.0, 0.0])
    .build();
let body_handle = bodies.insert(body);

let collider = ColliderBuilder::ball(0.5)
    .density(2700.0)  // Aluminum
    .friction(0.7)
    .restitution(0.3)
    .build();
colliders.insert_with_parent(collider, body_handle, &mut bodies);
```

### Raycasting

```rust
let query_pipeline = broad_phase.as_query_pipeline(
    &QueryDispatcher,
    &bodies,
    &colliders,
    QueryFilter::default()
);

let ray = Ray::new(point![0.0, 10.0, 0.0], vector![0.0, -1.0, 0.0]);
if let Some((handle, toi)) = query_pipeline.cast_ray(&ray, Real::MAX, true) {
    let hit_point = ray.origin + ray.dir * toi;
    println!("Hit {:?} at {:?}, distance = {}", handle, hit_point, toi);
}
```

### Applying Forces vs Impulses

```rust
// IMPULSE: Instant change (jumping, explosions)
body.apply_impulse(vector![0.0, 500.0, 0.0], true);  // Jump!

// FORCE: Continuous (thrust, wind) - call every frame
body.add_force(vector![0.0, 100.0, 0.0], true);  // Rocket thrust
```

### Creating a Joint

```rust
let joint = RevoluteJointBuilder::new()
    .local_anchor1(point![1.0, 0.0, 0.0])
    .local_anchor2(point![-1.0, 0.0, 0.0])
    .limits([0.0, std::f32::consts::PI / 2.0])  // 0-90° rotation
    .build();
let joint_handle = joints.insert(body1, body2, joint, true);
```

### Character Controller

```rust
let controller = KinematicCharacterController {
    slide: true,
    autostep: Some(CharacterAutostep::default()),
    max_slope_climb_angle: 45.0_f32.to_radians(),
    snap_to_ground: Some(CharacterLength::Relative(0.2)),
    ..Default::default()
};

let desired_movement = vector![input_x, 0.0, input_z] * speed * dt;
let movement = controller.move_shape(
    dt, &bodies, &colliders, &query_pipeline,
    character_shape, &character_pos, desired_movement,
    QueryFilter::default(), |_| {}
);
character_pos.translation.vector += movement.translation;
```

### Collision Events

```rust
use std::sync::mpsc::channel;

let (collision_send, collision_recv) = channel();
let (force_send, force_recv) = channel();
let event_handler = ChannelEventCollector::new(collision_send, force_send);

// In physics step
physics_pipeline.step(..., &event_handler);

// After physics
while let Ok(event) = collision_recv.try_recv() {
    match event {
        CollisionEvent::Started(h1, h2, _) => println!("Collision!"),
        CollisionEvent::Stopped(h1, h2, _) => println!("Separated"),
    }
}
```

## Performance Optimization Tips

1. **Sleeping**: Let objects at rest sleep (default behavior)
2. **Shape choice**: Ball/Cuboid/Capsule >> Convex Hull >> TriMesh
3. **Solver iterations**: Lower `num_solver_iterations` if accuracy isn't critical
4. **Parallel**: Enable `parallel` feature for multi-core
5. **Broadphase**: Keep objects reasonably distributed (not all in one spot)
6. **CCD**: Only enable for fast objects that need it
7. **Event generation**: Only enable events on colliders that need them
8. **Collision groups**: Filter unnecessary collision checks
9. **Fixed timestep**: Use fixed `dt`, accumulate remainder for smooth rendering

## Common Patterns & Best Practices

### Handle Storage
```rust
// Store handles, not references
struct Player {
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
}

// Access when needed
let player_body = &mut bodies[player.body_handle];
```

### 2D Game in 3D Engine
```rust
let body = RigidBodyBuilder::dynamic()
    .locked_axes(
        LockedAxes::TRANSLATION_LOCKED_Z |
        LockedAxes::ROTATION_LOCKED_X |
        LockedAxes::ROTATION_LOCKED_Y
    )
    .build();
```

### One-Way Platforms (via hooks)
```rust
struct OneWayPlatform;
impl PhysicsHooks for OneWayPlatform {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        // Allow contact only if player is above platform
        if player_above_platform(context) {
            Some(SolverFlags::COMPUTE_IMPULSES)
        } else {
            None  // No collision
        }
    }
}
```

### Compound Shapes
```rust
// Multiple colliders on one body
let body_handle = bodies.insert(RigidBodyBuilder::dynamic().build());

colliders.insert_with_parent(
    ColliderBuilder::cuboid(1.0, 1.0, 1.0).translation(vector![0.0, 1.0, 0.0]).build(),
    body_handle, &mut bodies
);
colliders.insert_with_parent(
    ColliderBuilder::ball(0.5).translation(vector![0.0, 3.0, 0.0]).build(),
    body_handle, &mut bodies
);
// Now the body has a box + ball shape
```

## Troubleshooting

### Objects Tunneling Through Walls
- Enable CCD: `body.enable_ccd(true)`
- Increase wall thickness
- Reduce timestep (`dt`)
- Increase `num_solver_iterations`

### Unstable Simulation (Jittering, Explosions)
- Reduce mass ratios (avoid 1kg vs 1000kg objects)
- Increase `num_solver_iterations`
- Check for conflicting constraints
- Verify joint anchors are reasonable
- Reduce timestep if using large `dt`

### Poor Performance
- Check sleeping is enabled (`can_sleep(true)`)
- Use simpler shapes (capsules instead of meshes)
- Enable `parallel` feature
- Reduce `num_solver_iterations` if acceptable
- Use collision groups to avoid unnecessary checks
- Only enable events on colliders that need them

### Bodies Stuck/Not Moving
- Check if sleeping: `body.wake_up(true)`
- Verify mass > 0 (check collider density)
- Check locked axes aren't preventing movement
- Verify gravity scale isn't 0

## File Statistics

- **95 total Rust files** in `src/`
- **Top files by public function count:**
  - `rigid_body.rs`: 126 functions
  - `collider.rs`: 118 functions
  - `rigid_body_components.rs`: 56 functions
  - `generic_joint.rs`: 47 functions
  - `query_pipeline.rs`: 29 functions

## Documentation Improvements

### Session 1: Comprehensive API Documentation
Comprehensively documented **300+ public functions** across **45+ files**:

**Fully Documented Modules:**
1. **PhysicsPipeline** - Main simulation loop
2. **RigidBody** (~60 methods) - All position, velocity, force, impulse, damping, CCD, locking methods
3. **RigidBodySet** - All collection management methods
4. **RigidBodyBuilder** (~40 methods) - All configuration methods
5. **Collider** (~50 methods) - All property accessors and setters
6. **ColliderSet** - All collection methods
7. **ColliderBuilder** (~60 methods) - All shape constructors and configuration
8. **All 6 joint types** - Comprehensive docs for Fixed, Revolute, Prismatic, Spherical, Rope, Spring
9. **ImpulseJointSet** - All joint collection methods
10. **QueryPipeline** - All spatial query methods
11. **EventHandler & events** - Complete event system
12. **InteractionGroups** - Collision filtering
13. **IntegrationParameters** - Simulation settings
14. **IslandManager** - Sleep/wake system
15. **CCDSolver** - Tunneling prevention
16. **BroadPhaseBvh, NarrowPhase** - Collision detection
17. **CharacterController** - Player movement
18. **ContactPair** - Contact information
19. **All major enums/flags**: RigidBodyType, LockedAxes, ActiveEvents, ActiveHooks, ActiveCollisionTypes, CoefficientCombineRule, MotorModel, CollisionEvent, QueryFilter

**Documentation Style:**
All functions include:
- **Plain language** ("hitbox" not "geometric entity")
- **Real-world use cases** (when/why to use)
- **Code examples** (copy-paste ready)
- **Value guides** (friction 0-1, density values for real materials)
- **Warnings** (teleporting, performance costs, common mistakes)
- **Comparisons** (forces vs impulses, mass vs density, when to use each)

### Session 2: Documentation Example Testing
**Converted 75+ ignored documentation examples to be tested by `cargo test --doc`**

**Goal:** Ensure all documentation examples remain valid and compilable as the codebase evolves.

**Files with Fixed Examples:**

*Dynamics Module (33 examples):*
- `dynamics/rigid_body.rs` (13)
- `dynamics/rigid_body_set.rs` (8)
- `dynamics/rigid_body_components.rs` (1) - LockedAxes
- `dynamics/coefficient_combine_rule.rs` (1)
- `dynamics/integration_parameters.rs` (1)
- `dynamics/island_manager.rs` (1)
- `dynamics/joint/rope_joint.rs` (1)
- `dynamics/joint/revolute_joint.rs` (1)
- `dynamics/joint/generic_joint.rs` (1) - JointMotor
- `dynamics/joint/impulse_joint/impulse_joint_set.rs` (5)

*Geometry Module (10 examples):*
- `geometry/interaction_groups.rs` (1)
- `geometry/collider_set.rs` (4)
- `geometry/collider_components.rs` (1) - ActiveCollisionTypes
- `geometry/contact_pair.rs` (2)
- `geometry/mod.rs` (1) - CollisionEvent
- `geometry/interaction_graph.rs` (1)

*Pipeline Module (14 examples):*
- `pipeline/query_pipeline.rs` (9) - Raycasting, shape casting, point queries
- `pipeline/event_handler.rs` (3) - ActiveEvents, EventHandler trait, ChannelEventCollector
- `pipeline/physics_pipeline.rs` (1)
- `pipeline/collision_pipeline.rs` (1)

*Control Module (1 example):*
- `control/character_controller.rs` (1) - Complete character controller setup

*Data Module (25 examples):*
- `data/arena.rs` (25) - All Arena API methods

*Other Modules (4 examples):*
- `dynamics/joint/multibody_joint/multibody_joint_set.rs` (1)

**Conversion Pattern:**
```rust
// Before:
/// ```ignore
/// let body = RigidBodyBuilder::dynamic().build();
/// bodies.insert(body);
/// ```

// After:
/// ```
/// # use rapier3d::prelude::*;
/// # let mut bodies = RigidBodySet::new();
/// let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
/// ```
```

Hidden lines (prefixed with `#`) provide setup code while keeping examples readable.

**Key Fixes Required for Compilation:**

1. **Removed unnecessary `.build()` calls**: Builders implement `Into<T>`, so:
   - `RigidBodyBuilder::dynamic().build()` → `RigidBodyBuilder::dynamic()`
   - `ColliderBuilder::ball(0.5).build()` → `ColliderBuilder::ball(0.5)`
   - These work directly with `insert()` and `insert_with_parent()`

2. **Fixed API calls to match actual implementation:**
   - `&QueryDispatcher` → `narrow_phase.query_dispatcher()` (QueryPipeline needs a dispatcher reference)
   - Added `NarrowPhase::new()` setup for query pipeline examples

3. **Corrected property/field names:**
   - `hit.toi` → `hit.time_of_impact` (RayIntersection struct)
   - `collider.shape()` → `collider.shared_shape()` (when printing/debugging)

4. **Added required setup for complex examples:**
   - `project_point()` example: Added `IntegrationParameters`, `broad_phase.set_aabb()` call
   - Character controller: Changed to `Ball::new(0.5)` instead of shape reference
   - Joint examples: Fixed to use `Vector::y_axis()` instead of implicit axis

5. **Fixed joint constructor calls:**
   - `RevoluteJoint::new()` → `RevoluteJoint::new(Vector::y_axis())` (axis required)
   - `PrismaticJoint::new(...)` → `PrismaticJoint::new(Vector::x_axis())` (axis required)

**Remaining Work:**
- `geometry/collider.rs` has 12 ignored examples still marked as `ignore` (these are intentionally left as `ignore` for documentation purposes where full compilation context would be overly verbose)

**Impact:**
- ✅ Documentation examples now compile with `cargo test --doc`
- ✅ Examples stay correct as codebase evolves (tests will catch API changes)
- ✅ Copy-paste ready code that actually works
- ✅ Improved documentation quality and developer experience
- ✅ Builders work seamlessly without explicit `.build()` calls

## Examples Directory

`examples3d/` contains many demonstrations:

- `primitives3.rs` - Showcase of basic shapes
- `keva3.rs` - Large tower of blocks (stress test)
- `platform3.rs` - Moving kinematic platforms
- `joints3.rs` - All joint types demonstrated
- `character_controller3.rs` - Character movement
- `vehicle_controller3.rs` - Vehicle physics
- `ccd3.rs` - Fast bullets with CCD
- `sensor3.rs` - Trigger zones
- `despawn3.rs` - Removing objects
- `debug_boxes3.rs` - Visual debugging
- `rotating_floor_stacks3.rs` - **Custom example**: 20 pyramids (10×10 cube bases) on slowly rotating floor

**Run**: `cargo run --release --bin all_examples3`

## Building & Testing

```bash
# Development build
cargo build

# Release build (much faster!)
cargo build --release

# Run all 3D examples
cargo run --release --bin all_examples3

# Run all 2D examples
cargo run --release --bin all_examples2

# Run tests
cargo test

# With parallelism
cargo build --features parallel --release

# With SIMD
cargo build --features simd-stable --release

# Benchmarks
cargo run --release --manifest-path benchmarks3d/Cargo.toml
```

## Cargo Features

- `parallel` - Multi-threaded solving (big performance gain on multi-core)
- `simd-stable` - SIMD optimizations on stable Rust
- `simd-nightly` - More SIMD opts on nightly
- `serde-serialize` - Snapshot/restore support
- `enhanced-determinism` - Stricter determinism (disables SIMD)
- `debug-render` - Visual debugging helpers
- `profiler` - Detailed performance counters
- `dim2` / `dim3` - 2D or 3D (set by crate, not user)
- `f32` / `f64` - Precision (set by crate, not user)

## Resources

- **Official Site**: https://rapier.rs
- **User Guide**: https://rapier.rs/docs/
- **API Reference**: https://docs.rs/rapier3d
- **Discord**: https://discord.gg/vt9DJSW
- **GitHub**: https://github.com/dimforge/rapier
- **Blog**: https://www.dimforge.com/blog
- **Crates.io**: https://crates.io/crates/rapier3d
- **NPM** (JS/WASM): Available for web development

## Related Dimforge Projects

- **Parry**: Collision detection library (Rapier's foundation)
- **Salva**: SPH fluid simulation
- **nphysics**: Previous-gen physics engine (deprecated, use Rapier)
- **nalgebra**: Linear algebra library
- **Bevy_rapier**: Integration with Bevy game engine

## Quick Reference Card

### Most Common Operations

```rust
// Create world
let mut bodies = RigidBodySet::new();
let mut colliders = ColliderSet::new();
let mut joints = ImpulseJointSet::new();
let mut pipeline = PhysicsPipeline::new();

// Add dynamic ball
let body = bodies.insert(RigidBodyBuilder::dynamic().translation(vector![0.0, 5.0, 0.0]).build());
colliders.insert_with_parent(ColliderBuilder::ball(0.5).build(), body, &mut bodies);

// Add static floor
let floor = bodies.insert(RigidBodyBuilder::fixed().build());
colliders.insert_with_parent(ColliderBuilder::cuboid(10.0, 0.1, 10.0).build(), floor, &mut bodies);

// Simulate
pipeline.step(&gravity, &params, &mut islands, &mut broad_phase, &mut narrow_phase,
              &mut bodies, &mut colliders, &mut joints, &mut multibody_joints,
              &mut ccd_solver, &(), &());

// Query
let pos = bodies[body].translation();
let vel = bodies[body].linvel();

// Modify
bodies[body].apply_impulse(vector![0.0, 100.0, 0.0], true);
bodies[body].set_linvel(vector![1.0, 0.0, 0.0], true);
```

### Material Presets

```rust
// Ice
.friction(0.01).restitution(0.1)

// Wood
.friction(0.5).restitution(0.2)

// Rubber
.friction(1.0).restitution(0.8)

// Metal
.friction(0.6).restitution(0.3)

// Bouncy ball
.friction(0.7).restitution(0.9)
```

### Common Densities (kg/m³)

```rust
.density(1000.0)   // Water
.density(2700.0)   // Aluminum
.density(7850.0)   // Steel
.density(11340.0)  // Lead
.density(920.0)    // Ice
.density(1.2)      // Air
```

This documentation provides complete coverage of Rapier's architecture, APIs, usage patterns, and best practices for both beginners and advanced users!
