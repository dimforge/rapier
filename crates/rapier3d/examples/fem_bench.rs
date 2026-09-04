//! `cargo run -p rapier3d --features fem --release --example fem_bench`: A/B of the FEM soft-body
//! solver against the constraint rows; reports per scene and solver the median step time and a
//! quality figure (a loaded beam's static deflection or a settling body's residual velocity).

#[cfg(feature = "fem")]
use rapier3d::prelude::*;
#[cfg(feature = "fem")]
use std::time::Instant;

#[cfg(not(feature = "fem"))]
fn main() {
    println!("this bench needs --features fem");
}

#[cfg(feature = "fem")]
fn main() {
    println!(
        "{:<22} {:<12} {:>7} {:>10} {:>12} {:>12}",
        "scene", "solver", "substeps", "ms/step", "deflection", "max |v|"
    );
    for substeps in [1usize, 4] {
        for solver in [SoftBodySolver::Constraints, SoftBodySolver::Fem] {
            cantilever(solver, substeps);
        }
    }
    for solver in [SoftBodySolver::Constraints, SoftBodySolver::Fem] {
        jelly_pile(solver);
        cloth(solver);
    }
}

#[cfg(feature = "fem")]
fn report(
    scene: &str,
    solver: SoftBodySolver,
    substeps: usize,
    ms: f64,
    deflection: Real,
    v: Real,
) {
    let name = match solver {
        SoftBodySolver::Constraints => "constraints",
        SoftBodySolver::Fem => "fem",
    };
    println!("{scene:<22} {name:<12} {substeps:>7} {ms:>10.3} {deflection:>12.5} {v:>12.5}");
}

/// Median of the per-step times of `steps` steps, discarding the first tenth (warm-up).
#[cfg(feature = "fem")]
fn run(world: &mut PhysicsWorld, steps: usize) -> f64 {
    let mut times = Vec::with_capacity(steps);
    for _ in 0..steps {
        let start = Instant::now();
        world.step();
        times.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    let skip = steps / 10;
    let mut tail: Vec<f64> = times[skip..].to_vec();
    tail.sort_by(|a, b| a.partial_cmp(b).unwrap());
    tail[tail.len() / 2]
}

/// A stiff cantilever under its own weight: the deflection is the quality figure (the analytic
/// Euler-Bernoulli value is -0.0491 m).
#[cfg(feature = "fem")]
fn cantilever(solver: SoftBodySolver, substeps: usize) {
    let (length, thickness, young) = (2.0, 0.2, 1.2e7);
    let mut world = PhysicsWorld::new();
    world.integration_parameters.num_solver_iterations = substeps;
    let half = Vector::new(length * 0.5, thickness * 0.5, thickness * 0.5);
    let builder = SoftBodyBuilder::cuboid(Vector::new(length * 0.5, 0.0, 0.0), half, 9, 3, 3)
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: young,
            poisson_ratio: 0.0,
            elastic_damping_ratio: 1.0,
            ..Default::default()
        })
        .solver(solver)
        .mass(8.0)
        .no_surface_collider()
        .can_sleep(false);
    let pinned: Vec<u32> = builder
        .particle_positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.x < 1.0e-4)
        .map(|(i, _)| i as u32)
        .collect();
    let tip: Vec<usize> = builder
        .particle_positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.x > length - 1.0e-4)
        .map(|(i, _)| i)
        .collect();
    let handle = world.insert_soft_body(builder.pinned_particles(pinned));
    let ms = run(&mut world, 1500);
    let sb = &world.soft_bodies[handle];
    let deflection =
        tip.iter().map(|&i| sb.particle_position(i).y).sum::<Real>() / tip.len() as Real;
    let max_v = sb
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    report("stiff cantilever", solver, substeps, ms, deflection, max_v);
}

/// Nine jelly cubes dropped in a pile: the throughput case (contacts and self contacts).
#[cfg(feature = "fem")]
fn jelly_pile(solver: SoftBodySolver) {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(12.0, 0.1, 12.0),
    );
    for i in 0..9 {
        let x = ((i % 3) as Real - 1.0) * 1.0;
        let z = ((i / 3) as Real - 1.0) * 1.0;
        let cube = SoftBodyBuilder::cuboid(
            Vector::new(x, 0.55 + i as Real * 1.05, z),
            Vector::splat(0.5),
            4,
            4,
            4,
        )
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 3.0e4,
            poisson_ratio: 0.4,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .solver(solver)
        .can_sleep(false)
        .particle_mass(0.1);
        world.insert_soft_body(cube);
    }
    // Let the pile land before timing it: the drop itself is not the throughput case.
    for _ in 0..400 {
        world.step();
    }
    let ms = run(&mut world, 600);
    let max_v = world
        .soft_bodies
        .iter()
        .flat_map(|(_, sb)| sb.particles().iter())
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    report("jelly pile (9 cubes)", solver, 4, ms, 0.0, max_v);
}

/// A pinned cloth: distance and dihedral elements only, no cells.
#[cfg(feature = "fem")]
fn cloth(solver: SoftBodySolver) {
    let mut world = PhysicsWorld::new();
    let n = 30;
    let cloth = SoftBodyBuilder::cloth(
        Vector::new(0.0, 3.0, 0.0),
        Vector::new(0.1, 0.0, 0.0),
        Vector::new(0.0, 0.0, 0.1),
        n,
        n,
    )
    .pinned_particles([0, (n - 1) as u32])
    .softness(SpringCoefficients::new(100.0, 1.0))
    .solver(solver)
    .can_sleep(false)
    .particle_mass(0.02);
    let handle = world.insert_soft_body(cloth);
    let ms = run(&mut world, 600);
    let sb = &world.soft_bodies[handle];
    let max_v = sb
        .particles()
        .iter()
        .map(|p| p.velocity().length())
        .fold(0.0, Real::max);
    let lowest = sb
        .particle_positions()
        .map(|p| p.y)
        .fold(Real::MAX, Real::min);
    report("cloth 30x30", solver, 4, ms, lowest, max_v);
}
