//! Unit tests of the FEM system: the step-matrix responses and the column solves.

use super::SoftFemSystem;
use crate::alloc_prelude::*;
use crate::dynamics::SoftFemParameters;
use crate::math::Vector;
use crate::dynamics::{SoftBodyBuilder, SoftBodyCellModel, SoftBodyMaterial};

/// A response solves `A_step u = Jᵀ` (checked against the matrix), is zero at the pinned
/// particles and has gain `J · u`; the dense and conjugate-gradient paths agree.
#[test]
fn response_solves_the_step_matrix() {
    #[cfg(feature = "dim2")]
    let builder = SoftBodyBuilder::grid(Vector::ZERO, Vector::splat(0.5), 4, 4);
    #[cfg(feature = "dim3")]
    let builder = SoftBodyBuilder::cuboid(Vector::ZERO, Vector::splat(0.5), 3, 3, 3);
    let builder = builder
        .cell_model(SoftBodyCellModel::Corotational)
        .material(SoftBodyMaterial {
            young_modulus: 1.0e5,
            poisson_ratio: 0.3,
            elastic_damping_ratio: 0.5,
            ..Default::default()
        })
        .particle_mass(0.1)
        .pinned_particles(vec![0]);
    let mut set = crate::dynamics::SoftBodySet::new();
    let mut bodies = crate::dynamics::RigidBodySet::new();
    let mut colliders = crate::geometry::ColliderSet::new();
    let handle = set.insert(builder, &mut bodies, &mut colliders);
    let sb = &set[handle];
    let n = sb.particles().len();
    let slots: Vec<u32> = (0..n as u32).collect();
    let dt = 1.0 / 240.0;
    let entries = [(1u32, Vector::ONE), (0u32, Vector::ONE)];
    let mut results = Vec::new();
    for max_dense_dofs in [usize::MAX, 0] {
        let params = SoftFemParameters {
            max_dense_dofs,
            linear_tolerance: 1.0e-8,
            max_linear_iterations: 10_000,
            ..Default::default()
        };
        let mut system = SoftFemSystem::default();
        system.prepare(sb, &slots);
        system.factorize_step_matrix(dt, &params);
        let mut pool = vec![Vector::ZERO; n];
        let gain = system.response_into(entries.iter().copied(), &mut pool, &params);
        assert_eq!(pool[0], Vector::ZERO, "pinned particle answered");
        let mut lhs = vec![Vector::ZERO; n];
        system.step_matrix.mul(&pool, &mut lhs);
        for (k, l) in lhs.iter().enumerate() {
            let rhs = if k == 1 { Vector::ONE } else { Vector::ZERO };
            assert!((*l - rhs).length() < 1.0e-3 * (1.0 + rhs.length()), "row {k}: {l:?} vs {rhs:?}");
        }
        assert!((gain - Vector::ONE.dot(pool[1])).abs() < 1.0e-6);
        results.push((gain, pool));
    }
    // The column responses reproduce the direct solve.
    {
        let params = SoftFemParameters {
            linear_tolerance: 1.0e-8,
            max_linear_iterations: 10_000,
            ..Default::default()
        };
        let mut system = SoftFemSystem::default();
        system.prepare(sb, &slots);
        system.factorize_step_matrix(dt, &params);
        #[cfg(feature = "dim2")]
        let entries = [(1u32, Vector::new(0.3, -0.7)), (2u32, Vector::ONE * 0.5)];
        #[cfg(feature = "dim3")]
        let entries = [(1u32, Vector::new(0.3, -0.7, 0.2)), (2u32, Vector::ONE * 0.5)];
        let mut direct = vec![Vector::ZERO; n];
        let gain = system.response_into(entries.iter().copied(), &mut direct, &params);
        system.clear_columns();
        system.load_particle(1);
        system.load_particle(2);
        system.compute_columns(&params);
        let mut combined = vec![Vector::ZERO; n];
        let gain_columns = system
            .response_from_columns(entries.iter().copied(), &mut combined)
            .unwrap();
        assert!((gain - gain_columns).abs() < 1.0e-5 * gain.max(1.0e-6));
        for (a, b) in direct.iter().zip(&combined) {
            assert!((*a - *b).length() < 1.0e-5 * (1.0 + a.length()));
        }
        assert!(
            system
                .response_from_columns([(3u32, Vector::ONE)].into_iter(), &mut combined)
                .is_none(),
            "an unloaded particle must fall back to a solve"
        );
    }
    let (dense, cg) = (&results[0], &results[1]);
    assert!((dense.0 - cg.0).abs() < 1.0e-3 * dense.0, "gains {} vs {}", dense.0, cg.0);
    for (a, b) in dense.1.iter().zip(&cg.1) {
        assert!((*a - *b).length() < 1.0e-3 * (1.0 + a.length()));
    }
}
