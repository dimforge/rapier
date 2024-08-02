use super::InteractionGroups;
use crate::math::Real;
use na::DVector;

pub(crate) trait ConstraintTypes {
    type OneBody;
    type TwoBodies;
    type GenericOneBody;
    type GenericTwoBodies;

    #[cfg(feature = "simd-is-enabled")]
    type SimdOneBody;
    #[cfg(feature = "simd-is-enabled")]
    type SimdTwoBodies;

    type BuilderOneBody;
    type BuilderTwoBodies;

    type GenericBuilderOneBody;
    type GenericBuilderTwoBodies;

    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderOneBody;
    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderTwoBodies;
}

#[derive(Debug)]
pub enum AnyConstraintMut<'a, Constraints: ConstraintTypes> {
    OneBody(&'a mut Constraints::OneBody),
    TwoBodies(&'a mut Constraints::TwoBodies),
    GenericOneBody(&'a mut Constraints::GenericOneBody),
    GenericTwoBodies(&'a mut Constraints::GenericTwoBodies),
    #[cfg(feature = "simd-is-enabled")]
    SimdOneBody(&'a mut Constraints::SimdOneBody),
    #[cfg(feature = "simd-is-enabled")]
    SimdTwoBodies(&'a mut Constraints::SimdTwoBodies),
}

pub(crate) struct SolverConstraintsSet<Constraints: ConstraintTypes> {
    pub generic_jacobians: DVector<Real>,
    pub two_body_interactions: Vec<usize>,
    pub one_body_interactions: Vec<usize>,
    pub generic_two_body_interactions: Vec<usize>,
    pub generic_one_body_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub one_body_interaction_groups: InteractionGroups,

    pub velocity_constraints: Vec<Constraints::TwoBodies>,
    pub generic_velocity_constraints: Vec<Constraints::GenericTwoBodies>,
    #[cfg(feature = "simd-is-enabled")]
    pub simd_velocity_constraints: Vec<Constraints::SimdTwoBodies>,
    pub velocity_one_body_constraints: Vec<Constraints::OneBody>,
    pub generic_velocity_one_body_constraints: Vec<Constraints::GenericOneBody>,
    #[cfg(feature = "simd-is-enabled")]
    pub simd_velocity_one_body_constraints: Vec<Constraints::SimdOneBody>,

    pub velocity_constraints_builder: Vec<Constraints::BuilderTwoBodies>,
    pub generic_velocity_constraints_builder: Vec<Constraints::GenericBuilderTwoBodies>,
    #[cfg(feature = "simd-is-enabled")]
    pub simd_velocity_constraints_builder: Vec<Constraints::SimdBuilderTwoBodies>,
    pub velocity_one_body_constraints_builder: Vec<Constraints::BuilderOneBody>,
    pub generic_velocity_one_body_constraints_builder: Vec<Constraints::GenericBuilderOneBody>,
    #[cfg(feature = "simd-is-enabled")]
    pub simd_velocity_one_body_constraints_builder: Vec<Constraints::SimdBuilderOneBody>,
}

impl<Constraints: ConstraintTypes> SolverConstraintsSet<Constraints> {
    pub fn new() -> Self {
        Self {
            generic_jacobians: DVector::zeros(0),
            two_body_interactions: vec![],
            one_body_interactions: vec![],
            generic_two_body_interactions: vec![],
            generic_one_body_interactions: vec![],
            interaction_groups: InteractionGroups::new(),
            one_body_interaction_groups: InteractionGroups::new(),

            velocity_constraints: vec![],
            generic_velocity_constraints: vec![],
            #[cfg(feature = "simd-is-enabled")]
            simd_velocity_constraints: vec![],
            velocity_one_body_constraints: vec![],
            generic_velocity_one_body_constraints: vec![],
            #[cfg(feature = "simd-is-enabled")]
            simd_velocity_one_body_constraints: vec![],

            velocity_constraints_builder: vec![],
            generic_velocity_constraints_builder: vec![],
            #[cfg(feature = "simd-is-enabled")]
            simd_velocity_constraints_builder: vec![],
            velocity_one_body_constraints_builder: vec![],
            generic_velocity_one_body_constraints_builder: vec![],
            #[cfg(feature = "simd-is-enabled")]
            simd_velocity_one_body_constraints_builder: vec![],
        }
    }

    #[allow(dead_code)] // Useful for debugging.
    pub fn print_counts(&self) {
        println!("Solver constraints:");
        println!(
            "|__ two_body_interactions: {}",
            self.two_body_interactions.len()
        );
        println!(
            "|__ one_body_interactions: {}",
            self.one_body_interactions.len()
        );
        println!(
            "|__ generic_two_body_interactions: {}",
            self.generic_two_body_interactions.len()
        );
        println!(
            "|__ generic_one_body_interactions: {}",
            self.generic_one_body_interactions.len()
        );

        println!(
            "|__ velocity_constraints: {}",
            self.velocity_constraints.len()
        );
        println!(
            "|__ generic_velocity_constraints: {}",
            self.generic_velocity_constraints.len()
        );
        #[cfg(feature = "simd-is-enabled")]
        println!(
            "|__ simd_velocity_constraints: {}",
            self.simd_velocity_constraints.len()
        );
        println!(
            "|__ velocity_one_body_constraints: {}",
            self.velocity_one_body_constraints.len()
        );
        println!(
            "|__ generic_velocity_one_body_constraints: {}",
            self.generic_velocity_one_body_constraints.len()
        );
        #[cfg(feature = "simd-is-enabled")]
        println!(
            "|__ simd_velocity_one_body_constraints: {}",
            self.simd_velocity_one_body_constraints.len()
        );

        println!(
            "|__ velocity_constraints_builder: {}",
            self.velocity_constraints_builder.len()
        );
        println!(
            "|__ generic_velocity_constraints_builder: {}",
            self.generic_velocity_constraints_builder.len()
        );
        #[cfg(feature = "simd-is-enabled")]
        println!(
            "|__ simd_velocity_constraints_builder: {}",
            self.simd_velocity_constraints_builder.len()
        );
        println!(
            "|__ velocity_one_body_constraints_builder: {}",
            self.velocity_one_body_constraints_builder.len()
        );
        println!(
            "|__ generic_velocity_one_body_constraints_builder: {}",
            self.generic_velocity_one_body_constraints_builder.len()
        );
        #[cfg(feature = "simd-is-enabled")]
        println!(
            "|__ simd_velocity_one_body_constraints_builder: {}",
            self.simd_velocity_one_body_constraints_builder.len()
        );
    }

    pub fn clear_constraints(&mut self) {
        self.generic_jacobians.fill(0.0);
        self.velocity_constraints.clear();
        self.velocity_one_body_constraints.clear();
        self.generic_velocity_constraints.clear();
        self.generic_velocity_one_body_constraints.clear();

        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_velocity_constraints.clear();
            self.simd_velocity_one_body_constraints.clear();
        }
    }

    pub fn clear_builders(&mut self) {
        self.velocity_constraints_builder.clear();
        self.velocity_one_body_constraints_builder.clear();
        self.generic_velocity_constraints_builder.clear();
        self.generic_velocity_one_body_constraints_builder.clear();

        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_velocity_constraints_builder.clear();
            self.simd_velocity_one_body_constraints_builder.clear();
        }
    }

    // Returns the generic jacobians and a mutable iterator through all the constraints.
    pub fn iter_constraints_mut(
        &mut self,
    ) -> (
        &DVector<Real>,
        impl Iterator<Item = AnyConstraintMut<Constraints>>,
    ) {
        let jac = &self.generic_jacobians;
        let a = self
            .velocity_constraints
            .iter_mut()
            .map(AnyConstraintMut::TwoBodies);
        let b = self
            .generic_velocity_constraints
            .iter_mut()
            .map(AnyConstraintMut::GenericTwoBodies);
        #[cfg(feature = "simd-is-enabled")]
        let c = self
            .simd_velocity_constraints
            .iter_mut()
            .map(AnyConstraintMut::SimdTwoBodies);
        let d = self
            .velocity_one_body_constraints
            .iter_mut()
            .map(AnyConstraintMut::OneBody);
        let e = self
            .generic_velocity_one_body_constraints
            .iter_mut()
            .map(AnyConstraintMut::GenericOneBody);
        #[cfg(feature = "simd-is-enabled")]
        let f = self
            .simd_velocity_one_body_constraints
            .iter_mut()
            .map(AnyConstraintMut::SimdOneBody);

        #[cfg(feature = "simd-is-enabled")]
        return (jac, a.chain(b).chain(c).chain(d).chain(e).chain(f));

        #[cfg(not(feature = "simd-is-enabled"))]
        return (jac, a.chain(b).chain(d).chain(e));
    }
}
