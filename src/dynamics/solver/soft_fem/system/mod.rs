//! The per-soft-body FEM system: element state, the assembly of `A = M + h D + h² K`, and the
//! substep's two solves (the implicit elastic predictor, the constraint impulses' propagation).

mod soft_fem_system_assemble;
mod soft_fem_system_elements;
mod soft_fem_system_prepare;
mod soft_fem_system_response;
#[cfg(test)]
mod tests;

pub(crate) use soft_fem_system_elements::SoftFemSystem;
