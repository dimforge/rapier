use self::velocity_solver::VelocitySolver;
use crate::alloc_prelude::*;

use contact_constraint::*;
pub(crate) use joint_constraint::MotorParameters;
pub use joint_constraint::*;
use solver_body::SolverVel;

mod categorization;
mod contact_constraint;
mod interaction_groups;
mod joint_constraint;
pub(crate) mod manifold_store;
mod solver_body;
pub(crate) mod solver_contact_graph;
mod staged_island_solver;
pub(crate) use staged_island_solver::StagedIslandSolver;
mod velocity_solver;

// TODO: SAFETY: restrict with bytemuck::Zeroable to make this safe.
/// Sets `buffer` to exactly `len` elements while REUSING the existing content:
/// surviving elements keep their stale-but-initialized bytes from the previous
/// step (only growth is zero-filled). Callers must overwrite, at generation
/// time, every field that is later read — the contact `generate` paths do
/// (including explicit accumulator zeroing). This removes the per-step
/// O(constraint-arena) memset that [`reset_buffer`] pays.
///
/// # Safety
/// Same contract as [`reset_buffer`] for the grown region; `T` must be plain
/// old data (no drop, any bit pattern valid).
pub unsafe fn reset_buffer_reusing<T>(buffer: &mut Vec<T>, len: usize) {
    if len <= buffer.len() {
        buffer.truncate(len);
    } else {
        let old_len = buffer.len();
        buffer.reserve(len - old_len);
        unsafe {
            buffer
                .as_mut_ptr()
                .add(old_len)
                .write_bytes(0, len - old_len);
            buffer.set_len(len);
        }
    }
}

pub unsafe fn reset_buffer<T>(buffer: &mut Vec<T>, len: usize) {
    buffer.clear();
    buffer.reserve(len);

    unsafe {
        // NOTE: writing zeros is faster than u8::MAX.
        buffer.as_mut_ptr().write_bytes(0, len);
        buffer.set_len(len);
    }
}
