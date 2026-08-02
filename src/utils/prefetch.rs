//! Software-prefetch hint, used by hot loops that walk large structs through an
//! index array (a pattern the hardware prefetcher can't predict).

/// Hints the CPU to bring the cache line containing `ptr` into L1 for reading.
///
/// `OFFSETS_64B` extra consecutive cache lines are prefetched after the first
/// one, for structs spanning multiple lines. This is a pure hint: it has no
/// architectural effect and is a no-op on unsupported targets.
#[inline(always)]
#[allow(unused_variables)]
pub(crate) fn prefetch_read<const OFFSETS_64B: usize, T>(ptr: *const T) {
    for i in 0..=OFFSETS_64B {
        let ptr = unsafe { (ptr as *const u8).add(i * 64) };

        #[cfg(target_arch = "x86_64")]
        unsafe {
            core::arch::x86_64::_mm_prefetch::<{ core::arch::x86_64::_MM_HINT_T0 }>(
                ptr as *const i8,
            );
        }

        #[cfg(target_arch = "aarch64")]
        unsafe {
            core::arch::asm!(
                "prfm pldl1keep, [{0}]",
                in(reg) ptr,
                options(nostack, preserves_flags),
            );
        }

        #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
        let _ = ptr;
    }
}
