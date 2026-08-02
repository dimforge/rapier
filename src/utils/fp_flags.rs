//! Floating-point control flags for exception handling.

/// This is an RAII structure that disables floating point exceptions while
/// it is alive, so that operations which generate NaNs and infinite values
/// intentionally will not trip an exception when debugging problematic
/// code that is generating NaNs and infinite values erroneously.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct DisableFloatingPointExceptionsFlags {
    #[cfg(feature = "debug-disable-legitimate-fe-exceptions")]
    // We can't get a precise size for this, because it's of type
    // `fenv_t`, which is a definition that doesn't exist in rust
    // (not even in the libc crate, as of the time of writing.)
    // But since the state is intended to be stored on the stack,
    // 256 bytes should be more than enough.
    original_flags: [u8; 256],
}

#[cfg(feature = "debug-disable-legitimate-fe-exceptions")]
extern "C" {
    fn feholdexcept(env: *mut core::ffi::c_void);
    fn fesetenv(env: *const core::ffi::c_void);
}

impl DisableFloatingPointExceptionsFlags {
    #[cfg(not(feature = "debug-disable-legitimate-fe-exceptions"))]
    #[allow(dead_code)]
    /// Disables floating point exceptions as long as this object is not dropped.
    pub fn disable_floating_point_exceptions() -> Self {
        Self {}
    }

    #[cfg(feature = "debug-disable-legitimate-fe-exceptions")]
    /// Disables floating point exceptions as long as this object is not dropped.
    pub fn disable_floating_point_exceptions() -> Self {
        unsafe {
            let mut original_flags = [0; 256];
            feholdexcept(original_flags.as_mut_ptr() as *mut _);
            Self { original_flags }
        }
    }
}

#[cfg(feature = "debug-disable-legitimate-fe-exceptions")]
impl Drop for DisableFloatingPointExceptionsFlags {
    fn drop(&mut self) {
        unsafe {
            fesetenv(self.original_flags.as_ptr() as *const _);
        }
    }
}
