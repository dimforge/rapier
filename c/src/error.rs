use crate::*;
use std::{
    cell::{Cell, RefCell},
    ffi::{CString, c_char, c_void},
    panic::{AssertUnwindSafe, catch_unwind},
};

/// Status-returning operations use these integer codes.
/// @ingroup errors
pub type RprStatus = u32;
/// @ingroup errors
/// Operation succeeded.
pub const RPR_OK: RprStatus = 0;
/// @ingroup errors
/// A required pointer was NULL.
pub const RPR_NULL_POINTER: RprStatus = 1;
/// @ingroup errors
/// An argument failed validation.
pub const RPR_INVALID_ARGUMENT: RprStatus = 2;
/// @ingroup errors
/// The entity handle is stale, invalid, or belongs to another world.
pub const RPR_INVALID_HANDLE: RprStatus = 3;
/// @ingroup errors
/// Output capacity is insufficient; the returned count is the required capacity.
pub const RPR_BUFFER_TOO_SMALL: RprStatus = 4;
/// @ingroup errors
/// This build or object does not support the operation.
pub const RPR_UNSUPPORTED: RprStatus = 5;
/// @ingroup errors
/// Rust panicked; discard objects mutated by the call.
pub const RPR_PANIC: RprStatus = 6;
/// @ingroup errors
/// No matching query result or object was found.
pub const RPR_NOT_FOUND: RprStatus = 7;
/// @ingroup errors
/// Conflicting or reentrant access to simulation state. No mutation was performed.
pub const RPR_WORLD_BUSY: RprStatus = 8;
pub(crate) type Result<T = ()> = std::result::Result<T, (RprStatus, String)>;
thread_local! { static LAST_STATUS: Cell<RprStatus> = const { Cell::new(RPR_OK) }; }
thread_local! { static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default()); }
/// Called synchronously on the calling thread when an operation reports an error.
/// The diagnostic is borrowed for the duration of the callback. The callback
/// must return normally or terminate the process: never throw or longjmp across
/// the Rust/C boundary. Nested failing calls do not invoke the handler recursively.
/// @ingroup errors
pub type RprErrorCallback = Option<unsafe extern "C" fn(RprStatus, *const c_char, *mut c_void)>;

/// An optional thread-local error handler. A null callback disables reporting.
/// Keep the callback and user_data alive until the handler is replaced.
/// @ingroup errors
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct RprErrorHandler {
    /// Optional error callback; NULL disables notifications.
    pub callback: RprErrorCallback,
    /// Application data; Rapier does not own pointers encoded in it.
    pub user_data: *mut c_void,
}

thread_local! {
    static ERROR_HANDLER: Cell<RprErrorHandler> = const { Cell::new(RprErrorHandler {
        callback: None,
        user_data: std::ptr::null_mut(),
    }) };
    static FFI_DEPTH: Cell<usize> = const { Cell::new(0) };
}

/// Replace this thread's error handler and return the previous handler so it can
/// be restored at the end of a scope. Status returns are unchanged. A handler
/// that returns lets the caller recover by checking the status; a fail-fast
/// handler may terminate the process. Includes RPR_NOT_FOUND query misses.
/// @ingroup errors
#[rapier_export]
pub unsafe extern "C" fn rpr_set_error_handler(handler: RprErrorHandler) -> RprErrorHandler {
    ERROR_HANDLER.with(|current| current.replace(handler))
}

pub(crate) fn invalid(message: impl Into<String>) -> (RprStatus, String) {
    (RPR_INVALID_ARGUMENT, message.into())
}
pub(crate) fn missing() -> (RprStatus, String) {
    (RPR_INVALID_HANDLE, "invalid or stale handle".into())
}
pub(crate) fn ensure(condition: bool, message: &str) -> Result {
    if condition {
        Ok(())
    } else {
        Err(invalid(message))
    }
}
pub(crate) fn finite(value: Real) -> Result<Real> {
    ensure(value.is_finite(), "expected a finite number")?;
    Ok(value)
}
pub(crate) fn nonnegative(value: Real) -> Result<Real> {
    finite(value)?;
    ensure(value >= 0.0, "expected a nonnegative number")?;
    Ok(value)
}
pub(crate) fn positive(value: Real) -> Result<Real> {
    finite(value)?;
    ensure(value > 0.0, "expected a positive number")?;
    Ok(value)
}
pub(crate) fn boolean(value: u32) -> Result<bool> {
    ensure(value <= 1, "boolean must be 0 or 1")?;
    Ok(value != 0)
}
struct FfiCall;
impl Drop for FfiCall {
    fn drop(&mut self) {
        FFI_DEPTH.with(|depth| depth.set(depth.get() - 1));
    }
}

pub(crate) fn ffi(f: impl FnOnce() -> Result) -> RprStatus {
    let outermost = FFI_DEPTH.with(|depth| {
        let previous = depth.get();
        depth.set(previous + 1);
        previous == 0
    });
    let _call = FfiCall;
    let result = catch_unwind(AssertUnwindSafe(f)).unwrap_or_else(|payload| {
        let msg = payload
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("Rust panic");
        Err((
            RPR_PANIC,
            format!("Rapier panic: {msg}; discard objects mutated by this call"),
        ))
    });
    match result {
        Ok(()) => {
            LAST_STATUS.with(|s| s.set(RPR_OK));
            LAST_ERROR.with(|e| *e.borrow_mut() = CString::default());
            RPR_OK
        }
        Err((status, message)) => {
            LAST_STATUS.with(|s| s.set(status));
            let message = CString::new(message.replace('\0', "?")).unwrap();
            LAST_ERROR.with(|e| *e.borrow_mut() = message.clone());
            let handler = ERROR_HANDLER.with(Cell::get);
            if let Some(callback) = handler.callback {
                if outermost {
                    // All operation-local borrows and the panic boundary have ended.
                    // Keep the diagnostic alive even if the handler calls Rapier again.
                    unsafe { callback(status, message.as_ptr(), handler.user_data) };
                    LAST_ERROR.with(|e| *e.borrow_mut() = message);
                    LAST_STATUS.with(|s| s.set(status));
                }
            }
            status
        }
    }
}
/// Status of the most recent fallible operation on this thread. Reading this or
/// LastError does not clear it. Infallible value constructors do not change it.
/// Check immediately after a fallible value-returning operation when recovering
/// from errors instead of using a fail-fast error callback.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_last_status() -> RprStatus {
    LAST_STATUS.with(Cell::get)
}

/// Capture a newly produced value without exposing an output pointer in the ABI.
/// Errors return the type's default value. A short-buffer error preserves the
/// required length so callers can resize and retry.
pub(crate) fn ffi_value<T: Default>(f: impl FnOnce(*mut T) -> RprStatus) -> T {
    let mut value = T::default();
    let status = f(&mut value);
    if status != RPR_OK && status != RPR_BUFFER_TOO_SMALL {
        return T::default();
    }
    value
}

/// Thread-local UTF-8 diagnostic, valid until the next fallible call on this thread.
/// @ingroup errors
#[rapier_export]
pub extern "C" fn rpr_last_error() -> *const c_char {
    LAST_ERROR.with(|e| e.borrow().as_ptr())
}
/// These helpers check null and alignment, not allocation validity or ownership.
pub(crate) unsafe fn get<'a, T>(p: *const T) -> Result<&'a T> {
    if p.is_null() {
        return Err((RPR_NULL_POINTER, "null pointer".into()));
    }
    ensure(p.is_aligned(), "misaligned pointer")?;
    Ok(unsafe { &*p })
}
pub(crate) unsafe fn get_mut<'a, T>(p: *mut T) -> Result<&'a mut T> {
    if p.is_null() {
        return Err((RPR_NULL_POINTER, "null pointer".into()));
    }
    ensure(p.is_aligned(), "misaligned pointer")?;
    Ok(unsafe { &mut *p })
}
pub(crate) unsafe fn input<'a, T>(p: *const T, count: usize) -> Result<&'a [T]> {
    if count == 0 {
        return Ok(&[]);
    }
    unsafe {
        get(p)?;
    }
    ensure(
        count <= isize::MAX as usize / std::mem::size_of::<T>().max(1),
        "array is too large",
    )?;
    Ok(unsafe { std::slice::from_raw_parts(p, count) })
}
pub(crate) unsafe fn output<T>(p: *mut T, value: T) -> Result {
    if p.is_null() {
        return Err((RPR_NULL_POINTER, "null output pointer".into()));
    }
    ensure(p.is_aligned(), "misaligned output pointer")?;
    unsafe {
        p.write(value);
    }
    Ok(())
}
pub(crate) unsafe fn out_ptr<T>(p: *mut T) -> Result {
    if p.is_null() {
        return Err((RPR_NULL_POINTER, "null output pointer".into()));
    }
    ensure(p.is_aligned(), "misaligned output pointer")
}
/// A null buffer with capacity zero is a successful size query. Otherwise no partial writes.
pub(crate) unsafe fn copy_out<T: Copy>(
    values: &[T],
    buffer: *mut T,
    capacity: usize,
    count: *mut usize,
) -> Result {
    unsafe {
        out_ptr(count)?;
    }
    if buffer.is_null() && capacity == 0 {
        return unsafe { output(count, values.len()) };
    }
    ensure(
        capacity <= isize::MAX as usize / std::mem::size_of::<T>().max(1),
        "buffer is too large",
    )?;
    unsafe {
        out_ptr(buffer)?;
    }
    unsafe {
        output(count, values.len())?;
    }
    if capacity < values.len() {
        return Err((RPR_BUFFER_TOO_SMALL, "output buffer is too small".into()));
    }
    unsafe {
        std::ptr::copy_nonoverlapping(values.as_ptr(), buffer, values.len());
    }
    Ok(())
}
