//! Snapshot deltas through the C ABI: replication into receivers, and rejected inputs.
use crate::*;
use std::ffi::CStr;

/// One byte over the 256 MiB limit, which counts the 16-byte header.
const TOO_LARGE: usize = 256 * 1024 * 1024 + 1;

fn last_error() -> String {
    unsafe { CStr::from_ptr(rpr_last_error()) }
        .to_string_lossy()
        .into_owned()
}

#[track_caller]
fn assert_status(status: RprStatus) {
    assert_eq!(rpr_last_status(), status, "{}", last_error());
}

/// Copies and releases owned bytes.
unsafe fn take(bytes: *mut RprBytes) -> Vec<u8> {
    unsafe {
        let view = rpr_bytes_data(bytes);
        let copy = std::slice::from_raw_parts(view.data, view.count).to_vec();
        assert_eq!(rpr_free_bytes(bytes), RPR_OK);
        copy
    }
}

unsafe fn snapshot(world: *const RprWorld) -> Vec<u8> {
    unsafe {
        let bytes = rpr_serialize_world(world);
        assert_status(RPR_OK);
        take(bytes)
    }
}

/// Returns the delta and tick bytes since `since`.
unsafe fn delta(world: *mut RprWorld, since: Option<&[u8]>) -> (Vec<u8>, Vec<u8>) {
    unsafe {
        let (data, count) = since.map_or((std::ptr::null(), 0), |s| (s.as_ptr(), s.len()));
        let result = rpr_serialize_world_delta(world, data, count);
        assert_status(RPR_OK);
        (take(result.delta), take(result.tick))
    }
}

/// Expects `since` to be rejected with `status`.
unsafe fn reject_delta(world: *mut RprWorld, data: *const u8, count: usize, status: RprStatus) {
    unsafe {
        let result = rpr_serialize_world_delta(world, data, count);
        assert_status(status);
        assert!(result.delta.is_null() && result.tick.is_null());
    }
}

unsafe fn change_tick(world: *mut RprWorld) -> Vec<u8> {
    unsafe {
        let tick = rpr_change_tick(world);
        assert_status(RPR_OK);
        take(tick)
    }
}

unsafe fn set_baseline(world: *mut RprWorld, tick: &[u8]) {
    unsafe {
        assert_eq!(
            rpr_set_delta_baseline(world, tick.as_ptr(), tick.len()),
            RPR_OK,
            "{}",
            last_error()
        );
    }
}

unsafe fn restore(snapshot: &[u8]) -> *mut RprWorld {
    unsafe {
        let world = rpr_deserialize_world(snapshot.as_ptr(), snapshot.len());
        assert_status(RPR_OK);
        world
    }
}

unsafe fn apply(world: *mut RprWorld, delta: &[u8]) -> Vec<u8> {
    unsafe {
        let tick = rpr_apply_world_delta(world, delta.as_ptr(), delta.len());
        assert_status(RPR_OK);
        take(tick)
    }
}

unsafe fn reject_apply(world: *mut RprWorld, delta: &[u8], status: RprStatus) {
    unsafe {
        assert!(rpr_apply_world_delta(world, delta.as_ptr(), delta.len()).is_null());
        assert_status(status);
    }
}

unsafe fn with_world<R>(world: *mut RprWorld, f: impl FnOnce(&mut PhysicsWorld) -> R) -> R {
    unsafe {
        let access = (*world).write().unwrap();
        f(&mut (*access.raw()).0)
    }
}

unsafe fn step(world: *mut RprWorld) {
    assert_eq!(
        unsafe { rpr_step(world, std::ptr::null(), std::ptr::null()) },
        RPR_OK
    );
}

fn ball(world: &mut PhysicsWorld, x: Real, y: Real) -> RigidBodyHandle {
    let body =
        world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::X * x + Vector::Y * y));
    world.insert_collider(ColliderBuilder::ball(0.5), Some(body));
    body
}

/// A ground and a few falling balls, stepped until they are in contact.
unsafe fn pile() -> *mut RprWorld {
    unsafe {
        let world = rpr_new_world();
        with_world(world, |w| {
            #[cfg(feature = "dim2")]
            let ground = ColliderBuilder::cuboid(10.0, 0.5);
            #[cfg(feature = "dim3")]
            let ground = ColliderBuilder::cuboid(10.0, 0.5, 10.0);
            w.insert_collider(ground, None);
            for i in 0..4 {
                ball(w, 0.1 * i as Real, 1.0 + 1.1 * i as Real);
            }
        });
        for _ in 0..10 {
            step(world);
        }
        world
    }
}

struct Receiver {
    world: *mut RprWorld,
    acked: Option<Vec<u8>>,
    every: usize,
}

impl Receiver {
    unsafe fn sync(&mut self, sender: *mut RprWorld) {
        unsafe {
            let (delta, tick) = delta(sender, self.acked.as_deref());
            assert_eq!(apply(self.world, &delta), tick);
            self.acked = Some(tick);
            assert!(snapshot(self.world) == snapshot(sender));
        }
    }
}

#[test]
fn world_deltas_replicate_a_stepping_world() {
    unsafe {
        let sender = pile();
        assert_eq!(rpr_enable_change_tracking(sender), RPR_OK);
        // Restored receivers declare the tick issued right before their snapshot.
        let baseline = change_tick(sender);
        let snapshot0 = snapshot(sender);
        let restored = || {
            let world = restore(&snapshot0);
            set_baseline(world, &baseline);
            world
        };
        let mut receivers = [
            Receiver {
                world: restored(),
                acked: Some(baseline.clone()),
                every: 1,
            },
            Receiver {
                world: rpr_new_world(),
                acked: None,
                every: 1,
            },
            // Lags behind, each delta against its own older acknowledged tick.
            Receiver {
                world: restored(),
                acked: Some(baseline.clone()),
                every: 4,
            },
        ];
        let mut spawned = None;
        for i in 0..30 {
            match i {
                5 => spawned = Some(with_world(sender, |w| ball(w, 2.0, 3.0))),
                13 => with_world(sender, |w| {
                    w.remove_body_with_colliders(spawned.unwrap(), true)
                        .unwrap();
                }),
                _ => {}
            }
            step(sender);
            for receiver in &mut receivers {
                if (i + 1) % receiver.every == 0 {
                    receiver.sync(sender);
                }
            }
        }

        // Once caught up, receivers keep simulating identically.
        receivers[2].sync(sender);
        for _ in 0..20 {
            step(sender);
            for receiver in &receivers {
                step(receiver.world);
                assert!(snapshot(receiver.world) == snapshot(sender));
            }
        }
        for receiver in receivers {
            assert_eq!(rpr_free_world(receiver.world), RPR_OK);
        }
        assert_eq!(rpr_free_world(sender), RPR_OK);
    }
}

#[test]
fn world_deltas_reject_bad_baselines_and_bytes() {
    unsafe {
        let null: *const u8 = std::ptr::null();
        let sender = pile();

        // Without change tracking.
        reject_delta(sender, null, 0, RPR_UNSUPPORTED);
        assert!(rpr_change_tick(sender).is_null());
        assert_status(RPR_UNSUPPORTED);
        assert_eq!(rpr_enable_change_tracking(sender), RPR_OK);
        let old_tick = change_tick(sender);
        assert_eq!(rpr_disable_change_tracking(sender), RPR_OK);
        reject_delta(sender, old_tick.as_ptr(), old_tick.len(), RPR_UNSUPPORTED);
        // A restarted clock rejects the ticks issued before.
        assert_eq!(rpr_enable_change_tracking(sender), RPR_OK);
        reject_delta(
            sender,
            old_tick.as_ptr(),
            old_tick.len(),
            RPR_BASELINE_MISMATCH,
        );
        let (full, tick0) = delta(sender, None);

        // Ticks of another world.
        let other = rpr_new_world();
        assert_eq!(rpr_enable_change_tracking(other), RPR_OK);
        let other_tick = change_tick(other);
        reject_delta(
            sender,
            other_tick.as_ptr(),
            other_tick.len(),
            RPR_BASELINE_MISMATCH,
        );

        // Malformed tick bytes: a NULL tick with a length (NULL_POINTER, as for every input),
        // header damage, truncation, trailing bytes, or delta bytes in place of a tick.
        reject_delta(sender, null, 1, RPR_NULL_POINTER);
        for index in 0..16 {
            let mut damaged = tick0.clone();
            damaged[index] ^= 1;
            reject_delta(
                sender,
                damaged.as_ptr(),
                damaged.len(),
                RPR_INVALID_ARGUMENT,
            );
        }
        for len in 0..tick0.len() {
            reject_delta(sender, tick0.as_ptr(), len, RPR_INVALID_ARGUMENT);
        }
        let mut trailing = tick0.clone();
        trailing.push(0);
        reject_delta(
            sender,
            trailing.as_ptr(),
            trailing.len(),
            RPR_INVALID_ARGUMENT,
        );
        reject_delta(sender, full.as_ptr(), full.len(), RPR_INVALID_ARGUMENT);
        reject_delta(sender, tick0.as_ptr(), TOO_LARGE, RPR_INVALID_ARGUMENT);
        reject_delta(std::ptr::null_mut(), null, 0, RPR_NULL_POINTER);
        assert!(rpr_change_tick(std::ptr::null_mut()).is_null());
        assert_status(RPR_NULL_POINTER);

        // Malformed baseline declarations.
        let declare = |world, data: *const u8, count| {
            let status = rpr_set_delta_baseline(world, data, count);
            assert_eq!(status, rpr_last_status());
            status
        };
        assert_eq!(declare(sender, null, 0), RPR_INVALID_ARGUMENT);
        assert_eq!(declare(sender, null, 1), RPR_NULL_POINTER);
        assert_eq!(
            declare(sender, full.as_ptr(), full.len()),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            declare(sender, tick0.as_ptr(), tick0.len() - 1),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            declare(sender, tick0.as_ptr(), TOO_LARGE),
            RPR_INVALID_ARGUMENT
        );
        assert_eq!(
            declare(std::ptr::null_mut(), tick0.as_ptr(), tick0.len()),
            RPR_NULL_POINTER
        );

        // Malformed delta bytes leave the receiver untouched.
        let receiver = rpr_new_world();
        let empty = snapshot(receiver);
        reject_apply(receiver, b"not a delta", RPR_INVALID_ARGUMENT);
        reject_apply(receiver, &[], RPR_INVALID_ARGUMENT);
        reject_apply(receiver, &tick0, RPR_INVALID_ARGUMENT);
        for index in 0..16 {
            let mut damaged = full.clone();
            damaged[index] ^= 1;
            reject_apply(receiver, &damaged, RPR_INVALID_ARGUMENT);
        }
        reject_apply(receiver, &full[..full.len() - 1], RPR_INVALID_ARGUMENT);
        let mut trailing = full.clone();
        trailing.push(0);
        reject_apply(receiver, &trailing, RPR_INVALID_ARGUMENT);
        assert!(rpr_apply_world_delta(receiver, full.as_ptr(), TOO_LARGE).is_null());
        assert_status(RPR_INVALID_ARGUMENT);
        assert!(rpr_apply_world_delta(std::ptr::null_mut(), full.as_ptr(), full.len()).is_null());
        assert_status(RPR_NULL_POINTER);
        assert!(snapshot(receiver) == empty);

        // A delta against a baseline the receiver moved past is rejected.
        assert_eq!(apply(receiver, &full), tick0);
        step(sender);
        let (delta1, tick1) = delta(sender, Some(&tick0));
        assert_eq!(apply(receiver, &delta1), tick1);
        step(sender);
        let (stale, _) = delta(sender, Some(&tick0));
        reject_apply(receiver, &stale, RPR_BASELINE_MISMATCH);

        // A full delta resynchronizes it.
        let (full, tick) = delta(sender, None);
        assert_eq!(apply(receiver, &full), tick);
        assert!(snapshot(receiver) == snapshot(sender));

        // Stepping a receiver clears its baseline, even if it stays identical to the sender.
        step(sender);
        step(receiver);
        let (next, _) = delta(sender, Some(&tick));
        reject_apply(receiver, &next, RPR_BASELINE_MISMATCH);
        let (full, tick) = delta(sender, None);
        assert_eq!(apply(receiver, &full), tick);

        // A restored receiver needs its declared baseline, and a fresh one a full delta first.
        let before = change_tick(sender);
        let restored = restore(&snapshot(sender));
        let fresh = rpr_new_world();
        step(sender);
        let (next, next_tick) = delta(sender, Some(&before));
        reject_apply(restored, &next, RPR_BASELINE_MISMATCH);
        reject_apply(fresh, &next, RPR_BASELINE_MISMATCH);
        set_baseline(restored, &before);
        assert_eq!(apply(restored, &next), next_tick);
        assert!(snapshot(restored) == snapshot(sender));

        for world in [sender, other, receiver, restored, fresh] {
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
