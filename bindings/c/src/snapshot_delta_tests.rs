//! Snapshot deltas through the C ABI: replication into receivers, and rejected inputs.
use crate::*;
use std::ffi::CStr;

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
        // Restored receivers start from a snapshot taken at the instant of their baseline.
        let restore = |bytes: &[u8]| {
            let world = rpr_deserialize_world(bytes.as_ptr(), bytes.len());
            assert_status(RPR_OK);
            world
        };
        let snapshot0 = snapshot(sender);
        let (full, baseline) = delta(sender, None);
        assert!(full.starts_with(b"RPRD") && baseline.starts_with(b"RPRT"));
        let mut receivers = [
            Receiver {
                world: restore(&snapshot0),
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
                world: restore(&snapshot0),
                acked: Some(baseline),
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
        assert_eq!(rpr_enable_change_tracking(sender), RPR_OK);
        let (_, old_tick) = delta(sender, None);
        assert_eq!(rpr_disable_change_tracking(sender), RPR_OK);
        reject_delta(sender, old_tick.as_ptr(), old_tick.len(), RPR_UNSUPPORTED);
        // A restarted clock rejects the ticks issued before.
        assert_eq!(rpr_enable_change_tracking(sender), RPR_OK);
        reject_delta(
            sender,
            old_tick.as_ptr(),
            old_tick.len(),
            RPR_INVALID_ARGUMENT,
        );
        assert!(last_error().contains("baseline"));
        let (full, tick0) = delta(sender, None);

        // Ticks of another world.
        let other = rpr_new_world();
        assert_eq!(rpr_enable_change_tracking(other), RPR_OK);
        let (_, other_tick) = delta(other, None);
        reject_delta(
            sender,
            other_tick.as_ptr(),
            other_tick.len(),
            RPR_INVALID_ARGUMENT,
        );
        assert!(last_error().contains("baseline"));

        // Malformed tick bytes: a NULL tick with a length, header damage, truncation, trailing
        // bytes, or delta bytes in place of a tick.
        reject_delta(sender, null, 1, RPR_INVALID_ARGUMENT);
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
        reject_delta(
            sender,
            tick0.as_ptr(),
            256 * 1024 * 1024 + 1,
            RPR_INVALID_ARGUMENT,
        );
        reject_delta(std::ptr::null_mut(), null, 0, RPR_NULL_POINTER);

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
        assert!(rpr_apply_world_delta(receiver, full.as_ptr(), 256 * 1024 * 1024 + 1).is_null());
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
        reject_apply(receiver, &stale, RPR_INVALID_ARGUMENT);
        assert!(last_error().contains("baseline"));

        // A full delta resynchronizes it.
        let (full, tick) = delta(sender, None);
        assert_eq!(apply(receiver, &full), tick);
        assert!(snapshot(receiver) == snapshot(sender));

        for world in [sender, other, receiver] {
            assert_eq!(rpr_free_world(world), RPR_OK);
        }
    }
}
