//! Error handling for rapier.
//!
//! A number of checks are in place to catch inconsistencies as soon as they occur,
//! while it's best to make sure none of these errors occur, some may be recoverable
//! through removing the root cause (incorrect shape or position for example).
//!
//! Setting [`GLOBAL_ERROR_HANDLER`] can help you as the end user to react to those errors.
//!
//! This module is typically NOT used by library authors, to allow end user to customize their own error handler.
//!
//! Its default behaviour is to [`panic!`].

use std::sync::OnceLock;

use log::warn;

/// Possible errors to handle through [`default_error_handler`].
#[derive(Debug, PartialEq, Eq, Clone)]
pub enum Error {
    /// Error detected during `SAPAxis` `batch_insert`.
    SapAxisInsert(String),
    /// Error detected when trying to access a point key.
    PointKey(String),
}

/// A global error handler. This can be set at startup, as long as it is set before
/// any uses.
///
/// # Example
///
/// ```
/// use crate::error::{GLOBAL_ERROR_HANDLER, warn};
/// GLOBAL_ERROR_HANDLER.set(warn).expect("The error handler can only be set once, globally.");
/// ```
pub static GLOBAL_ERROR_HANDLER: OnceLock<Box<dyn Fn(Error) + Sync + Send>> = OnceLock::new();

/// The default error handler. This defaults to [`panic()`].
#[inline]
pub fn default_error_handler() -> &'static dyn Fn(Error) {
    GLOBAL_ERROR_HANDLER.get_or_init(|| Box::new(panic))
}

/// Error handler that panics with the error.
#[track_caller]
#[inline(always)]
pub fn panic(error: Error) {
    panic!("Encountered an error:\n{:?}", error);
}

/// Error handler that logs the error at the `warn` level.
#[track_caller]
#[inline]
pub fn warn(error: Error) {
    warn!("Encountered an error:\n{:?}", error);
}

#[cfg(all(feature = "dim3", feature = "f32"))]
#[cfg(test)]
mod test {
    use core::f32;

    use na::{Isometry, Translation3};

    use crate::error_handler::GLOBAL_ERROR_HANDLER;
    use crate::prelude::*;
    use std::sync::mpsc::{self, Receiver, Sender};

    #[test]
    fn error_handling() {
        use log::error;

        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut pipeline = PhysicsPipeline::new();
        let mut bf = BroadPhaseMultiSap::new();
        let mut nf = NarrowPhase::new();
        let mut bodies = RigidBodySet::new();
        let mut islands = IslandManager::new();

        let rb = RigidBodyBuilder::fixed()
            .position(Isometry::from_parts(
                Translation3::new(f32::MIN, f32::MIN, f32::MIN),
                Rotation::identity(),
            ))
            .build();
        let h1 = bodies.insert(rb.clone());
        let co = ColliderBuilder::ball(10.0).build();
        colliders.insert_with_parent(co.clone(), h1, &mut bodies);

        let (tx, rx): (Sender<i32>, Receiver<i32>) = mpsc::channel();
        // Set error handling
        if GLOBAL_ERROR_HANDLER
            .set(Box::new(move |error| {
                println!("error: {:?}", error);
                // TODO: add more context:
                // - offending rigidbody ?
                // - backtrace ?
                assert!(tx.send(1).is_ok());
            }))
            .is_err()
        {
            error!("The error handler can only be set once, globally.");
        }

        pipeline.step(
            &Vector::zeros(),
            &IntegrationParameters::default(),
            &mut islands,
            &mut bf,
            &mut nf,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut CCDSolver::new(),
            None,
            &(),
            &(),
        );
        assert!(rx.try_recv() == Ok(1));
        assert!(rx.try_recv().is_ok());
        assert!(rx.try_recv().is_ok());
        assert!(rx.try_recv().is_ok());
        assert!(rx.try_recv().is_ok());
        assert!(rx.try_recv().is_ok());
        assert!(rx.try_recv().is_err());
    }
}
