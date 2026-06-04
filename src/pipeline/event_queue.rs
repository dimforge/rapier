use crate::utils;
use crate::utils::FlatHandle;
use rapier::geometry::{CollisionEvent, ContactForceEvent};
use rapier::pipeline::ChannelEventCollector;
use std::sync::mpsc::Receiver;
use wasm_bindgen::prelude::*;

/// A structure responsible for collecting events generated
/// by the physics engine.
#[wasm_bindgen]
pub struct RawEventQueue {
    pub(crate) collector: ChannelEventCollector,
    collision_events: Receiver<CollisionEvent>,
    contact_force_events: Receiver<ContactForceEvent>,
    pub(crate) auto_drain: bool,
}

#[wasm_bindgen]
pub struct RawContactForceEvent(ContactForceEvent);

#[wasm_bindgen]
impl RawContactForceEvent {
    /// The first collider involved in the contact.
    pub fn collider1(&self) -> FlatHandle {
        crate::utils::flat_handle(self.0.collider1.0)
    }

    /// The second collider involved in the contact.
    pub fn collider2(&self) -> FlatHandle {
        crate::utils::flat_handle(self.0.collider2.0)
    }

    /// The sum of all the forces between the two colliders.
    pub fn total_force(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.total_force;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        #[cfg(feature = "dim3")]
        scratch_buffer.set_index(2, u.z);
    }

    /// The sum of the magnitudes of each force between the two colliders.
    ///
    /// Note that this is **not** the same as the magnitude of `self.total_force`.
    /// Here we are summing the magnitude of all the forces, instead of taking
    /// the magnitude of their sum.
    pub fn total_force_magnitude(&self) -> f32 {
        self.0.total_force_magnitude
    }

    /// The world-space (unit) direction of the force with strongest magnitude.
    pub fn max_force_direction(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.max_force_direction;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        #[cfg(feature = "dim3")]
        scratch_buffer.set_index(2, u.z);
    }

    /// The magnitude of the largest force at a contact point of this contact pair.
    pub fn max_force_magnitude(&self) -> f32 {
        self.0.max_force_magnitude
    }
}

// #[wasm_bindgen]
// /// The proximity state of a sensor collider and another collider.
// pub enum RawIntersection {
//     /// The sensor is intersecting the other collider.
//     Intersecting = 0,
//     /// The sensor is within tolerance margin of the other collider.
//     WithinMargin = 1,
//     /// The sensor is disjoint from the other collider.
//     Disjoint = 2,
// }

#[wasm_bindgen]
impl RawEventQueue {
    /// Creates a new event collector.
    ///
    /// # Parameters
    /// - `autoDrain`: setting this to `true` is strongly recommended. If true, the collector will
    /// be automatically drained before each `world.step(collector)`. If false, the collector will
    /// keep all events in memory unless it is manually drained/cleared; this may lead to unbounded use of
    /// RAM if no drain is performed.
    #[wasm_bindgen(constructor)]
    pub fn new(autoDrain: bool) -> Self {
        let collision_channel = std::sync::mpsc::channel();
        let contact_force_channel = std::sync::mpsc::channel();
        let collector = ChannelEventCollector::new(collision_channel.0, contact_force_channel.0);

        Self {
            collector,
            collision_events: collision_channel.1,
            contact_force_events: contact_force_channel.1,
            auto_drain: autoDrain,
        }
    }

    /// Applies the given javascript closure on each collision event of this collector, then clear
    /// the internal collision event buffer.
    ///
    /// # Parameters
    /// - `f(handle1, handle2, started)`:  JavaScript closure applied to each collision event. The
    /// closure should take three arguments: two integers representing the handles of the colliders
    /// involved in the collision, and a boolean indicating if the collision started (true) or stopped
    /// (false).
    pub fn drainCollisionEvents(&mut self, f: &js_sys::Function) {
        let this = JsValue::null();
        while let Ok(event) = self.collision_events.try_recv() {
            match event {
                CollisionEvent::Started(co1, co2, _) => {
                    let h1 = utils::flat_handle(co1.0);
                    let h2 = utils::flat_handle(co2.0);
                    let _ = f.call3(
                        &this,
                        &JsValue::from(h1),
                        &JsValue::from(h2),
                        &JsValue::from_bool(true),
                    );
                }
                CollisionEvent::Stopped(co1, co2, _) => {
                    let h1 = utils::flat_handle(co1.0);
                    let h2 = utils::flat_handle(co2.0);
                    let _ = f.call3(
                        &this,
                        &JsValue::from(h1),
                        &JsValue::from(h2),
                        &JsValue::from_bool(false),
                    );
                }
            }
        }
    }

    pub fn drainContactForceEvents(&mut self, f: &js_sys::Function) {
        let this = JsValue::null();
        while let Ok(event) = self.contact_force_events.try_recv() {
            let _ = f.call1(&this, &JsValue::from(RawContactForceEvent(event)));
        }
    }

    /// Removes all events contained by this collector.
    pub fn clear(&self) {
        while let Ok(_) = self.collision_events.try_recv() {}
    }
}
