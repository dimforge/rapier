#![allow(missing_docs)]
use std::collections::HashMap;

/// Lightweight cache for persistent contact impulses.
#[derive(Default)]
pub struct ContactPersistenceCache {
    impulses: HashMap<u128, (f32, f32, u64)>,
    max_entries: usize,
}

impl ContactPersistenceCache {
    pub fn insert(&mut self, id: u128, normal: f32, tangent: f32, step: u64) {
        if self.max_entries != 0
            && self.impulses.len() >= self.max_entries
            && !self.impulses.contains_key(&id)
        {
            if let Some(oldest) = self
                .impulses
                .iter()
                .min_by_key(|(_, v)| v.2)
                .map(|(k, _)| *k)
            {
                self.impulses.remove(&oldest);
            }
        }
        self.impulses.insert(id, (normal, tangent, step));
    }

    pub fn set_max_entries(&mut self, max_entries: usize) {
        self.max_entries = max_entries;
        while max_entries != 0 && self.impulses.len() > max_entries {
            if let Some(oldest) = self
                .impulses
                .iter()
                .min_by_key(|(_, v)| v.2)
                .map(|(k, _)| *k)
            {
                self.impulses.remove(&oldest);
            }
        }
    }

    pub fn get(&self, id: u128) -> Option<(f32, f32)> {
        self.impulses.get(&id).map(|v| (v.0, v.1))
    }

    /// Removes one cached contact. Callers can enumerate their contact IDs to
    /// invalidate all points belonging to a pair without relying on lossy hashing.
    pub fn remove_contact(&mut self, id: u128) {
        self.impulses.remove(&id);
    }

    pub fn clear(&mut self) {
        self.impulses.clear();
    }
    pub fn len(&self) -> usize {
        self.impulses.len()
    }

    /// Removes entries not touched within `max_age` simulation steps.
    pub fn retain_recent(&mut self, step: u64, max_age: u64) {
        self.impulses
            .retain(|_, (_, _, last)| step.saturating_sub(*last) <= max_age);
    }

    /// Scales all cached impulses, typically when the integration timestep changes.
    pub fn scale_impulses(&mut self, factor: f32) {
        if !factor.is_finite() || factor < 0.0 {
            self.clear();
            return;
        }
        for (normal, tangent, _) in self.impulses.values_mut() {
            *normal *= factor;
            *tangent *= factor;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::ContactPersistenceCache;

    #[test]
    fn cache_lifecycle_and_expiration() {
        let mut cache = ContactPersistenceCache::default();
        cache.insert(1, 2.0, 3.0, 10);
        cache.insert(2, 4.0, 5.0, 1);
        assert_eq!(cache.get(1), Some((2.0, 3.0)));
        cache.remove_contact(1);
        assert_eq!(cache.get(1), None);
        cache.retain_recent(10, 2);
        assert_eq!(cache.len(), 0);
    }

    #[test]
    fn cache_scales_and_rejects_invalid_factor() {
        let mut cache = ContactPersistenceCache::default();
        cache.insert(7, 2.0, 4.0, 1);
        cache.scale_impulses(0.5);
        assert_eq!(cache.get(7), Some((1.0, 2.0)));
        cache.scale_impulses(f32::NAN);
        assert_eq!(cache.len(), 0);
    }
}
