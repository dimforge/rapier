//! The narrow-phase driver of the self contact detection: runs the self pass of every self-colliding mesh of the awake soft bodies and keeps its results per collider.

use crate::alloc_prelude::*;

use crate::dynamics::{RigidBodySet, SoftMeshRef};
use crate::geometry::ColliderHandle;
use crate::geometry::narrow_phase::NarrowPhase;

use super::soft_self_contacts::update_self;
use super::{SoftDetectionCtx, SoftSelfContacts, body_frozen};

impl NarrowPhase {
    /// The self contact detection of the soft collision mesh held by `collider`, as of the
    /// last narrow-phase update that ran it (`None` for a sleeping body, or a mesh without
    /// self contacts).
    pub(crate) fn soft_self_contacts(&self, collider: ColliderHandle) -> Option<&SoftSelfContacts> {
        self.soft_self.get(&collider)
    }

    /// Runs the self contact detection of every self-colliding mesh of the awake soft bodies
    /// (after the pair transitions: a body woken by a new contact is included), in parallel
    /// under the `parallel` feature.
    pub(crate) fn update_soft_self_contacts(
        &mut self,
        ctx: &SoftDetectionCtx,
        bodies: &RigidBodySet,
    ) {
        let mut jobs: Vec<(ColliderHandle, SoftMeshRef, SoftSelfContacts)> = Vec::new();
        for (handle, sb) in ctx.soft_bodies.iter() {
            let awake = bodies
                .get(sb.root_body())
                .is_some_and(|rb| !rb.is_sleeping() && rb.is_enabled());
            if !awake || body_frozen(sb) {
                continue;
            }
            for mesh in sb.meshes() {
                if !mesh.collision_enabled() || !mesh.self_contacts_enabled() {
                    continue;
                }
                let collider = mesh.collider();
                // The `enhanced-determinism` map is an `IndexMap`, whose `remove` is a
                // deprecated alias of `swap_remove`; hashbrown's only has `remove`.
                #[cfg(feature = "enhanced-determinism")]
                let payload = self.soft_self.swap_remove(&collider).unwrap_or_default();
                #[cfg(not(feature = "enhanced-determinism"))]
                let payload = self.soft_self.remove(&collider).unwrap_or_default();
                jobs.push((
                    collider,
                    SoftMeshRef {
                        body: handle,
                        id: mesh.id(),
                    },
                    payload,
                ));
            }
        }
        // The entries left are those of the sleeping bodies (kept for their wake-up) and of
        // the removed meshes (dropped).
        let colliders = ctx.colliders;
        self.soft_self.retain(|h, _| {
            colliders
                .get(*h)
                .is_some_and(|c| c.deformable_mesh_ref.is_some())
        });
        let run = |(collider, mesh_ref, payload): &mut (
            ColliderHandle,
            SoftMeshRef,
            SoftSelfContacts,
        )| {
            let (Some(sb), Some(co)) =
                (ctx.soft_bodies.get(mesh_ref.body), colliders.get(*collider))
            else {
                return;
            };
            let Some(mesh) = sb.mesh(mesh_ref.id) else {
                return;
            };
            update_self(payload, sb, mesh, *collider, co, ctx);
        };
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            jobs.par_iter_mut().for_each(run);
        }
        #[cfg(not(feature = "parallel"))]
        jobs.iter_mut().for_each(run);
        for (collider, _, payload) in jobs {
            self.soft_self.insert(collider, payload);
        }
    }
}
