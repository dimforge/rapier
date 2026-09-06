//! The pair entry points of the soft contact detection: the update of a soft-soft pair (both vertex passes, the edge pass, the volume patches) and of a soft-rigid pair (the predictive vertex contacts, the rigid patch).

use crate::alloc_prelude::*;

use crate::dynamics::{RigidBodySet, SoftMeshRef};
use crate::geometry::{Collider, ColliderHandle, ContactPair, PairContacts, RigidPairContacts};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::soft_contacts_volume::{detect_rigid_patch, detect_volume_patch};
use super::{
    SoftDetectionCtx, SoftVertexPass, SoftEdgePass, SoftPairContacts, body_frozen, detect_vertex_pass,
    detect_edges,
};

/// The edge-pass owner of a pair of meshes: the lower `(body handle, cluster, mesh)`, which is
/// the solver's awake order for two touching bodies (they share a substep group).
fn owns_edge_pass(a: SoftMeshRef, b: SoftMeshRef) -> bool {
    (a.body.0.into_raw_parts(), a.id.cluster, a.id.mesh)
        <= (b.body.0.into_raw_parts(), b.id.cluster, b.id.mesh)
}


pub(crate) fn update_pair_soft_soft(
    pair: &mut ContactPair,
    co1: &Collider,
    co2: &Collider,
    ctx: &SoftDetectionCtx,
) {
    let (h1, h2) = (pair.collider1, pair.collider2);
    if let (Some(r1), Some(r2)) = (co1.deformable_mesh_ref, co2.deformable_mesh_ref) {
        let PairContacts::Soft {
            touching,
            candidates,
        } = &mut pair.contacts
        else {
            return;
        };
        candidates.clear();
        detect_soft_pair(candidates, (h1, co1, r1), (h2, co2, r2), ctx);
        candidates.number_slots();
        *touching = candidates.is_touching();
    }
}

pub(crate) fn update_pair_soft_rigid(
    rigid: &mut RigidPairContacts,
    h1: ColliderHandle,
    h2: ColliderHandle,
    co1: &Collider,
    co2: &Collider,
    bodies: &RigidBodySet,
    ctx: &SoftDetectionCtx,
) {
}
    match (co1.deformable_mesh_ref, co2.deformable_mesh_ref) {
        (Some(r), None) | (None, Some(r)) => {
            let (soft_co, other_co, handle) = if co1.deformable_mesh_ref.is_some() {
                (co1, co2, h1)
            } else {
                (co2, co1, h2)
            };
            rigid.soft_patch = ctx
                .soft_bodies
                .get(r.body)
                .and_then(|sb| sb.mesh(r.id).map(|mesh| (sb, mesh)))
                .and_then(|(sb, mesh)| {
                    detect_rigid_patch((sb, mesh, handle, soft_co), other_co, bodies, ctx)
                })
                .map(Box::new);
        }
        _ => {}
}

/// The detection of a pair of two soft surfaces (see [`update_pair`]), into the cleared
/// candidates.
fn detect_soft_pair(
    soft: &mut SoftPairContacts,
    (h1, co1, r1): (ColliderHandle, &Collider, SoftMeshRef),
    (h2, co2, r2): (ColliderHandle, &Collider, SoftMeshRef),
    ctx: &SoftDetectionCtx,
) {
    let (Some(sb1), Some(sb2)) = (ctx.soft_bodies.get(r1.body), ctx.soft_bodies.get(r2.body))
    else {
        return;
    };
    let (Some(mesh1), Some(mesh2)) = (sb1.mesh(r1.id), sb2.mesh(r2.id)) else {
        return;
    };
    // Two meshes of one body: alternative descriptions of the same particles when they share a
    // cluster (a hull and a skin), so they never collide; across clusters they do, but only if
    // both opted in and their clusters share no particle.
    if r1.body == r2.body {
        let same_cluster = r1.id.cluster == r2.id.cluster;
        if same_cluster
            || !mesh1.self_contacts_enabled()
            || !mesh2.self_contacts_enabled()
            || !sb1.clusters_are_disjoint(r1.id.cluster, r2.id.cluster)
        {
            return;
        }
    }
    // Two frozen surfaces have no DOF between them.
    let (frozen1, frozen2) = (body_frozen(sb1), body_frozen(sb2));
    if frozen1 && frozen2 {
        return;
    }
    // Both vertex passes: each surface's owner consumes the other's vertices against it (and
    // the reverse pass too when the other body is not simulated).
    let mut pass1 = SoftVertexPass::default();
    let mut pass2 = SoftVertexPass::default();
    detect_vertex_pass(
        &mut pass1,
        (sb1, mesh1, h1, co1),
        (sb2, mesh2, h2, co2),
        None,
        ctx,
    );
    detect_vertex_pass(
        &mut pass2,
        (sb2, mesh2, h2, co2),
        (sb1, mesh1, h1, co1),
        None,
        ctx,
    );
    // The edge pass, oriented from its owner: a crossed closed-closed pair whose volume constraint
    // stands its edges down (see `overlap_edge_stand_down`) skips it once the owner's vertex pass
    // found the crossing; the solver applies the per-vertex part of that rule itself.
    let (own, other) = if owns_edge_pass(r1, r2) {
        ((sb1, mesh1, h1, co1), (sb2, mesh2, h2, co2))
    } else {
        ((sb2, mesh2, h2, co2), (sb1, mesh1, h1, co1))
    };
    let owner_pass = if own.2 == h1 { &pass1 } else { &pass2 };
    let recovery = &ctx.params.soft_bodies.recovery;
    let stood_down = cfg!(feature = "dim3")
        && mesh1.is_closed()
        && mesh2.is_closed()
        && recovery.overlap_constraints
        && recovery.overlap_edge_stand_down
        && !owner_pass.cross_tangled_elements.is_empty();
    let mut edges = SoftEdgePass::default();
    let has_edges = !stood_down && detect_edges(&mut edges, own, other, None, ctx);
    // The volume patches, from the vertex pass of the pair's lower surface.
    let (lower_pass, lower, higher) = if h1.into_raw_parts() < h2.into_raw_parts() {
        (&pass1, (sb1, mesh1, h1, co1), (sb2, mesh2, h2, co2))
    } else {
        (&pass2, (sb2, mesh2, h2, co2), (sb1, mesh1, h1, co1))
    };
    soft.volume = detect_volume_patch(lower_pass, lower, higher, ctx);
    soft.vertex_passes.push(pass1);
    soft.vertex_passes.push(pass2);
    soft.edges = has_edges.then_some(edges);
}
