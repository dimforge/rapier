//! The data types of the soft contact detection: the step context, the pair candidates (vertex, edge and volume), the pair payload and its impulse reports, the self-tangle signal.

use crate::alloc_prelude::*;
use core::ops::Range;

use crate::dynamics::{
    IntegrationParameters, RigidBodySet, SoftBody, SoftBodyHandle, SoftBodySet, SoftCollisionMesh,
};
use crate::geometry::{Collider, ColliderHandle, ColliderSet};
use crate::math::{DIM, Real, Rotation, Vector};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use super::VolumeBin;

/// The step-constant inputs of the soft contact detection.
pub(crate) struct SoftDetectionCtx<'a> {
    pub soft_bodies: &'a SoftBodySet,
    pub bodies: &'a RigidBodySet,
    pub colliders: &'a ColliderSet,
    pub params: &'a IntegrationParameters,
    /// The whole step's length (the closing-speed rule of the speculative reach).
    pub dt: Real,
}

impl SoftDetectionCtx<'_> {
    /// Whether two soft bodies are pieces of one torn body (one is the other's origin, or both
    /// share one): their rest shapes live in the same frame, and the contact skin between two of
    /// their features is capped by the gap those features had at rest (see `rest_gap_skins`).
    pub fn pieces_of_one_body(
        &self,
        (sb1, h1): (&SoftBody, SoftBodyHandle),
        (sb2, h2): (&SoftBody, SoftBodyHandle),
    ) -> bool {
        sb1.origin() == Some(h2)
            || sb2.origin() == Some(h1)
            || (sb1.origin().is_some() && sb1.origin() == sb2.origin())
    }

    /// The speculative motion margin of a soft body's deformable collider, held by its parent
    /// cluster proxy (zero for a rigid collider or without a parent).
    pub fn motion_margin(&self, co: &Collider) -> Real {
        co.parent()
            .and_then(|h| self.bodies.get(h))
            .map_or(0.0, |rb| co.soft_motion_margin(rb))
    }
}

/// A surface element within reach of a vertex of the other surface (see [`SoftVertexPass`]):
/// the closest point's barycentric weights on the element and the separation, skins deducted.
/// The contact hook may edit `dir` and `dist` or disable it; the solver reports its impulse.
#[derive(Copy, Clone, Debug)]
pub struct SoftVertexCandidate {
    /// The element of the surface side.
    pub element: u32,
    /// The closest point's barycentric weights on the element.
    pub weights: [Real; DIM],
    /// The force direction on the surface (from the vertex toward its closest point).
    pub dir: Vector,
    /// The separation, skins deducted (negative: penetrating).
    pub dist: Real,
    /// The element's outward normal (closed surfaces only).
    pub outward: Option<Vector>,
    /// The closest point lies inside the element (not on its boundary).
    pub interior: bool,
    /// Whether the solver builds a constraint for it (the hook clears it to drop the contact).
    pub enabled: bool,
    /// The two surfaces are pieces of one torn body, their separations measured from the
    /// features' rest gaps (see `rest_gap_skins`): the constraint holds a crack's rest geometry,
    /// and the along-normal volume-patch policy leaves it as it is.
    pub rest_gap: bool,
    /// The normal impulse the solver applied through it at the last step.
    pub impulse: Real,
    /// The world-space friction impulse the solver applied through it at the last step.
    pub tangent_impulse: Vector,
}

/// The candidates of one surface vertex against the other side's surface.
#[derive(Clone, Debug)]
pub struct SoftVertexHits {
    /// The vertex of the vertex side.
    pub vertex: u32,
    /// Range into [`SoftVertexPass::candidates`].
    pub candidates: Range<u32>,
    /// The vertex crossed the closed surface (seen from behind only, and inside by parity):
    /// the pair is recovery-owned.
    pub crossed: bool,
}

/// An edge-vs-edge contact candidate (see [`SoftEdgePass`]): the closest points of two
/// edges, interior to both. The contact-modification hook may edit `dir` and `dist`, or
/// disable the candidate; the soft-body solver reports the impulse it applied.
#[derive(Copy, Clone, Debug)]
pub struct SoftEdgeCandidate {
    /// The edge of the owner mesh.
    pub edge: u32,
    /// The edge of the other mesh.
    pub other_edge: u32,
    /// The closest point's barycentric weights on the owner's edge.
    pub bcoords: [Real; 2],
    /// The closest point's barycentric weights on the other edge.
    pub other_bcoords: [Real; 2],
    /// The closest point on the owner's edge (world space).
    pub point: Vector,
    /// The closest point on the other edge (world space).
    pub other_point: Vector,
    /// The force direction on the owner's edge (away from the other edge).
    pub dir: Vector,
    /// The separation, skins deducted (negative: penetrating).
    pub dist: Real,
    /// Whether the solver builds a constraint for it (the hook clears it to drop the contact).
    pub enabled: bool,
    /// The normal impulse the solver applied through it at the last step.
    pub impulse: Real,
    /// The world-space friction impulse the solver applied through it at the last step.
    pub tangent_impulse: Vector,
}

/// The vertex-vs-surface pass of one side of a pair of soft surfaces: the vertices of
/// `vertices_of` against the surface of `surface` (for a self pass, both are the same mesh).
#[derive(Clone, Default, Debug)]
pub struct SoftVertexPass {
    /// The collider whose surface the vertices are tested against.
    pub surface: ColliderHandle,
    /// The collider whose surface vertices are tested.
    pub vertices_of: ColliderHandle,
    /// Elements of the surface side taking part in a crossing between the two surfaces (empty
    /// while they do not cross): constraints touching them may only expel, never hold.
    pub cross_tangled_elements: Vec<bool>,
    /// Vertices of the vertex side taking part in a crossing, same rule as
    /// [`Self::cross_tangled_elements`].
    pub cross_tangled_vertices: Vec<bool>,
    /// Element-granularity crossing flags on the vertex-side mesh.
    pub cross_tangled_vb_elements: Vec<bool>,
    /// The recorded (surface-side, vertex-side) crossing element pairs.
    pub cross_pairs: Vec<(u32, u32)>,
    /// Per tested vertex of the vertex side, its candidates against the surface side.
    pub hits: Vec<SoftVertexHits>,
    /// The candidate pool the [`Self::hits`] ranges index into.
    pub candidates: Vec<SoftVertexCandidate>,
    /// The volume normals guiding the crossing repulsion (see `crossing_repulsion_guide`): per
    /// grid cell, its center and its normal from the surface side into the vertex side (for a
    /// self pass, the push direction of the fold's vertices, the facing surface's the opposite).
    pub repel_guides: Vec<(Vector, Vector)>,
    /// Per vertex of the vertex side, whether it lies inside the surface side's volume patch
    /// (see `overlap_patch_constraints`; empty under the `Keep` policy).
    pub patch_inside_vb: Vec<bool>,
    /// For a self pass, whether each vertex lies in its own body's self-intersection region
    /// (see `classify_inside_self`).
    pub repel_inside: Vec<bool>,
    /// The pair's contact slot of the first candidate (candidate `i` is slot `offset + i`,
    /// see [`SoftPairContacts::report_impulse`]); `u32::MAX` for a self pass.
    pub(crate) slot_offset: u32,
}

impl SoftVertexPass {
    pub(super) fn clear(&mut self) {
        self.cross_tangled_elements.clear();
        self.cross_tangled_vertices.clear();
        self.cross_tangled_vb_elements.clear();
        self.cross_pairs.clear();
        self.hits.clear();
        self.candidates.clear();
        self.repel_guides.clear();
        self.patch_inside_vb.clear();
        self.repel_inside.clear();
        self.slot_offset = u32::MAX;
    }

    /// The candidates of a vertex hit.
    pub fn candidates_of(&self, hit: &SoftVertexHits) -> &[SoftVertexCandidate] {
        &self.candidates[hit.candidates.start as usize..hit.candidates.end as usize]
    }
}

/// The edge-vs-edge candidates of a pair, owned by (oriented from) `own`.
#[derive(Clone, Default, Debug)]
pub struct SoftEdgePass {
    /// The collider whose edges own the pass (the candidates' `edge` and `dir`).
    pub own: ColliderHandle,
    /// The other collider.
    pub other: ColliderHandle,
    /// The candidates.
    pub candidates: Vec<SoftEdgeCandidate>,
    /// The two surfaces cross (found by the edge pass's own crossing test): the pair is
    /// recovery-owned.
    pub crossed: bool,
    /// The pair's contact slot of the first candidate (see
    /// [`SoftPairContacts::report_impulse`]); `u32::MAX` for a self pass.
    pub(crate) slot_offset: u32,
}

/// The volume patches of a crossed pair of closed soft surfaces (see `SoftOverlapConstraint`): the
/// bins' own side is the surface of `own` (the pair's lower collider).
#[derive(Clone, Debug)]
pub struct SoftVolumePatch {
    /// The collider of the bins' own side.
    pub own: ColliderHandle,
    /// The bins (the volume constraints).
    pub bins: Vec<VolumeBin>,
    /// The pair's contact slot of the first bin (see [`SoftPairContacts::report_impulse`]).
    pub(crate) slot_offset: u32,
}

/// The volume patch of a closed soft surface intruded by a rigid collider (see
/// `SoftOverlapConstraint`): the bins (own side only), the surface vertices' depths in the patch
/// (`NEG_INFINITY` outside, patch-constraint policies only), and the rigid side's CoM and rotation.
#[derive(Clone, Debug)]
pub struct SoftRigidPatch {
    pub bins: Vec<VolumeBin>,
    pub depth: Vec<Real>,
    pub com: Vector,
    pub rot: Rotation,
}

/// A predictive contact between a surface vertex of a soft-rigid pair and the convex rigid
/// collider: the support of the vertex itself, which an element's manifold does not give when
/// the rigid shape wraps the element (a single closest point per element).
#[derive(Copy, Clone, Debug)]
pub(crate) struct SoftRigidVertexContact {
    /// The vertex of the soft mesh.
    pub vertex: u32,
    /// A manifold of the pair on an element incident to the vertex (its material).
    pub manifold: u32,
    /// The closest point on the rigid collider, in that collider's frame.
    pub local_point: Vector,
    /// The force direction on the vertex (away from the rigid collider), in that collider's
    /// frame.
    pub local_dir: Vector,
    /// The separation, skins deducted (negative: penetrating).
    pub dist: Real,
    /// The normal impulse the solver applied through it at the last step.
    pub impulse: Real,
    /// The world-space friction impulse the solver applied through it at the last step.
    pub tangent_impulse: Vector,
}

/// The soft contacts of a soft-rigid pair beside its manifolds, rebuilt by every update.
#[derive(Clone, Default, Debug)]
pub(crate) struct SoftRigidContacts {
    /// The predictive vertex contacts, sorted by vertex.
    pub vertices: Vec<SoftRigidVertexContact>,
    /// The volume patch of a closed soft surface intruded by the rigid collider.
    pub patch: Option<SoftRigidPatch>,
}

/// The contacts of a pair of two soft surfaces: the candidates the narrow phase detected between
/// them, which the soft-body solver turns into constraints, rebuilt by every update of the pair.
/// Each slot (vertex-pass, edge, then volume-bin candidates) reports the solver's impulse.
#[derive(Clone, Default, Debug)]
pub struct SoftPairContacts {
    /// Both vertex passes: the vertices of the pair's second collider against the first's
    /// surface, then the reverse.
    pub vertex_passes: Vec<SoftVertexPass>,
    /// The edge-vs-edge pass (`None`: no edge contacts for this pair).
    pub edges: Option<SoftEdgePass>,
    /// The volume patches of a crossed pair of closed surfaces.
    pub volume: Option<SoftVolumePatch>,
}

/// The impulse a contact slot reported (see [`SoftPairContacts::impulses`]).
#[derive(Copy, Clone, Debug)]
pub struct SoftContactImpulse {
    /// The world-space normal, from the pair's first collider to the second.
    pub normal: Vector,
    /// The normal impulse applied along it.
    pub impulse: Real,
    /// The world-space friction impulse.
    pub tangent_impulse: Vector,
}

impl SoftPairContacts {
    pub(crate) fn clear(&mut self) {
        for pass in &mut self.vertex_passes {
            pass.clear();
        }
        self.vertex_passes.clear();
        if let Some(e) = &mut self.edges {
            e.candidates.clear();
            e.crossed = false;
            e.slot_offset = u32::MAX;
        }
        self.edges = None;
        self.volume = None;
    }

    /// The vertex pass whose surface side is `surface`.
    pub fn vertex_pass_on(&self, surface: ColliderHandle) -> Option<&SoftVertexPass> {
        self.vertex_passes
            .iter()
            .find(|pass| pass.surface == surface)
    }

    /// Disables every candidate (the contact-modification hook's way of dropping the pair's
    /// contacts: the pair still reports as touching).
    pub fn disable_all(&mut self) {
        for pass in &mut self.vertex_passes {
            for c in &mut pass.candidates {
                c.enabled = false;
            }
        }
        if let Some(e) = &mut self.edges {
            for c in &mut e.candidates {
                c.enabled = false;
            }
        }
        if let Some(v) = &mut self.volume {
            for b in &mut v.bins {
                b.enabled = false;
            }
        }
    }

    /// Whether some candidate exists: a feature of one surface within contact reach of the
    /// other, or a volume patch of the crossed pair.
    pub fn is_touching(&self) -> bool {
        self.vertex_passes.iter().any(|pass| !pass.hits.is_empty())
            || self
                .edges
                .as_ref()
                .is_some_and(|e| !e.candidates.is_empty())
            || self.volume.as_ref().is_some_and(|v| !v.bins.is_empty())
    }

    /// Numbers the contact slots: the vertex passes' candidates, the edge candidates, the bins.
    pub(super) fn number_slots(&mut self) {
        let mut n = 0u32;
        for pass in &mut self.vertex_passes {
            pass.slot_offset = n;
            n += pass.candidates.len() as u32;
        }
        if let Some(e) = &mut self.edges {
            e.slot_offset = n;
            n += e.candidates.len() as u32;
        }
        if let Some(v) = &mut self.volume {
            v.slot_offset = n;
        }
    }

    /// Records the impulse the solver applied through contact slot `slot` (a bin takes the
    /// normal impulse only).
    pub(crate) fn report_impulse(&mut self, slot: u32, impulse: Real, tangent_impulse: Vector) {
        for pass in &mut self.vertex_passes {
            if let Some(c) = slot
                .checked_sub(pass.slot_offset)
                .and_then(|i| pass.candidates.get_mut(i as usize))
            {
                c.impulse = impulse;
                c.tangent_impulse = tangent_impulse;
                return;
            }
        }
        if let Some(e) = &mut self.edges {
            if let Some(c) = slot
                .checked_sub(e.slot_offset)
                .and_then(|i| e.candidates.get_mut(i as usize))
            {
                c.impulse = impulse;
                c.tangent_impulse = tangent_impulse;
                return;
            }
        }
        if let Some(v) = &mut self.volume {
            if let Some(b) = slot
                .checked_sub(v.slot_offset)
                .and_then(|i| v.bins.get_mut(i as usize))
            {
                b.impulse = impulse;
            }
        }
    }

    /// The impulses the solver applied through the contacts at the last step, with their
    /// normals oriented from the pair's first collider (`collider1`) to the second.
    pub fn impulses(
        &self,
        collider1: ColliderHandle,
    ) -> impl Iterator<Item = SoftContactImpulse> + '_ {
        let vertices = self.vertex_passes.iter().flat_map(move |pass| {
            // `dir` is the force on the surface, from the vertex side into the surface side.
            let sign = if pass.surface == collider1 { -1.0 } else { 1.0 };
            pass.candidates.iter().map(move |c| SoftContactImpulse {
                normal: c.dir * sign,
                impulse: c.impulse,
                tangent_impulse: c.tangent_impulse,
            })
        });
        let edges = self.edges.iter().flat_map(move |e| {
            // `dir` is the force on the owner's edge, from the other side into the owner.
            let sign = if e.own == collider1 { -1.0 } else { 1.0 };
            e.candidates.iter().map(move |c| SoftContactImpulse {
                normal: c.dir * sign,
                impulse: c.impulse,
                tangent_impulse: c.tangent_impulse,
            })
        });
        let bins = self.volume.iter().flat_map(move |v| {
            // A bin's normal goes from its own side into the other.
            let sign = if v.own == collider1 { 1.0 } else { -1.0 };
            v.bins.iter().map(move |b| SoftContactImpulse {
                normal: b.normal * sign,
                impulse: b.impulse,
                tangent_impulse: Vector::ZERO,
            })
        });
        vertices.chain(edges).chain(bins)
    }
}

/// Whether a soft body has no free particle at all (fully pinned): it has no constraint of its own,
/// but its surface still holds the awake bodies meeting it.
pub(crate) fn body_frozen(sb: &SoftBody) -> bool {
    !sb.particles.iter().any(|p| p.inv_mass > 0.0)
}

/// The self-tangle signal of a mesh, for a self pass: the features whose self contacts stand
/// down, and the recorded self-crossings.
pub(crate) struct SelfTangles<'a> {
    pub tangled_elements: &'a [bool],
    pub tangled_vertices: &'a [bool],
    pub crossings: &'a [(u32, u32)],
}

pub(super) type Side<'a> = (
    &'a SoftBody,
    &'a SoftCollisionMesh,
    ColliderHandle,
    &'a Collider,
);

/// The contact skins between two features of two pieces of one torn body: the full `skins`, capped
/// by the features' rest-shape distance. Crack faces rest at distance zero (a crack snapped to the
/// cells' facets interlocks the sides), so features keep their rest gap and collide from there.
pub(crate) fn rest_gap_skins(skins: Real, rest_gap: Real) -> Real {
    skins.min(rest_gap.max(0.0))
}
