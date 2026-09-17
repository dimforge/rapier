//! Per-body contact constraints: the surface's narrow-phase manifold constraints (rigid bodies and particle balls), then the dispatch to the vertex and edge passes.

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

use crate::alloc_prelude::*;

use super::soft_contact_assembly_workspace::SOURCE_RIGID_VERTEX;
use super::{AssemblyCtx, BodyContacts, mesh_ref};
use super::super::soft_constraints_set::{SoftConstraintsSet, barycentric_weights};
use super::super::soft_contact::CONTACT_ANCHORS;
use super::super::soft_contact::{SoftContact, SoftContactSource};
use crate::dynamics::soft_body::SoftPatchConstraints;
use crate::dynamics::SoftBodyHandle;
use crate::geometry::ColliderHandle;
use crate::geometry::soft_contacts::SoftEdgePass;
use crate::math::{DIM, Real, Vector};
use crate::utils::{CrossProduct, DotProduct};

impl SoftConstraintsSet {
    /// The contact constraints of awake body `ai`: its surface's narrow-phase pairs, then the
    /// vertex-vs-surface and edge-vs-edge constraints against the other soft surfaces met and against
    /// itself (self contacts).
    pub(super) fn assemble_body_contacts(&self, ai: usize, ctx: &AssemblyCtx, out: &mut BodyContacts) {
        use crate::geometry::contact_pair::NEW_CONTACT_BIT;
        let AssemblyCtx {
            island_id,
            params,
            narrow_phase,
            colliders,
            bodies,
            soft_bodies,
            dyn_soft: (dyn_erp, dyn_cfm),
            static_soft: (static_erp, static_cfm),
            ..
        } = *ctx;
        {
            let awake = &self.awake[ai];
            // SAFETY: read-only access during assembly.
            let sb = unsafe { &*awake.ptr };
            for mesh in sb.meshes().filter(|mesh| mesh.collision_enabled()) {
                let surface_handle = mesh.collider();
                out.begin_mesh(mesh.id());
                let self_frozen = awake.frozen;
                let slots = &self.slots[awake.slot_start..awake.slot_start + awake.num_particles];
                // The other soft surfaces whose edge-vs-edge constraints this body owns: (awake
                // index, handle, surface collider, the pair's edge candidates), assembled
                // once the pairs are done.
                let mut edge_tasks: Vec<(
                    Option<usize>,
                    SoftBodyHandle,
                    ColliderHandle,
                    Option<&SoftEdgePass>,
                )> = Vec::new();

                if params.soft_bodies.recovery.overlap_constraints
                    && params.soft_bodies.recovery.overlap_rigid
                    && !self_frozen
                {
                    self.assemble_rigid_overlap(ai, ctx, mesh, surface_handle, out);
                }
                for pair in narrow_phase.contact_pairs_with(surface_handle) {
                    let surface_first = pair.collider1 == surface_handle;
                    let other_handle = if surface_first {
                        pair.collider2
                    } else {
                        pair.collider1
                    };
                    let (Some(surface_co), Some(other_co)) =
                        (colliders.get(surface_handle), colliders.get(other_handle))
                    else {
                        continue;
                    };
                    // Another soft body: vertex-vs-surface constraints (each owner, in pair order)
                    // and edge-vs-edge ones (lower awake index, after its pairs) from the detected
                    // candidates; a pinned other is frozen, a non-awake one gets both passes here.
                    if other_co.deformable_mesh_ref.is_some() {
                        // The pair reports a contact while the two surfaces' bounds overlap
                        // (see the narrow phase): nothing to do otherwise.
                        if !pair.has_any_active_contact() {
                            continue;
                        }
                        let Some(detected) = pair.soft() else {
                            continue;
                        };
                        let other_ref = mesh_ref(other_co);
                        let other_sb_handle = other_ref.body;
                        let other_ai = self.edge_workspace.awake_of.get(&other_sb_handle).copied();
                        let Some(other_sb) = soft_bodies.get(other_sb_handle) else {
                            continue;
                        };
                        // Two meshes of one body never collide within a cluster (a hull and a skin
                        // describe the same particles); across clusters they do, if both opted in
                        // and their clusters share no particle.
                        if other_sb_handle == awake.handle {
                            let Some(other_mesh) = other_sb.mesh(other_ref.id) else {
                                continue;
                            };
                            let same_cluster = other_ref.id.cluster == mesh.id().cluster;
                            if same_cluster
                                || !mesh.self_contacts_enabled()
                                || !other_mesh.self_contacts_enabled()
                                || !sb.clusters_are_disjoint(mesh.id().cluster, other_ref.id.cluster)
                            {
                                continue;
                            }
                        }
                        let frozen = match other_ai {
                            Some(bi) => self.awake[bi].frozen,
                            None => other_sb.particles.iter().all(|p| p.inv_mass == 0.0),
                        };
                        // Two frozen surfaces have no DOF between them.
                        if self_frozen && frozen {
                            continue;
                        }
                        let both = match other_ai {
                            Some(_) => false,
                            None if frozen => true,
                            None => continue,
                        };
                        let flips: &[bool] = if both { &[false, true] } else { &[false] };
                        for &flipped in flips {
                            // The vertex pass whose surface side this pass assembles.
                            let surface = if flipped { other_handle } else { surface_handle };
                            let Some(vertex_pass) = detected.vertex_pass_on(surface) else {
                                continue;
                            };
                            self.assemble_vertex_contacts(
                                ctx,
                                ai,
                                other_ai,
                                other_sb,
                                (surface_handle, surface_co),
                                (other_handle, other_co),
                                flipped,
                                vertex_pass,
                                detected.volume.as_ref(),
                                &[],
                                out,
                            );
                        }
                        // The edge-vs-edge pass of a pair is owned by its lower
                        // (awake index, cluster, mesh): exactly one side assembles it.
                        let own_key = (ai, mesh.id().cluster, mesh.id().mesh);
                        let other_key = (
                            other_ai.unwrap_or(usize::MAX),
                            other_ref.id.cluster,
                            other_ref.id.mesh,
                        );
                        if own_key <= other_key {
                            edge_tasks.push((
                                other_ai,
                                other_sb_handle,
                                other_handle,
                                detected.edges.as_ref(),
                            ));
                        }
                        continue;
                    }
                    if !pair.has_any_active_contact() {
                        continue;
                    }
                    let surface_rb = surface_co.parent().and_then(|h| bodies.get(h));
                    let other_rb = other_co.parent().and_then(|h| bodies.get(h));
                    // The other side's soft body and particle when it is a particle ball, and
                    // that body's awake index (for the chunking).
                    let other_body = u32::MAX;
                    // Self contact: the other side is one of this soft body's own particles.
                    let self_particle = None;
                    let com_pose = |rb: &crate::dynamics::RigidBody| {
                        rb.pos
                            .position
                            .prepend_translation(rb.mprops.local_mprops.local_com)
                    };
                    // The other side's solver body (its contact point is tracked in that body's
                    // CoM frame), if it is simulated in this island: a rigid body, or the
                    // particle's solver body (its frame is at the particle).
                    let (other_slot, other_com_pose) = match other_rb
                    {
                        Some(rb)
                            if rb.ids.active_island_id == island_id as u32
                                && !rb.is_sleeping()
                                && rb.is_enabled()
                                && rb.is_dynamic_or_kinematic() =>
                        {
                            (rb.ids.active_set_id, Some(com_pose(rb)))
                        }
                        _ => (u32::MAX, None),
                    };
                    // A frozen surface against a fixed or sleeping body: no DOF on either side.
                    if self_frozen && other_slot == u32::MAX {
                        continue;
                    }
                    let (erp_inv_dt, cfm_factor) = if other_slot == u32::MAX {
                        (static_erp, static_cfm)
                    } else {
                        (dyn_erp, dyn_cfm)
                    };
                    // Adjacent elements of a flat patch all report the shared vertex as a
                    // contact point: keep one constraint per (particle, pair) for those (the deepest),
                    // interior points are kept as they are.
                    let mut vertex_constraints: parry::utils::hashmap::HashMap<u32, usize> =
                        Default::default();
                    // The vertices a manifold constraint holds at (or next to) their position: they need
                    // no predictive vertex constraint (see below).
                    let mut held_vertices: Vec<u32> = Vec::new();
                    // A particle at a rigid feature (a box edge under a cloth) collects up to ~16
                    // speculative constraints whose relax-pass slack sums to a phantom velocity:
                    // those of one feature dominated by the same particle merge into the deepest.
                    let mut feature_constraints: parry::utils::hashmap::HashMap<(u32, u32), usize> =
                        Default::default();
                    // The narrow phase localized the anchors in the parent bodies' CoM frames,
                    // except on the dominant side of a pair (world anchor, like a fixed body):
                    // the same rule (`relative_dominance`) recovers the world anchors.
                    let (rb1, rb2) = if surface_first {
                        (surface_rb, other_rb)
                    } else {
                        (other_rb, surface_rb)
                    };
                    let anchor_poses = |manifold: &crate::geometry::ContactManifold| {
                        let rel_dom = manifold.data.relative_dominance;
                        (
                            rb1.filter(|_| rel_dom <= 0).map(com_pose),
                            rb2.filter(|_| rel_dom >= 0).map(com_pose),
                        )
                    };
                    let localized = |anchor: Vector, pose: &Option<crate::math::Pose>| match pose {
                        Some(pose) => pose * anchor,
                        None => anchor,
                    };
                    let manifold_element = |manifold: &crate::geometry::ContactManifold| {
                        let element_id = if surface_first {
                            manifold.subshape1
                        } else {
                            manifold.subshape2
                        } as usize;
                        (element_id < mesh.indices().len())
                            .then(|| (element_id, mesh.element(element_id)))
                    };
                    // An element's unused slots (a wire's third one) stay at the neutral
                    // values the constraints already use: no vertex, no weight.
                    let contact_geometry =
                        |element: &[u32],
                         manifold: &crate::geometry::ContactManifold,
                         sc: &crate::geometry::SolverContact|
                         -> ([Vector; DIM], [Real; DIM], Vector) {
                            let (pose1, pose2) = anchor_poses(manifold);
                            let (surface_anchor, other_anchor) = if surface_first {
                                (localized(sc.anchor1, &pose1), localized(sc.anchor2, &pose2))
                            } else {
                                (localized(sc.anchor2, &pose2), localized(sc.anchor1, &pose1))
                            };
                            let positions: [Vector; DIM] = core::array::from_fn(|k| {
                                element
                                    .get(k)
                                    .map_or(Vector::ZERO, |v| mesh.vertex(sb, *v as usize))
                            });
                            let weights =
                                barycentric_weights(&positions[..element.len()], surface_anchor);
                            (positions, weights, other_anchor)
                        };
                    // Contacts whose point lies inside the element (vertex/edge contacts have
                    // normals of their own).
                    let is_interior = |element: &[u32], weights: &[Real; DIM]| {
                        weights[..element.len()].iter().all(|w| *w > 0.02)
                    };
                    // Normal approach speed of the other body (rigid ones only: soft particles
                    // count through their own body's speeds) toward the element's point.
                    let rigid_other = other_rb.filter(|rb| rb.soft_body().is_none());
                    let approach_speed = |element: &[u32],
                                          weights: &[Real; DIM],
                                          anchor: Vector,
                                          dir: Vector| {
                        let Some(rb) = rigid_other else {
                            return 0.0;
                        };
                        let v_other =
                            rb.linvel() + rb.angvel().gcross(anchor - rb.center_of_mass());
                        let mut v_surface = Vector::ZERO;
                        for (k, v) in element.iter().enumerate() {
                            v_surface += mesh.vertex_velocity(sb, *v as usize) * weights[k];
                        }
                        (v_other - v_surface).gdot(dir).max(0.0)
                    };

                    // A closed surface's reversed interior contacts (an intruder seen from inside,
                    // a fold's far layer) are dropped: the volume constraints and the elasticity
                    // resolve them. The rule reads the cells' winding, so a skin collides plainly.
                    let skinned = mesh.is_skinned();
                    let closed = mesh.is_closed() && self_particle.is_none() && !skinned;

                    // The features a volume constraint acts on (see `overlap_patch_constraints`).
                    let patch_policy = params.soft_bodies.recovery.overlap_patch_constraints;
                    let rigid_patch: Option<(Vec<Real>, Vec<(Vector, Vector)>)> =
                        (patch_policy != SoftPatchConstraints::Keep)
                            .then(|| {
                                out.rigid_patches
                                    .iter()
                                    .find(|p| p.0 == other_handle)
                                    .map(|p| (p.1.clone(), p.2.clone()))
                            })
                            .flatten();
                    for (mi, manifold) in pair.manifolds().iter().enumerate() {
                        let Some((element_id, element)) = manifold_element(manifold) else {
                            continue;
                        };
                        if let Some(p) = self_particle {
                            if mesh.self_contact_excluded(sb, p, element, surface_co.contact_skin()) {
                                continue;
                            }
                        }
                        let in_patch = rigid_patch.as_ref().is_some_and(|(depths, _)| {
                            element.iter().any(|&v| {
                                depths.get(v as usize).copied().unwrap_or(Real::NEG_INFINITY)
                                    > Real::NEG_INFINITY
                            })
                        });
                        // Force direction on the surface (the normal points from collider 1 to 2).
                        let normal = manifold.data.normal;
                        let dir = if surface_first { -normal } else { normal };
                        // Reversed interior contacts of a closed surface: expelled through the
                        // far side, or dropped (see above).
                        let reversed = if closed {
                            mesh.element_outward_normal(sb, element_id)
                                .and_then(|n| n.try_normalize())
                                .filter(|n| dir.gdot(*n) > 0.5)
                        } else {
                            None
                        };
                        // An element contact's point: weights, point, rigid anchor, separation,
                        // warm-start share. A segment's interior contact on a smooth rigid surface
                        // splits into one per endpoint, in the tangent plane, so it cannot rock.
                        let contact_points = |sc: &crate::geometry::SolverContact| {
                            let (positions, weights, other_anchor) =
                                contact_geometry(element, manifold, sc);
                            // The constraint tracks the barycentric point of the element: its reference
                            // must be that point too (the anchor is offset from the element by
                            // the contact skins when the surface is the pair's first collider).
                            let mut surface_point0 = Vector::ZERO;
                            for k in 0..DIM {
                                surface_point0 += positions[k] * weights[k];
                            }
                            let split = element.len() == 2
                                && weights[0] > 0.02
                                && weights[1] > 0.02
                                && (positions[1] - positions[0]).gdot(dir).abs()
                                    < 0.5 * (positions[1] - positions[0]).length();
                            let mut points = [(
                                positions,
                                weights,
                                surface_point0,
                                other_anchor,
                                sc.dist,
                                1.0,
                            ); 2];
                            if split {
                                for (k, point) in points.iter_mut().enumerate() {
                                    let mut w = [0.0; DIM];
                                    w[k] = 1.0;
                                    let offset = positions[k] - surface_point0;
                                    let along = offset.gdot(dir);
                                    *point = (
                                        positions,
                                        w,
                                        positions[k],
                                        other_anchor + offset - dir * along,
                                        sc.dist + along,
                                        0.5,
                                    );
                                }
                            }
                            points
                                .into_iter()
                                .take(if split { 2 } else { 1 })
                                .map(move |p| (p, split))
                        };
                        for (sc, (point, split_endpoints)) in manifold
                            .data
                            .solver_contacts
                            .iter()
                            .flat_map(|sc| contact_points(sc).map(move |p| (sc, p)))
                        {
                            let point_id = (sc.contact_id[0] & !NEW_CONTACT_BIT) as usize;
                            let (positions, weights, surface_point0, other_anchor, sc_dist, warm_share) = point;
                            // A constraint disagreeing with the volume constraint (see
                            // `overlap_patch_constraints`) pushes the surface into the rigid body
                            // along the nearest cell's normal: stood down, or bent along it.
                            let disagreeing = if in_patch {
                                rigid_patch.as_ref().and_then(|(depths, cells)| {
                                    let n = cells
                                        .iter()
                                        .min_by(|a, b| {
                                            let da = (a.0 - surface_point0).length_squared();
                                            let db = (b.0 - surface_point0).length_squared();
                                            da.partial_cmp(&db)
                                                .unwrap_or(core::cmp::Ordering::Equal)
                                        })
                                        .map(|c| c.1)?;
                                    if dir.gdot(n) <= 0.0 {
                                        return None;
                                    }
                                    let mut depth = 0.0;
                                    for k in 0..DIM {
                                        if let Some(&v) = element.get(k) {
                                            depth += weights[k]
                                                * depths[v as usize].max(0.0);
                                        }
                                    }
                                    Some((-n, -depth, false))
                                })
                            } else {
                                None
                            };
                            if disagreeing.is_some() && patch_policy == SoftPatchConstraints::StandDown
                            {
                                continue;
                            }
                            let along_normal = disagreeing
                                .filter(|_| patch_policy == SoftPatchConstraints::AlongNormal);
                            let (dir, dist, _expelling) = match (along_normal, reversed) {
                                (Some(constraint), _) => constraint,
                                (None, Some(_)) if is_interior(element, &weights) => continue,
                                _ => (dir, sc_dist, false),
                            };
                            // The point the constraint tracks, expressed in the particles it acts through.
                            let (anchors, anchor_weights, tracked) =
                                mesh.contact_anchors(sb, element, &weights, surface_point0);
                            let surface_point0 = tracked;
                            let particles: [u32; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                                if anchors[k] == u32::MAX {
                                    u32::MAX
                                } else {
                                    slots[anchors[k] as usize]
                                }
                            });
                            let im_particles: [Real; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                                if particles[k] == u32::MAX {
                                    0.0
                                } else {
                                    sb.particles[anchors[k] as usize].inv_mass
                                }
                            });
                            let anchor_positions: [Vector; CONTACT_ANCHORS] =
                                core::array::from_fn(|k| {
                                    if anchors[k] == u32::MAX {
                                        Vector::ZERO
                                    } else {
                                        sb.particles[anchors[k] as usize].position
                                    }
                                });
                            let (body_local_point, body_arm) = match &other_com_pose {
                                Some(pose) => (
                                    pose.inverse_transform_point(other_anchor),
                                    other_anchor - pose.translation,
                                ),
                                None => (other_anchor, Vector::ZERO),
                            };
                            // A small ball (a soft particle) touching a surface vertex/edge while
                            // its center projects inside a neighboring element is a ghost contact
                            // (spurious normal): skipped; large balls and split endpoints are real.
                            if let Some(ball) = other_co
                                .shape()
                                .as_ball()
                                .filter(|_| !split_endpoints)
                            {
                                let size = (positions[1] - positions[0]).length();
                                if ball.radius < size {
                                    let center = other_anchor - dir * ball.radius;
                                    if !skinned
                                        && mesh.contact_is_ghost(sb, element_id, &weights, center)
                                    {
                                        continue;
                                    }
                                }
                            }
                            // Vertex contact shared with a neighboring element?
                            let vertex = (0..element.len()).find(|&k| weights[k] > 0.999);
                            let vertex_key = vertex.map(|k| element[k]);
                            let mut replace_constraint = None;
                            if let Some(key) = vertex_key {
                                if let Some(&existing) = vertex_constraints.get(&key) {
                                    if out.contacts[existing].dist0 <= dist {
                                        continue;
                                    }
                                    replace_constraint = Some(existing);
                                }
                            }
                            let feature_key = if vertex_key.is_none() && rigid_other.is_some() {
                                let point = &manifold.points[point_id];
                                let fid = if surface_first {
                                    point.fid2
                                } else {
                                    point.fid1
                                };
                                let dominant = (0..element.len())
                                    .max_by(|&a, &b| weights[a].partial_cmp(&weights[b]).unwrap())
                                    .map(|k| element[k])
                                    .unwrap_or(0);
                                Some((fid.0, dominant))
                            } else {
                                None
                            };
                            if let Some(key) = feature_key {
                                if let Some(&existing) = feature_constraints.get(&key) {
                                    if out.contacts[existing].dist0 <= dist {
                                        continue;
                                    }
                                    replace_constraint = Some(existing);
                                }
                            }
                            let tangents = SoftContact::tangent_basis(dir);
                            // Warm start from the manifold point (world-space friction impulse
                            // projected on the fresh tangent basis).
                            let data = &manifold.points[point_id].data;
                            let warm_normal = data.warmstart_impulse * warm_share;
                            #[cfg(feature = "dim2")]
                            let warm_tangent = [data.warmstart_tangent_impulse[0] * warm_share];
                            #[cfg(feature = "dim3")]
                            let warm_tangent = [
                                data.warmstart_tangent_world.dot(tangents[0]) * warm_share,
                                data.warmstart_tangent_world.dot(tangents[1]) * warm_share,
                            ];
                            let contact = SoftContact {
                                source: SoftContactSource {
                                    collider1: pair.collider1,
                                    collider2: pair.collider2,
                                    manifold: mi as u32,
                                    point: point_id as u32,
                                    slot: u32::MAX,
                                },
                                support_body: ai as u32,
                                support_particle: anchors,
                                particles,
                                weights: anchor_weights,
                                im_particles,
                                frozen_pos: anchor_positions,
                                body: other_slot,
                                element: None,
                                other_body,
                                body_im: Vector::ZERO,
                                body_ii: Default::default(),
                                body_local_point,
                                body_arm,
                                surface_point0,
                                body_point0: other_anchor,
                                dir,
                                tangents,
                                dist0: dist,
                                friction: manifold.data.friction,
                                soft_other: false,
                                fem: [None, None],
                                erp_inv_dt,
                                cfm_factor,
                                max_bias: Real::MAX,
                                torque_dir: Default::default(),
                                ii_torque_dir: Default::default(),
                                r_normal: 0.0,
                                rhs_normal: 0.0,
                                cfm_normal: 1.0,
                                impulse_normal: warm_normal,
                                impulse_normal_acc: -warm_normal,
                                torque_tangent: [Default::default(); DIM - 1],
                                ii_torque_tangent: [Default::default(); DIM - 1],
                                r_tangent: [0.0; DIM - 1],
                                rhs_tangent: [0.0; DIM - 1],
                                impulse_tangent: warm_tangent,
                                impulse_tangent_acc: core::array::from_fn(|j| -warm_tangent[j]),
                            };
                            out.max_approach_speed = out.max_approach_speed.max(approach_speed(
                                element,
                                &weights,
                                other_anchor,
                                dir,
                            ));
                            match replace_constraint {
                                Some(existing) => out.contacts[existing] = contact,
                                None => {
                                    if let Some(key) = vertex_key {
                                        vertex_constraints.insert(key, out.contacts.len());
                                    }
                                    if let Some(key) = feature_key {
                                        feature_constraints.insert(key, out.contacts.len());
                                    }
                                    out.contacts.push(contact);
                                }
                            }
                            if let Some(k) = (0..element.len()).find(|&k| weights[k] > 0.9) {
                                held_vertices.push(element[k]);
                            }
                        }
                    }

                    // The pair's predictive vertex contacts (see `SoftRigidVertexContact`): the
                    // support of the vertices a manifold leaves free when the rigid shape wraps
                    // the elements, unless a manifold constraint already holds the vertex.
                    let vertex_contacts = pair
                        .rigid()
                        .and_then(|r| r.soft.as_deref())
                        .map_or(&[][..], |s| &s.vertices[..]);
                    let other_pose = other_co.position();
                    held_vertices.sort_unstable();
                    for (index, vc) in vertex_contacts.iter().enumerate() {
                        if held_vertices.binary_search(&vc.vertex).is_ok() {
                            continue;
                        }
                        // A vertex of the volume patch is left to the volume constraint.
                        if rigid_patch.as_ref().is_some_and(|(depths, _)| {
                            depths
                                .get(vc.vertex as usize)
                                .is_some_and(|d| *d > Real::NEG_INFINITY)
                        }) {
                            continue;
                        }
                        let Some(manifold) = pair.manifolds().get(vc.manifold as usize) else {
                            continue;
                        };
                        let dir = other_pose.rotation * vc.local_dir;
                        // A closed surface's reversed contact (a body inside it) is dropped, like
                        // the manifold constraints'.
                        if closed
                            && manifold_element(manifold)
                                .and_then(|(id, _)| mesh.element_outward_normal(sb, id))
                                .and_then(|n| n.try_normalize())
                                .is_some_and(|n| dir.gdot(n) > 0.5)
                        {
                            continue;
                        }
                        let other_anchor = other_pose * vc.local_point;
                        let vertex_pos = mesh.vertex(sb, vc.vertex as usize);
                        let (anchors, anchor_weights, surface_point0) =
                            mesh.contact_anchors(sb, &[vc.vertex], &[1.0], vertex_pos);
                        let particles: [u32; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                            if anchors[k] == u32::MAX {
                                u32::MAX
                            } else {
                                slots[anchors[k] as usize]
                            }
                        });
                        let im_particles: [Real; CONTACT_ANCHORS] = core::array::from_fn(|k| {
                            if particles[k] == u32::MAX {
                                0.0
                            } else {
                                sb.particles[anchors[k] as usize].inv_mass
                            }
                        });
                        let anchor_positions: [Vector; CONTACT_ANCHORS] =
                            core::array::from_fn(|k| {
                                if anchors[k] == u32::MAX {
                                    Vector::ZERO
                                } else {
                                    sb.particles[anchors[k] as usize].position
                                }
                            });
                        let (body_local_point, body_arm) = match &other_com_pose {
                            Some(pose) => (
                                pose.inverse_transform_point(other_anchor),
                                other_anchor - pose.translation,
                            ),
                            None => (other_anchor, Vector::ZERO),
                        };
                        let tangents = SoftContact::tangent_basis(dir);
                        let warm_tangent: [Real; DIM - 1] =
                            core::array::from_fn(|k| vc.tangent_impulse.gdot(tangents[k]));
                        let mut weights = [0.0; DIM];
                        weights[0] = 1.0;
                        out.max_approach_speed = out.max_approach_speed.max(approach_speed(
                            &[vc.vertex],
                            &weights,
                            other_anchor,
                            dir,
                        ));
                        out.contacts.push(SoftContact {
                            source: SoftContactSource {
                                collider1: pair.collider1,
                                collider2: pair.collider2,
                                manifold: SOURCE_RIGID_VERTEX,
                                point: index as u32,
                                slot: u32::MAX,
                            },
                            support_body: ai as u32,
                            support_particle: anchors,
                            particles,
                            weights: anchor_weights,
                            im_particles,
                            frozen_pos: anchor_positions,
                            body: other_slot,
                            element: None,
                            other_body,
                            body_im: Vector::ZERO,
                            body_ii: Default::default(),
                            body_local_point,
                            body_arm,
                            surface_point0,
                            body_point0: other_anchor,
                            dir,
                            tangents,
                            dist0: vc.dist,
                            friction: manifold.data.friction,
                            soft_other: false,
                            fem: [None, None],
                            erp_inv_dt,
                            cfm_factor,
                            max_bias: Real::MAX,
                            torque_dir: Default::default(),
                            ii_torque_dir: Default::default(),
                            r_normal: 0.0,
                            rhs_normal: 0.0,
                            cfm_normal: 1.0,
                            impulse_normal: vc.impulse,
                            impulse_normal_acc: -vc.impulse,
                            torque_tangent: [Default::default(); DIM - 1],
                            ii_torque_tangent: [Default::default(); DIM - 1],
                            r_tangent: [0.0; DIM - 1],
                            rhs_tangent: [0.0; DIM - 1],
                            impulse_tangent: warm_tangent,
                            impulse_tangent_acc: core::array::from_fn(|k| -warm_tangent[k]),
                        });
                    }
                }
                // Self contacts (the body's own vertices and edges against its surface), then
                // the edge-vs-edge constraints against the other soft surfaces met above.
                let Some(surface_co) = colliders.get(surface_handle) else {
                    continue;
                };
                if mesh.self_contacts && !self_frozen {
                    if let Some(detected) = narrow_phase.soft_self_contacts(surface_handle) {
                        // The tangle signal the narrow phase read off the surface (see
                        // `soft_contacts`), and the crossing sweep's travel bookkeeping.
                        out.mesh().crossing_sweep_travel = detected.crossing_sweep_travel_next;
                        out.tangled_elements.clone_from(&detected.tangled_elements);
                        out.tangled_vertices.clone_from(&detected.tangled_vertices);
                        out.crossings.clone_from(&detected.crossings);
                        out.exempt_self_tangles(surface_handle);
                        self.assemble_vertex_contacts(
                            ctx,
                            ai,
                            Some(ai),
                            sb,
                            (surface_handle, surface_co),
                            (surface_handle, surface_co),
                            false,
                            &detected.vertex_pass,
                            None,
                            &detected.region_bins,
                            out,
                        );
                        self.assemble_edge_contacts(
                            params,
                            ai,
                            Some(ai),
                            sb,
                            (surface_handle, surface_co),
                            (surface_handle, surface_co),
                            (dyn_erp, dyn_cfm),
                            detected.edges.as_ref(),
                            out,
                        );
                    }
                }
                for (other_ai, other_sb_handle, other_surface, detected) in edge_tasks {
                    let (Some(other_sb), Some(other_co)) = (
                        soft_bodies.get(other_sb_handle),
                        colliders.get(other_surface),
                    ) else {
                        continue;
                    };
                    self.assemble_edge_contacts(
                        params,
                        ai,
                        other_ai,
                        other_sb,
                        (surface_handle, surface_co),
                        (other_surface, other_co),
                        (dyn_erp, dyn_cfm),
                        detected,
                        out,
                    );
                }
            }
        }
    }
}
