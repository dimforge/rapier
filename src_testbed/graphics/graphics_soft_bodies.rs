//! Soft-body rendering: soft-body colors, the skins a soft body only draws, and the collision
//! meshes whose geometry deforms under their colliders.

use super::graphics_polyline::polyline_geometry;
#[cfg(feature = "dim2")]
use super::graphics_polyline::stroked_polyline;
#[cfg(feature = "dim3")]
use super::graphics_polyline::tube_polyline;
use super::{GraphicsManager, IndividualNode, SceneNode};
use kiss3d::prelude::*;
use rapier::dynamics::{RigidBodySet, SoftBodyHandle, SoftBodySet, SoftMeshRef};
use rapier::geometry::{Collider, ColliderSet, Shape, SharedShape};
use std::collections::HashMap;

/// A render-only node following a mesh a soft body only *draws*: the skin it wears without
/// colliding through it. A colliding mesh is a collider and is drawn as one; a drawn skin has no
/// collider, and is what the body looks like, so it replaces the collider on screen.
struct SoftMeshNode {
    node: SceneNode,
    mesh: SoftMeshRef,
    color: Color,
    /// Whether the node's mesh shares its vertices between elements (smooth normals): how it
    /// was built, and how the moved vertices are written back into it.
    smooth: bool,
}

/// The soft-body render state of a [`GraphicsManager`].
#[derive(Default)]
pub(super) struct SoftBodyGraphics {
    /// Soft-body meshes that are only drawn (the skins), which no collider holds.
    mesh_nodes: Vec<SoftMeshNode>,
    /// Colors per soft body (shared by its particles and its surface).
    colors: HashMap<SoftBodyHandle, Color>,
}

impl SoftBodyGraphics {
    pub(super) fn clear(&mut self) {
        self.mesh_nodes.clear();
        self.colors.clear();
    }
}

/// Is this mesh a skin the body only draws? A skin is what the body looks like, so a skin that
/// does not collide is still rendered, unlike a boundary the body does not collide
/// through, which stands for nothing on screen.
fn is_drawn_skin(mesh: &rapier::dynamics::SoftCollisionMesh) -> bool {
    mesh.is_skinned() && !mesh.collision_enabled()
}

/// The shape a drawn soft-body mesh is built as, from its world-space geometry: the same form
/// its colliding counterpart takes (a triangle mesh in 3D, a polyline in 2D).
fn drawn_mesh_shape(
    vertices: Vec<rapier::math::Vector>,
    indices: &[[u32; rapier::math::DIM]],
) -> Option<SharedShape> {
    if vertices.is_empty() || indices.is_empty() {
        return None;
    }
    #[cfg(feature = "dim2")]
    {
        use rapier::parry::shape::Polyline;
        Some(SharedShape::new(Polyline::new(
            vertices,
            Some(indices.to_vec()),
        )))
    }
    #[cfg(feature = "dim3")]
    {
        rapier::geometry::TriMesh::new(vertices, indices.to_vec())
            .ok()
            .map(SharedShape::new)
    }
}

/// Writes moved `vertices` into a triangle-mesh node's vertex buffer in the node's layout: three
/// unshared vertices per triangle under flat shading (`RenderMesh::replicate_vertices`),
/// the mesh's own under smooth shading. Returns `false` when the counts differ (topology changed).
#[cfg(feature = "dim3")]
fn write_mesh_vertices(
    node: &mut SceneNode,
    smooth: bool,
    vertices: &[rapier::math::Vector],
    indices: &[[u32; 3]],
) -> bool {
    let single = if smooth {
        vertices.len()
    } else {
        indices.len() * 3
    };
    // An open surface is built double-sided (see `create_individual_node`): a second copy of
    // the geometry, wound the other way, after the first.
    let expected = if super::is_open_surface(indices) {
        single * 2
    } else {
        single
    };
    let mut usable = true;
    node.modify_vertices(&mut |vtx: &mut Vec<Vec3>| {
        if vtx.len() != expected {
            usable = false;
            return;
        }
        let point = |i: u32| {
            let p = vertices[i as usize];
            Vec3::new(p.x as f32, p.y as f32, p.z as f32)
        };
        if smooth {
            for (v, i) in vtx.iter_mut().zip(0..vertices.len() as u32) {
                *v = point(i);
            }
            if expected > single {
                let (front, back) = vtx.split_at_mut(single);
                back.copy_from_slice(front);
            }
        } else {
            for (triangle, corners) in indices.iter().enumerate() {
                for (k, corner) in corners.iter().enumerate() {
                    vtx[triangle * 3 + k] = point(*corner);
                }
            }
            if expected > single {
                for (triangle, [a, b, c]) in indices.iter().enumerate() {
                    let base = single + triangle * 3;
                    vtx[base] = point(*a);
                    vtx[base + 1] = point(*c);
                    vtx[base + 2] = point(*b);
                }
            }
        }
    });
    if usable {
        node.recompute_normals();
    }
    usable
}

/// The 2D counterpart: a soft body's drawn mesh is a polyline, redrawn as a stroked mesh from
/// its moved vertices.
#[cfg(feature = "dim2")]
fn write_mesh_vertices(
    node: &mut SceneNode,
    _smooth: bool,
    vertices: &[rapier::math::Vector],
    indices: &[[u32; 2]],
) -> bool {
    let points: Vec<Vec2> = vertices
        .iter()
        .map(|p| Vec2::new(p.x as f32, p.y as f32))
        .collect();
    let (stroke, _) = stroked_polyline(&points, indices);
    let mut usable = true;
    node.modify_vertices(&mut |vtx: &mut Vec<Vec2>| {
        if vtx.len() != stroke.len() {
            usable = false;
            return;
        }
        vtx.copy_from_slice(&stroke);
    });
    usable
}

/// Whether a collider's node is hidden by a drawn skin: a body wearing one is seen through it, and
/// its collision meshes (the cells' boundary, which encloses the skin) would hide it.
#[cfg(feature = "dim3")]
pub(super) fn is_hidden_by_skin(collider: &Collider, soft_bodies: &SoftBodySet) -> bool {
    collider
        .deformable_mesh_ref()
        .and_then(|mesh| soft_bodies.get(mesh.body))
        .is_some_and(|sb| sb.meshes().any(is_drawn_skin))
}

impl GraphicsManager {
    /// The color of a soft body (allocated on first request; also applied to its particles).
    pub fn soft_body_color(&mut self, handle: SoftBodyHandle) -> Color {
        if let Some(c) = self.soft_graphics.colors.get(&handle) {
            return *c;
        }
        let color = Self::gen_color(&mut self.curr_color_index);
        self.soft_graphics.colors.insert(handle, color);
        color
    }

    /// Sets the color of a soft body (its surface and its particles).
    pub fn set_initial_soft_body_color(&mut self, handle: SoftBodyHandle, color: Color) {
        self.soft_graphics.colors.insert(handle, color);
    }

    /// Adds a render node for each mesh `handle` only draws (a skin worn without colliding through
    /// it): such a mesh has no collider for the collider path and is what the body looks like, so
    /// the body's collision meshes are hidden in its favor (see [`Self::draw`]).
    pub fn add_soft_body_meshes(&mut self, handle: SoftBodyHandle, soft_bodies: &SoftBodySet) {
        let Some(sb) = soft_bodies.get(handle) else {
            return;
        };
        let color = self.soft_body_color(handle);
        let smooth = self.smooth_mesh_colliders;
        let visible = self.draw_surfaces && self.colliders_visible;
        for mesh in sb.meshes().filter(|mesh| is_drawn_skin(mesh)) {
            let Some(shape) = drawn_mesh_shape(mesh.vertex_positions(sb).collect(), mesh.indices())
            else {
                continue;
            };
            let Some(mut node) =
                Self::create_individual_node(&mut self.scene, &*shape, color, false, smooth)
            else {
                continue;
            };
            node.set_visible(visible);
            self.soft_graphics.mesh_nodes.push(SoftMeshNode {
                node,
                mesh: SoftMeshRef {
                    body: handle,
                    id: mesh.id(),
                },
                color,
                smooth,
            });
        }
    }

    /// Registers render nodes for what the soft bodies gained since the last frame: a tear splits
    /// pieces as new soft bodies (fresh proxies and colliders) and moves clusters between bodies.
    /// Each proxy wears its body's color; drawn skins get nodes, moved-away meshes lose theirs.
    pub fn add_missing_soft_body_graphics(
        &mut self,
        window: &mut Window,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        soft_bodies: &SoftBodySet,
    ) {
        for (handle, sb) in soft_bodies.iter() {
            let color = self.soft_body_color(handle);
            for (_, cluster) in sb.live_clusters() {
                let proxy = cluster.proxy();
                self.set_initial_body_color(proxy, color);
                let Some(rb) = bodies.get(proxy) else {
                    continue;
                };
                for &co_handle in rb.colliders() {
                    if self.c2nodes.contains_key(&co_handle) {
                        continue;
                    }
                    let Some(co) = colliders.get(co_handle) else {
                        continue;
                    };
                    self.add_shape(
                        window,
                        co_handle,
                        Some(proxy),
                        co.shape(),
                        co.is_sensor(),
                        rapier::math::Pose::IDENTITY,
                        color,
                    );
                }
            }
            for mesh in sb.meshes().filter(|mesh| is_drawn_skin(mesh)) {
                let mesh_ref = SoftMeshRef {
                    body: handle,
                    id: mesh.id(),
                };
                if self.soft_graphics.mesh_nodes.iter().any(|n| n.mesh == mesh_ref) {
                    continue;
                }
                let Some(shape) =
                    drawn_mesh_shape(mesh.vertex_positions(sb).collect(), mesh.indices())
                else {
                    continue;
                };
                let smooth = self.smooth_mesh_colliders;
                let Some(mut node) =
                    Self::create_individual_node(&mut self.scene, &*shape, color, false, smooth)
                else {
                    continue;
                };
                node.set_visible(self.draw_surfaces && self.colliders_visible);
                self.soft_graphics.mesh_nodes.push(SoftMeshNode {
                    node,
                    mesh: mesh_ref,
                    color,
                    smooth,
                });
            }
        }
        // The nodes of meshes that are gone (a removed body, a mesh moved to a piece).
        self.soft_graphics.mesh_nodes.retain_mut(|n| {
            let live = soft_bodies
                .get(n.mesh.body)
                .and_then(|sb| sb.mesh(n.mesh.id))
                .is_some();
            if !live {
                n.node.detach();
            }
            live
        });
    }

    /// Follows the drawn soft-body meshes: their vertices move every step, so each node's vertex
    /// buffer is rewritten in place. A node whose mesh is gone (a removed body) is hidden, and
    /// one whose geometry no longer matches (a shading change, or a tear) is built anew.
    pub(super) fn update_soft_mesh_nodes(&mut self, soft_bodies: &SoftBodySet, visible: bool) {
        let smooth = self.smooth_mesh_colliders;
        for index in 0..self.soft_graphics.mesh_nodes.len() {
            let node = &mut self.soft_graphics.mesh_nodes[index];
            let mesh_ref = node.mesh;
            let Some(sb) = soft_bodies.get(mesh_ref.body) else {
                node.node.set_visible(false);
                continue;
            };
            let Some(mesh) = sb.mesh(mesh_ref.id).filter(|mesh| is_drawn_skin(mesh)) else {
                node.node.set_visible(false);
                continue;
            };
            node.node.set_visible(visible);
            let vertices: Vec<_> = mesh.vertex_positions(sb).collect();
            let stale = node.smooth != smooth
                || !write_mesh_vertices(&mut node.node, node.smooth, &vertices, mesh.indices());
            if stale {
                self.rebuild_soft_mesh_node(index, vertices, mesh.indices(), visible);
            }
            let node = &mut self.soft_graphics.mesh_nodes[index];
            node.node
                .set_pose(rapier::math::Pose::from_translation(self.gfx_shift).into());
            node.node.set_color_recursive(node.color);
        }
    }

    /// Rebuilds a drawn mesh's node from the geometry it now has.
    fn rebuild_soft_mesh_node(
        &mut self,
        index: usize,
        vertices: Vec<rapier::math::Vector>,
        indices: &[[u32; rapier::math::DIM]],
        visible: bool,
    ) {
        let smooth = self.smooth_mesh_colliders;
        let color = self.soft_graphics.mesh_nodes[index].color;
        let Some(shape) = drawn_mesh_shape(vertices, indices) else {
            return;
        };
        self.soft_graphics.mesh_nodes[index].node.detach();
        if let Some(mut fresh) =
            Self::create_individual_node(&mut self.scene, &*shape, color, false, smooth)
        {
            fresh.set_visible(visible);
            let node = &mut self.soft_graphics.mesh_nodes[index];
            node.node = fresh;
            node.smooth = smooth;
        }
    }

    /// Follows a deformable collider's geometry: its shape's vertices move every step, so the
    /// node's vertex buffer is rewritten in place rather than rebuilt. Returns `false` when the
    /// shape's topology changed (a tear replaced it) and the node must be built anew.
    #[cfg(feature = "dim3")]
    fn update_deformable_node(node: &mut IndividualNode, shape: &dyn Shape) -> bool {
        if let Some((vertices, segments)) = polyline_geometry(shape) {
            let (tube, _) = tube_polyline(&vertices, &segments);
            let mut usable = true;
            node.node.modify_vertices(&mut |vtx: &mut Vec<Vec3>| {
                if vtx.len() != tube.len() {
                    usable = false;
                    return;
                }
                vtx.copy_from_slice(&tube);
            });
            if usable {
                node.node.recompute_normals();
            }
            return usable;
        }
        let Some(trimesh) = shape.as_trimesh() else {
            return true;
        };
        write_mesh_vertices(
            &mut node.node,
            node.smooth,
            trimesh.vertices(),
            trimesh.indices(),
        )
    }

    /// See the 3D overload: in 2D a deformable collider is a polyline, drawn as a stroked mesh
    /// whose geometry is rebuilt from the moved vertices.
    #[cfg(feature = "dim2")]
    fn update_deformable_node(node: &mut IndividualNode, shape: &dyn Shape) -> bool {
        let Some((vertices, segments)) = polyline_geometry(shape) else {
            return true;
        };
        let (stroke, _) = stroked_polyline(&vertices, &segments);
        let mut usable = true;
        node.node.modify_vertices(&mut |vtx: &mut Vec<Vec2>| {
            if vtx.len() != stroke.len() {
                usable = false;
                return;
            }
            vtx.copy_from_slice(&stroke);
        });
        usable
    }

    /// Follows a soft body's collision mesh: a collider like any other, except that its shape
    /// deforms every step and a tear replaces it.
    pub(super) fn update_deformable_collider(&mut self, index: usize, collider: &Collider) {
        if collider.deformable_mesh_ref().is_some()
            && !Self::update_deformable_node(&mut self.individual_nodes[index], collider.shape())
        {
            self.rebuild_individual_node(index, collider);
        }
    }
}
