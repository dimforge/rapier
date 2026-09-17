//! Mouse grabbing: left-click-drag any dynamic object (rigid body, multibody link, or soft
//! body) to pull it around with a soft mouse joint, box2d-style. The grab spawns an invisible
//! kinematic body following the cursor, tied to the picked body by a motor-only joint.

#![allow(clippy::useless_conversion)] // Conversions are needed for switching between f32/f64.
#![allow(clippy::unnecessary_cast)]

use crate::mouse::SceneMouse;
use rapier::dynamics::{ImpulseJointHandle, RigidBodyBuilder, RigidBodyHandle};
use rapier::math::{Real, Vector};
use rapier::pipeline::PhysicsWorld;
use rapier::prelude::{
    GenericJoint, GenericJointBuilder, JointAxesMask, JointAxis, QueryFilter, QueryFilterFlags,
};

#[cfg(feature = "dim3")]
use rapier::prelude::Ray;

/// Spring gains of the mouse joint, roughly a 5 Hz near-critically-damped spring.
const GRAB_STIFFNESS: Real = 1000.0;
const GRAB_DAMPING: Real = 50.0;

#[derive(Default)]
pub struct MouseGrab {
    grabbed: Option<Grabbed>,
}

struct Grabbed {
    /// The body the joint pulled at first: the picked body itself, or the temporary cluster's
    /// proxy (a tear may have moved the joint to another proxy since, see [`Grabbed::pulled`]).
    body: RigidBodyHandle,
    /// Invisible kinematic body following the cursor.
    mouse_body: RigidBodyHandle,
    /// The mouse joint.
    joint: ImpulseJointHandle,
    /// Whether the grab holds a soft body through a temporary cluster proxy, torn down on
    /// release together with whatever proxy the joint ended on.
    soft: bool,
    /// Anchor of the plane the cursor ray is intersected with while dragging.
    #[cfg(feature = "dim3")]
    plane_point: Vector,
}

impl Grabbed {
    /// The body the joint pulls now: the picked body, or the proxy a tear handed the joint to.
    fn pulled(&self, world: &PhysicsWorld) -> RigidBodyHandle {
        world
            .impulse_joints
            .get(self.joint)
            .map_or(self.body, |joint| joint.body2())
    }
}

impl MouseGrab {
    pub fn active(&self) -> bool {
        self.grabbed.is_some()
    }

    /// World-space endpoints of the mouse joint, for drawing a cue line.
    pub fn cue_line(&self, world: &PhysicsWorld) -> Option<(Vector, Vector)> {
        let g = self.grabbed.as_ref()?;
        let mouse = world.bodies.get(g.mouse_body)?.position().translation;
        let body = world.bodies.get(g.pulled(world))?.position().translation;
        Some((mouse, body))
    }

    /// Drops the grab state without touching the world (the world was replaced).
    pub fn forget(&mut self) {
        self.grabbed = None;
    }

    /// Picks the dynamic object under the cursor and attaches the mouse joint to it.
    /// Returns `true` if something was grabbed.
    pub fn try_grab(&mut self, mouse: &SceneMouse, world: &mut PhysicsWorld) -> bool {
        if self.grabbed.is_some() {
            return false;
        }
        let Some((body_handle, grab_point)) = pick(mouse, world) else {
            return false;
        };
        if !world.bodies[body_handle].is_dynamic() {
            return false;
        }

        let grabbed = if let Some(sb_handle) = world.bodies[body_handle].soft_body() {
            // Soft body: drag its nearest particle through a temporary one-particle cluster.
            // The single particle makes the cluster rank-deficient, so the joint's angular
            // axes are stripped automatically and this acts as a pure point pull.
            let sb = &world.soft_bodies[sb_handle];
            let mut nearest = 0u32;
            let mut best_dist = Real::MAX;
            for (i, p) in sb.particle_positions().enumerate() {
                let dist = (p - grab_point).length_squared();
                if dist < best_dist {
                    best_dist = dist;
                    nearest = i as u32;
                }
            }
            let anchor = sb.particle_position(nearest as usize);
            let Some(cluster) = world.add_soft_body_cluster(sb_handle, &[nearest]) else {
                return false;
            };
            let Some(proxy) = world.soft_bodies[sb_handle].cluster_proxy(cluster) else {
                world.remove_soft_body_cluster(sb_handle, cluster);
                return false;
            };
            let mouse_body = world
                .insert_body(RigidBodyBuilder::kinematic_position_based().translation(anchor));
            let joint = world.insert_impulse_joint(mouse_body, proxy, mouse_joint(Vector::ZERO));
            Grabbed {
                body: proxy,
                mouse_body,
                joint,
                soft: true,
                #[cfg(feature = "dim3")]
                plane_point: anchor,
            }
        } else {
            let local_anchor = world.bodies[body_handle]
                .position()
                .inverse_transform_point(grab_point);
            let mouse_body = world
                .insert_body(RigidBodyBuilder::kinematic_position_based().translation(grab_point));
            let joint =
                world.insert_impulse_joint(mouse_body, body_handle, mouse_joint(local_anchor));
            Grabbed {
                body: body_handle,
                mouse_body,
                joint,
                soft: false,
                #[cfg(feature = "dim3")]
                plane_point: grab_point,
            }
        };

        if let Some(rb) = world.bodies.get_mut(grabbed.body) {
            rb.wake_up(true);
        }
        self.grabbed = Some(grabbed);
        true
    }

    /// Moves the kinematic anchor to the cursor. In 3D the cursor ray is intersected with the
    /// camera-facing plane through the initial grab point, keeping the drag depth constant.
    pub fn update(
        &mut self,
        mouse: &SceneMouse,
        #[cfg(feature = "dim3")] camera_fwd: Vector,
        world: &mut PhysicsWorld,
    ) {
        let Some(g) = &self.grabbed else {
            return;
        };
        // The example may have removed the body (or the whole soft body) while we held it.
        let pulled = g.pulled(world);
        if world.bodies.get(pulled).is_none() || world.bodies.get(g.mouse_body).is_none() {
            self.release(world);
            return;
        }

        #[cfg(feature = "dim2")]
        let target = match mouse.point {
            Some(pt) => Vector::new(pt.x as Real, pt.y as Real),
            None => return,
        };
        #[cfg(feature = "dim3")]
        let target = {
            let Some((orig, dir)) = mouse.ray else {
                return;
            };
            let orig = Vector::new(orig.x as Real, orig.y as Real, orig.z as Real);
            let dir = Vector::new(dir.x as Real, dir.y as Real, dir.z as Real);
            let denom = dir.dot(camera_fwd);
            if denom.abs() < 1.0e-6 {
                return;
            }
            let t = (g.plane_point - orig).dot(camera_fwd) / denom;
            if t <= 0.0 {
                return;
            }
            orig + dir * t
        };

        world.bodies[g.mouse_body].set_next_kinematic_translation(target);
        world.bodies[pulled].wake_up(true);
    }

    /// Removes the mouse joint, its kinematic anchor, and any temporary cluster proxy (the one
    /// picked, and the one a tear handed the joint to). Removing a proxy dissolves its cluster
    /// in whichever soft body holds it.
    pub fn release(&mut self, world: &mut PhysicsWorld) {
        let Some(g) = self.grabbed.take() else {
            return;
        };
        let pulled = g.pulled(world);
        if let Some(rb) = world.bodies.get_mut(pulled) {
            rb.wake_up(true);
        }
        // Removing the body also removes the attached mouse joint.
        world.remove_body(g.mouse_body);
        if g.soft {
            for proxy in [g.body, pulled] {
                if world.bodies.get(proxy).is_some_and(|rb| rb.is_soft_frame()) {
                    world.remove_body(proxy);
                }
            }
        }
    }
}

/// A motor-only joint: nothing locked, spring position motors driving the anchors together on
/// every linear axis. Rotation stays completely free.
fn mouse_joint(local_anchor2: Vector) -> GenericJoint {
    let joint = GenericJointBuilder::new(JointAxesMask::empty())
        .local_anchor2(local_anchor2)
        .motor_position(JointAxis::LinX, 0.0, GRAB_STIFFNESS, GRAB_DAMPING)
        .motor_position(JointAxis::LinY, 0.0, GRAB_STIFFNESS, GRAB_DAMPING);
    #[cfg(feature = "dim3")]
    let joint = joint.motor_position(JointAxis::LinZ, 0.0, GRAB_STIFFNESS, GRAB_DAMPING);
    joint.into()
}

fn query_filter() -> QueryFilter<'static> {
    QueryFilter::from(QueryFilterFlags::ONLY_DYNAMIC | QueryFilterFlags::EXCLUDE_SENSORS)
}

/// The dynamic body under the cursor and the world-space grab point on it.
#[cfg(feature = "dim2")]
fn pick(mouse: &SceneMouse, world: &PhysicsWorld) -> Option<(RigidBodyHandle, Vector)> {
    let pt = mouse.point?;
    let point = Vector::new(pt.x as Real, pt.y as Real);
    let query_pipeline = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        query_filter(),
    );
    // An exact hit inside a solid collider. A 2D soft body's surface polyline is oriented, so
    // parry classifies its interior (holes included) with the vertex pseudo-normals.
    let inside = query_pipeline.intersect_point(point).find_map(|(handle, co)| {
        // An open soft surface (rope, cloth) only has an outward side, not an interior.
        if let Some(mesh_ref) = co.deformable_mesh_ref() {
            let mesh = world.soft_bodies.get(mesh_ref.body)?.mesh_of(handle)?;
            if !mesh.is_closed() {
                return None;
            }
        }
        co.parent()
    });
    if let Some(body) = inside {
        return Some((body, point));
    }

    // Otherwise the nearest boundary within a cursor-sized tolerance: soft bodies expose thin
    // particle balls and surfaces that an exact test misses.
    let (handle, proj) = query_pipeline.project_point(point, mouse.pick_radius as Real, true)?;
    Some((world.colliders[handle].parent()?, proj.point))
}

#[cfg(feature = "dim3")]
fn pick(mouse: &SceneMouse, world: &PhysicsWorld) -> Option<(RigidBodyHandle, Vector)> {
    let (orig, dir) = mouse.ray?;
    let ray = Ray::new(orig.into(), dir.into());
    let query_pipeline = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        query_filter(),
    );
    let (handle, toi) = query_pipeline.cast_ray(&ray, Real::MAX, true)?;
    let body = world.colliders[handle].parent()?;
    Some((body, ray.origin + ray.dir * toi))
}
