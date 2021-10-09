use bevy::prelude::*;

use na::{point, Point3};

use crate::math::Isometry;
use crate::objects::node::EntityWithGraphics;
use rapier::dynamics::{RigidBodyHandle, RigidBodySet};
use rapier::geometry::{ColliderHandle, ColliderSet, Shape, ShapeType};
//use crate::objects::capsule::Capsule;
//#[cfg(feature = "dim3")]
//use crate::objects::mesh::Mesh;
//use crate::objects::plane::Plane;
//#[cfg(feature = "dim2")]
//use crate::objects::polyline::Polyline;
// use crate::objects::mesh::Mesh;
use rand::{Rng, SeedableRng};
use rand_pcg::Pcg32;
use std::collections::HashMap;

pub struct GraphicsManager {
    rand: Pcg32,
    b2sn: HashMap<RigidBodyHandle, Vec<EntityWithGraphics>>,
    b2color: HashMap<RigidBodyHandle, Point3<f32>>,
    c2color: HashMap<ColliderHandle, Point3<f32>>,
    b2wireframe: HashMap<RigidBodyHandle, bool>,
    ground_color: Point3<f32>,
    prefab_meshes: HashMap<ShapeType, Handle<Mesh>>,
}

impl GraphicsManager {
    pub fn new() -> GraphicsManager {
        GraphicsManager {
            rand: Pcg32::seed_from_u64(0),
            b2sn: HashMap::new(),
            b2color: HashMap::new(),
            c2color: HashMap::new(),
            ground_color: point![0.5, 0.5, 0.5],
            b2wireframe: HashMap::new(),
            prefab_meshes: HashMap::new(),
        }
    }

    pub fn clear(&mut self, commands: &mut Commands) {
        for sns in self.b2sn.values_mut() {
            for sn in sns.iter_mut() {
                commands.entity(sn.entity).despawn()
            }
        }

        self.b2sn.clear();
        self.c2color.clear();
        self.b2color.clear();
        self.b2wireframe.clear();
        self.rand = Pcg32::seed_from_u64(0);
    }

    pub fn remove_collider_nodes(
        &mut self,
        commands: &mut Commands,
        body: Option<RigidBodyHandle>,
        collider: ColliderHandle,
    ) {
        let body = body.unwrap_or(RigidBodyHandle::invalid());
        if let Some(sns) = self.b2sn.get_mut(&body) {
            for sn in sns.iter_mut() {
                if let Some(sn_c) = sn.collider {
                    if sn_c == collider {
                        commands.entity(sn.entity).despawn();
                    }
                }
            }
        }
    }

    pub fn remove_body_nodes(&mut self, commands: &mut Commands, body: RigidBodyHandle) {
        if let Some(sns) = self.b2sn.get_mut(&body) {
            for sn in sns.iter_mut() {
                commands.entity(sn.entity).despawn();
            }
        }

        self.b2sn.remove(&body);
    }

    pub fn set_body_color(
        &mut self,
        materials: &mut Assets<StandardMaterial>,
        b: RigidBodyHandle,
        color: [f32; 3],
    ) {
        self.b2color.insert(b, color.into());

        if let Some(ns) = self.b2sn.get_mut(&b) {
            for n in ns.iter_mut() {
                n.set_color(materials, color.into())
            }
        }
    }

    pub fn set_initial_body_color(&mut self, b: RigidBodyHandle, color: [f32; 3]) {
        self.b2color.insert(b, color.into());
    }

    pub fn set_initial_collider_color(&mut self, c: ColliderHandle, color: [f32; 3]) {
        self.c2color.insert(c, color.into());
    }

    pub fn set_body_wireframe(&mut self, b: RigidBodyHandle, enabled: bool) {
        self.b2wireframe.insert(b, enabled);

        if let Some(_ns) = self.b2sn.get_mut(&b) {
            // for n in ns.iter_mut().filter_map(|n| n.scene_node_mut()) {
            //     if enabled {
            //         n.set_surface_rendering_activation(true);
            //         n.set_lines_width(1.0);
            //     } else {
            //         n.set_surface_rendering_activation(false);
            //         n.set_lines_width(1.0);
            //     }
            // }
        }
    }

    pub fn toggle_wireframe_mode(&mut self, _colliders: &ColliderSet, _enabled: bool) {
        for _n in self.b2sn.values_mut().flat_map(|val| val.iter_mut()) {
            // let _force_wireframe = if let Some(collider) = colliders.get(n.collider) {
            //     collider.is_sensor()
            //         || self
            //             .b2wireframe
            //             .get(&collider.parent())
            //             .cloned()
            //             .unwrap_or(false)
            // } else {
            //     false
            // };

            // if let Some(node) = n.scene_node_mut() {
            //     if force_wireframe || enabled {
            //         node.set_lines_width(1.0);
            //         node.set_surface_rendering_activation(false);
            //     } else {
            //         node.set_lines_width(0.0);
            //         node.set_surface_rendering_activation(true);
            //     }
            // }
        }
    }

    pub fn next_color(&mut self) -> Point3<f32> {
        Self::gen_color(&mut self.rand)
    }

    fn gen_color(rng: &mut Pcg32) -> Point3<f32> {
        let mut color: Point3<f32> = rng.gen();
        color *= 1.5;
        color.x = color.x.min(1.0);
        color.y = color.y.min(1.0);
        color.z = color.z.min(1.0);
        color
    }

    fn alloc_color(
        &mut self,
        materials: &mut Assets<StandardMaterial>,
        handle: RigidBodyHandle,
        is_static: bool,
    ) -> Point3<f32> {
        let mut color = self.ground_color;

        if !is_static {
            match self.b2color.get(&handle).cloned() {
                Some(c) => color = c,
                None => color = Self::gen_color(&mut self.rand),
            }
        }

        self.set_body_color(materials, handle, color.into());

        color
    }

    pub fn add_body_colliders(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        components: &mut Query<(&mut Transform,)>,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        let body = bodies.get(handle).unwrap();

        let color = self
            .b2color
            .get(&handle)
            .cloned()
            .unwrap_or_else(|| self.alloc_color(materials, handle, !body.is_dynamic()));

        let _ = self.add_body_colliders_with_color(
            commands, meshes, materials, components, handle, bodies, colliders, color,
        );
    }

    pub fn add_body_colliders_with_color(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        components: &mut Query<(&mut Transform,)>,
        handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        color: Point3<f32>,
    ) -> Vec<EntityWithGraphics> {
        let mut new_nodes = Vec::new();

        for collider_handle in bodies[handle].colliders() {
            let color = self.c2color.get(collider_handle).copied().unwrap_or(color);
            let collider = &colliders[*collider_handle];
            self.add_shape(
                commands,
                meshes,
                materials,
                Some(*collider_handle),
                collider.shape(),
                collider.is_sensor(),
                collider.position(),
                &Isometry::identity(),
                color,
                &mut new_nodes,
            );
        }

        new_nodes
            .iter_mut()
            .for_each(|n| n.update(colliders, components));

        // for node in new_nodes.iter_mut().filter_map(|n| n.scene_node_mut()) {
        //     if self.b2wireframe.get(&handle).cloned() == Some(true) {
        //         node.set_lines_width(1.0);
        //         node.set_surface_rendering_activation(false);
        //     } else {
        //         node.set_lines_width(0.0);
        //         node.set_surface_rendering_activation(true);
        //     }
        // }

        let nodes = self.b2sn.entry(handle).or_insert_with(Vec::new);

        nodes.append(&mut new_nodes.clone());

        new_nodes
    }

    pub fn add_collider(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        handle: ColliderHandle,
        colliders: &ColliderSet,
    ) {
        let collider = &colliders[handle];
        let collider_parent = collider.parent().unwrap_or(RigidBodyHandle::invalid());
        let color = self
            .b2color
            .get(&collider_parent)
            .copied()
            .unwrap_or(self.ground_color);
        let color = self.c2color.get(&handle).copied().unwrap_or(color);
        let mut nodes = std::mem::replace(
            self.b2sn.entry(collider_parent).or_insert(vec![]),
            Vec::new(),
        );
        self.add_shape(
            commands,
            meshes,
            materials,
            Some(handle),
            collider.shape(),
            collider.is_sensor(),
            collider.position(),
            &Isometry::identity(),
            color,
            &mut nodes,
        );
        self.b2sn.insert(collider_parent, nodes);
    }

    pub fn add_shape(
        &mut self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        handle: Option<ColliderHandle>,
        shape: &dyn Shape,
        sensor: bool,
        pos: &Isometry<f32>,
        delta: &Isometry<f32>,
        color: Point3<f32>,
        out: &mut Vec<EntityWithGraphics>,
    ) {
        println!("Shape type: {:?}", shape.shape_type());
        if let Some(compound) = shape.as_compound() {
            for (shape_pos, shape) in compound.shapes() {
                self.add_shape(
                    commands,
                    meshes,
                    materials,
                    handle,
                    &**shape,
                    sensor,
                    pos,
                    &(shape_pos * delta),
                    color,
                    out,
                )
            }
        } else {
            if self.prefab_meshes.is_empty() {
                EntityWithGraphics::gen_prefab_meshes(&mut self.prefab_meshes, meshes);
            }

            let node = EntityWithGraphics::spawn(
                commands,
                meshes,
                materials,
                &self.prefab_meshes,
                shape,
                handle,
                *pos,
                *delta,
                color,
                sensor,
            );
            out.push(node);
        }
    }

    pub fn draw(
        &mut self,
        _bodies: &RigidBodySet,
        colliders: &ColliderSet,
        components: &mut Query<(&mut Transform,)>,
    ) {
        for (_, ns) in self.b2sn.iter_mut() {
            for n in ns.iter_mut() {
                // if let Some(co) = colliders.get(n.collider()) {
                //     let bo = &_bodies[co.parent()];
                //
                //     if bo.is_dynamic() {
                //         if bo.is_ccd_active() {
                //             n.set_color(point![1.0, 0.0, 0.0]);
                //         } else {
                //             n.set_color(point![0.0, 1.0, 0.0]);
                //         }
                //     }
                // }

                n.update(colliders, components);
            }
        }
    }

    // pub fn draw_positions(&mut self, window: &mut Window, rbs: &RigidBodies<f32>) {
    //     for (_, ns) in self.b2sn.iter_mut() {
    //         for n in ns.iter_mut() {
    //             let object = n.object();
    //             let rb = rbs.get(object).expect("Rigid body not found.");

    //             // if let WorldObjectBorrowed::RigidBody(rb) = object {
    //                 let t      = rb.position();
    //                 let center = rb.center_of_mass();

    //                 let rotmat = t.rotation.to_rotation_matrix().unwrap();
    //                 let x = rotmat.column(0) * 0.25f32;
    //                 let y = rotmat.column(1) * 0.25f32;
    //                 let z = rotmat.column(2) * 0.25f32;

    //                 window.draw_line(center, &(*center + x), &point![1.0, 0.0, 0.0]);
    //                 window.draw_line(center, &(*center + y), &point![0.0, 1.0, 0.0]);
    //                 window.draw_line(center, &(*center + z), &point![0.0, 0.0, 1.0]);
    //             // }
    //         }
    //     }
    // }

    pub fn body_nodes(&self, handle: RigidBodyHandle) -> Option<&Vec<EntityWithGraphics>> {
        self.b2sn.get(&handle)
    }

    pub fn body_nodes_mut(
        &mut self,
        handle: RigidBodyHandle,
    ) -> Option<&mut Vec<EntityWithGraphics>> {
        self.b2sn.get_mut(&handle)
    }

    pub fn nodes(&self) -> impl Iterator<Item = &EntityWithGraphics> {
        self.b2sn.values().flat_map(|val| val.iter())
    }

    pub fn nodes_mut(&mut self) -> impl Iterator<Item = &mut EntityWithGraphics> {
        self.b2sn.values_mut().flat_map(|val| val.iter_mut())
    }

    pub fn prefab_meshes(&self) -> &HashMap<ShapeType, Handle<Mesh>> {
        &self.prefab_meshes
    }

    pub fn prefab_meshes_mut(&mut self) -> &mut HashMap<ShapeType, Handle<Mesh>> {
        &mut self.prefab_meshes
    }
}

impl Default for GraphicsManager {
    fn default() -> Self {
        Self::new()
    }
}
