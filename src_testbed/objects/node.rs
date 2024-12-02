#![allow(clippy::unnecessary_cast)] // Casts are needed for switching between f32/f64.

use bevy::prelude::*;
use bevy::render::mesh::{Indices, VertexAttributeValues};

//use crate::objects::plane::Plane;
use na::{point, Point3, Vector3};
use std::collections::HashMap;

use bevy::render::render_resource::PrimitiveTopology;
use bevy_pbr::wireframe::Wireframe;
use rapier::geometry::{ColliderHandle, ColliderSet, Shape, ShapeType};
#[cfg(feature = "dim3")]
use rapier::geometry::{Cone, Cylinder};
use rapier::math::{Isometry, Real, Vector};

use crate::graphics::{BevyMaterial, InstancedMaterials, SELECTED_OBJECT_MATERIAL_KEY};
#[cfg(feature = "dim2")]
use {
    na::{Point2, Vector2},
    rapier::geometry::{Ball, Cuboid},
};

#[derive(Clone, Debug)]
pub struct EntityWithGraphics {
    pub entity: Entity,
    pub color: Point3<f32>,
    pub base_color: Point3<f32>,
    pub collider: Option<ColliderHandle>,
    pub delta: Isometry<Real>,
    pub opacity: f32,
    pub material: Handle<BevyMaterial>,
}

impl EntityWithGraphics {
    pub fn register_selected_object_material(
        materials: &mut Assets<BevyMaterial>,
        instanced_materials: &mut InstancedMaterials,
    ) {
        if instanced_materials.contains_key(&SELECTED_OBJECT_MATERIAL_KEY) {
            return; // Already added.
        }

        #[cfg(feature = "dim2")]
        let selection_material = bevy_sprite::ColorMaterial {
            color: Color::from(Srgba::rgb(1.0, 0.0, 0.0)),
            texture: None,
            ..default()
        };
        #[cfg(feature = "dim3")]
        let selection_material = StandardMaterial {
            metallic: 0.5,
            perceptual_roughness: 0.5,
            double_sided: true, // TODO: this doesn't do anything?
            ..StandardMaterial::from(Color::from(Srgba::rgb(1.0, 0.0, 0.0)))
        };

        instanced_materials.insert(
            SELECTED_OBJECT_MATERIAL_KEY,
            materials.add(selection_material),
        );
    }

    pub fn spawn(
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<BevyMaterial>,
        prefab_meshs: &HashMap<ShapeType, Handle<Mesh>>,
        instanced_materials: &mut InstancedMaterials,
        shape: &dyn Shape,
        collider: Option<ColliderHandle>,
        collider_pos: Isometry<Real>,
        delta: Isometry<Real>,
        color: Point3<f32>,
        sensor: bool,
    ) -> Self {
        Self::register_selected_object_material(materials, instanced_materials);

        let entity = commands.spawn_empty().id();

        let scale = collider_mesh_scale(shape);
        let mesh = prefab_meshs
            .get(&shape.shape_type())
            .cloned()
            .or_else(|| generate_collider_mesh(shape).map(|m| meshes.add(m)));

        let opacity = 1.0;
        let bevy_color = Color::from(Srgba::new(color.x, color.y, color.z, opacity));
        let shape_pos = collider_pos * delta;
        let mut transform = Transform::from_scale(scale);
        transform.translation.x = shape_pos.translation.vector.x as f32;
        transform.translation.y = shape_pos.translation.vector.y as f32;
        #[cfg(feature = "dim3")]
        {
            transform.translation.z = shape_pos.translation.vector.z as f32;
            transform.rotation = Quat::from_xyzw(
                shape_pos.rotation.i as f32,
                shape_pos.rotation.j as f32,
                shape_pos.rotation.k as f32,
                shape_pos.rotation.w as f32,
            );
        }
        #[cfg(feature = "dim2")]
        {
            if sensor {
                transform.translation.z = -10.0;
            }
            transform.rotation = Quat::from_rotation_z(shape_pos.rotation.angle() as f32);
        }

        #[cfg(feature = "dim2")]
        let material = bevy_sprite::ColorMaterial {
            color: bevy_color,
            texture: None,
            ..default()
        };
        #[cfg(feature = "dim3")]
        let material = StandardMaterial {
            metallic: 0.5,
            perceptual_roughness: 0.5,
            double_sided: true, // TODO: this doesn't do anything?
            ..StandardMaterial::from(bevy_color)
        };
        let material_handle = instanced_materials
            .entry(color.coords.map(|c| (c * 255.0) as usize).into())
            .or_insert_with(|| materials.add(material));
        let material_weak_handle = material_handle.clone_weak();

        if let Some(mesh) = mesh {
            #[cfg(feature = "dim2")]
            let bundle = (
                Mesh2d(mesh),
                MeshMaterial2d(material_handle.clone_weak()),
                transform,
            );
            #[cfg(feature = "dim3")]
            let bundle = (
                Mesh2d(mesh),
                MeshMaterial3d(material_handle.clone_weak()),
                transform,
            );

            let mut entity_commands = commands.entity(entity);
            entity_commands.insert(bundle);

            if sensor {
                entity_commands.insert(Wireframe);
            }
        }

        EntityWithGraphics {
            entity,
            color,
            base_color: color,
            collider,
            delta,
            material: material_weak_handle,
            opacity,
        }
    }

    pub fn despawn(&mut self, commands: &mut Commands) {
        //FIXME: Should this be despawn_recursive?
        commands.entity(self.entity).despawn();
    }

    pub fn set_color(&mut self, materials: &mut Assets<BevyMaterial>, color: Point3<f32>) {
        if let Some(material) = materials.get_mut(&self.material) {
            #[cfg(feature = "dim2")]
            {
                material.color = Color::from(Srgba::new(color.x, color.y, color.z, self.opacity));
            }
            #[cfg(feature = "dim3")]
            {
                material.base_color =
                    Color::from(Srgba::new(color.x, color.y, color.z, self.opacity));
            }
        }
        self.color = color;
        self.base_color = color;
    }

    pub fn update(
        &mut self,
        colliders: &ColliderSet,
        components: &mut Query<&mut Transform>,
        gfx_shift: &Vector<Real>,
    ) {
        if let Some(Some(co)) = self.collider.map(|c| colliders.get(c)) {
            if let Ok(mut pos) = components.get_mut(self.entity) {
                let co_pos = co.position() * self.delta;
                pos.translation.x = (co_pos.translation.vector.x + gfx_shift.x) as f32;
                pos.translation.y = (co_pos.translation.vector.y + gfx_shift.y) as f32;
                #[cfg(feature = "dim3")]
                {
                    pos.translation.z = (co_pos.translation.vector.z + gfx_shift.z) as f32;
                    pos.rotation = Quat::from_xyzw(
                        co_pos.rotation.i as f32,
                        co_pos.rotation.j as f32,
                        co_pos.rotation.k as f32,
                        co_pos.rotation.w as f32,
                    );
                }
                #[cfg(feature = "dim2")]
                {
                    pos.rotation = Quat::from_rotation_z(co_pos.rotation.angle() as f32);
                }
            }
        }
    }

    pub fn object(&self) -> Option<ColliderHandle> {
        self.collider
    }

    #[cfg(feature = "dim2")]
    pub fn gen_prefab_meshes(
        out: &mut HashMap<ShapeType, Handle<Mesh>>,
        meshes: &mut Assets<Mesh>,
    ) {
        //
        // Cuboid mesh
        //
        let cuboid = bevy_mesh_from_polyline(Cuboid::new(Vector2::new(1.0, 1.0)).to_polyline());
        out.insert(ShapeType::Cuboid, meshes.add(cuboid.clone()));
        out.insert(ShapeType::RoundCuboid, meshes.add(cuboid));

        //
        // Ball mesh
        //
        let ball = bevy_mesh_from_polyline(Ball::new(1.0).to_polyline(30));
        out.insert(ShapeType::Ball, meshes.add(ball));
    }

    #[cfg(feature = "dim3")]
    pub fn gen_prefab_meshes(
        out: &mut HashMap<ShapeType, Handle<Mesh>>,
        meshes: &mut Assets<Mesh>,
    ) {
        //
        // Cuboid mesh
        //
        let cuboid = Mesh::from(bevy::math::primitives::Cuboid::new(2.0, 2.0, 2.0));
        out.insert(ShapeType::Cuboid, meshes.add(cuboid.clone()));
        out.insert(ShapeType::RoundCuboid, meshes.add(cuboid));

        //
        // Ball mesh
        //
        let ball = Mesh::from(bevy::math::primitives::Sphere::new(1.0));
        out.insert(ShapeType::Ball, meshes.add(ball));

        //
        // Cylinder mesh
        //
        let cylinder = Cylinder::new(1.0, 1.0);
        let mesh = bevy_mesh(cylinder.to_trimesh(20));
        out.insert(ShapeType::Cylinder, meshes.add(mesh.clone()));
        out.insert(ShapeType::RoundCylinder, meshes.add(mesh));

        //
        // Cone mesh
        //
        let cone = Cone::new(1.0, 1.0);
        let mesh = bevy_mesh(cone.to_trimesh(10));
        out.insert(ShapeType::Cone, meshes.add(mesh.clone()));
        out.insert(ShapeType::RoundCone, meshes.add(mesh));

        //
        // Halfspace
        //
        let vertices = vec![
            point![-1000.0, 0.0, -1000.0],
            point![1000.0, 0.0, -1000.0],
            point![1000.0, 0.0, 1000.0],
            point![-1000.0, 0.0, 1000.0],
        ];
        let indices = vec![[0, 1, 2], [0, 2, 3]];
        let mesh = bevy_mesh((vertices, indices));
        out.insert(ShapeType::HalfSpace, meshes.add(mesh));
    }
}

#[cfg(feature = "dim2")]
fn bevy_mesh_from_polyline(vertices: Vec<Point2<Real>>) -> Mesh {
    let n = vertices.len();
    let idx = (1..n as u32 - 1).map(|i| [0, i, i + 1]).collect();
    let vtx = vertices
        .into_iter()
        .map(|v| Point3::new(v.x, v.y, 0.0))
        .collect();
    bevy_mesh((vtx, idx))
}

#[cfg(feature = "dim2")]
fn bevy_polyline(buffers: (Vec<Point2<Real>>, Option<Vec<[u32; 2]>>)) -> Mesh {
    let (vtx, idx) = buffers;
    // let mut normals: Vec<[f32; 3]> = vec![];
    let mut vertices: Vec<[f32; 3]> = vec![];

    if let Some(idx) = idx {
        for idx in idx {
            let a = vtx[idx[0] as usize];
            let b = vtx[idx[1] as usize];

            vertices.push([a.x as f32, a.y as f32, 0.0]);
            vertices.push([b.x as f32, b.y as f32, 0.0]);
        }
    } else {
        vertices = vtx.iter().map(|v| [v.x as f32, v.y as f32, 0.0]).collect();
    }

    let indices: Vec<_> = (0..vertices.len() as u32).collect();
    let uvs: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0]).collect();
    let normals: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0, 1.0]).collect();

    // Generate the mesh
    let mut mesh = Mesh::new(PrimitiveTopology::LineStrip, Default::default());
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn bevy_mesh(buffers: (Vec<Point3<Real>>, Vec<[u32; 3]>)) -> Mesh {
    let (vtx, idx) = buffers;
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut vertices: Vec<[f32; 3]> = vec![];

    for idx in idx {
        let a = vtx[idx[0] as usize];
        let b = vtx[idx[1] as usize];
        let c = vtx[idx[2] as usize];

        vertices.push(a.cast::<f32>().into());
        vertices.push(b.cast::<f32>().into());
        vertices.push(c.cast::<f32>().into());
    }

    for vtx in vertices.chunks(3) {
        let a = Point3::from(vtx[0]);
        let b = Point3::from(vtx[1]);
        let c = Point3::from(vtx[2]);
        let n = (b - a).cross(&(c - a)).normalize();
        normals.push(n.cast::<f32>().into());
        normals.push(n.cast::<f32>().into());
        normals.push(n.cast::<f32>().into());
    }

    normals
        .iter_mut()
        .for_each(|n| *n = Vector3::from(*n).normalize().into());
    let indices: Vec<_> = (0..vertices.len() as u32).collect();
    let uvs: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0]).collect();

    // Generate the mesh
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, Default::default());
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn collider_mesh_scale(co_shape: &dyn Shape) -> Vec3 {
    match co_shape.shape_type() {
        #[cfg(feature = "dim2")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::new(c.half_extents.x as f32, c.half_extents.y as f32, 1.0)
        }
        #[cfg(feature = "dim2")]
        ShapeType::RoundCuboid => {
            let c = &co_shape.as_round_cuboid().unwrap().inner_shape;
            Vec3::new(c.half_extents.x as f32, c.half_extents.y as f32, 1.0)
        }
        ShapeType::Ball => {
            let b = co_shape.as_ball().unwrap();
            Vec3::new(b.radius as f32, b.radius as f32, b.radius as f32)
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::from_slice(c.half_extents.cast::<f32>().as_slice())
        }
        #[cfg(feature = "dim3")]
        ShapeType::RoundCuboid => {
            let c = co_shape.as_round_cuboid().unwrap();
            Vec3::from_slice(c.inner_shape.half_extents.cast::<f32>().as_slice())
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cylinder => {
            let c = co_shape.as_cylinder().unwrap();
            Vec3::new(c.radius as f32, c.half_height as f32, c.radius as f32)
        }
        #[cfg(feature = "dim3")]
        ShapeType::RoundCylinder => {
            let c = &co_shape.as_round_cylinder().unwrap().inner_shape;
            Vec3::new(c.radius as f32, c.half_height as f32, c.radius as f32)
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cone => {
            let c = co_shape.as_cone().unwrap();
            Vec3::new(c.radius as f32, c.half_height as f32, c.radius as f32)
        }
        #[cfg(feature = "dim3")]
        ShapeType::RoundCone => {
            let c = &co_shape.as_round_cone().unwrap().inner_shape;
            Vec3::new(c.radius as f32, c.half_height as f32, c.radius as f32)
        }
        _ => Vec3::ONE,
    }
}

#[cfg(feature = "dim2")]
fn generate_collider_mesh(co_shape: &dyn Shape) -> Option<Mesh> {
    let mesh = match co_shape.shape_type() {
        ShapeType::Capsule => {
            let capsule = co_shape.as_capsule().unwrap();
            bevy_mesh_from_polyline(capsule.to_polyline(10))
        }
        ShapeType::Triangle => {
            let tri = co_shape.as_triangle().unwrap();
            bevy_mesh_from_polyline(vec![tri.a, tri.b, tri.c])
        }
        ShapeType::TriMesh => {
            let trimesh = co_shape.as_trimesh().unwrap();
            let vertices = trimesh
                .vertices()
                .iter()
                .map(|p| point![p.x, p.y, 0.0])
                .collect();
            bevy_mesh((vertices, trimesh.indices().to_vec()))
        }
        ShapeType::Polyline => {
            let polyline = co_shape.as_polyline().unwrap();
            bevy_polyline((
                polyline.vertices().to_vec(),
                Some(polyline.indices().to_vec()),
            ))
        }
        ShapeType::HeightField => {
            let heightfield = co_shape.as_heightfield().unwrap();
            let vertices: Vec<_> = heightfield
                .segments()
                .flat_map(|s| vec![s.a, s.b])
                .collect();
            bevy_polyline((vertices, None))
        }
        ShapeType::ConvexPolygon => {
            let poly = co_shape.as_convex_polygon().unwrap();
            bevy_mesh_from_polyline(poly.points().to_vec())
        }
        ShapeType::RoundConvexPolygon => {
            let poly = co_shape.as_round_convex_polygon().unwrap();
            bevy_mesh_from_polyline(poly.inner_shape.points().to_vec())
        }
        _ => return None,
    };

    Some(mesh)
}

#[cfg(feature = "dim3")]
fn generate_collider_mesh(co_shape: &dyn Shape) -> Option<Mesh> {
    let mesh = match co_shape.shape_type() {
        ShapeType::Capsule => {
            let capsule = co_shape.as_capsule().unwrap();
            bevy_mesh(capsule.to_trimesh(20, 10))
        }
        ShapeType::Triangle => {
            let tri = co_shape.as_triangle().unwrap();
            bevy_mesh((vec![tri.a, tri.b, tri.c], vec![[0, 1, 2], [0, 2, 1]]))
        }
        ShapeType::TriMesh => {
            let trimesh = co_shape.as_trimesh().unwrap();
            bevy_mesh((trimesh.vertices().to_vec(), trimesh.indices().to_vec()))
        }
        ShapeType::HeightField => {
            let heightfield = co_shape.as_heightfield().unwrap();
            bevy_mesh(heightfield.to_trimesh())
        }
        ShapeType::ConvexPolyhedron => {
            let poly = co_shape.as_convex_polyhedron().unwrap();
            bevy_mesh(poly.to_trimesh())
        }
        ShapeType::RoundConvexPolyhedron => {
            let poly = co_shape.as_round_convex_polyhedron().unwrap();
            bevy_mesh(poly.inner_shape.to_trimesh())
        }
        _ => return None,
    };

    Some(mesh)
}
