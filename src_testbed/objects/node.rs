use bevy::prelude::*;
use bevy::render::mesh::{Indices, VertexAttributeValues};

//use crate::objects::plane::Plane;
use na::{point, Point3, Vector3};
use std::collections::HashMap;

use bevy::render::pipeline::PrimitiveTopology;
use bevy::render::wireframe::Wireframe;
use rapier::geometry::{ColliderHandle, ColliderSet, Shape, ShapeType};
#[cfg(feature = "dim3")]
use rapier::geometry::{Cone, Cylinder};
use rapier::math::Isometry;
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
    pub collider: ColliderHandle,
    pub delta: Isometry<f32>,
    material: Handle<StandardMaterial>,
}

impl EntityWithGraphics {
    pub fn spawn(
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        prefab_meshs: &HashMap<ShapeType, Handle<Mesh>>,
        shape: &dyn Shape,
        collider: ColliderHandle,
        collider_pos: Isometry<f32>,
        delta: Isometry<f32>,
        color: Point3<f32>,
        sensor: bool,
    ) -> Self {
        let entity = commands.spawn().id();

        let scale = collider_mesh_scale(shape);
        let mesh = prefab_meshs
            .get(&shape.shape_type())
            .cloned()
            .or_else(|| generate_collider_mesh(shape).map(|m| meshes.add(m)))
            .expect("Could not build the collider's render mesh");

        let bevy_color = Color::rgb(color.x, color.y, color.z);
        let shape_pos = collider_pos * delta;
        let mut transform = Transform::from_scale(scale);
        transform.translation.x = shape_pos.translation.vector.x;
        transform.translation.y = shape_pos.translation.vector.y;
        #[cfg(feature = "dim3")]
        {
            transform.translation.z = shape_pos.translation.vector.z;
            transform.rotation = Quat::from_xyzw(
                shape_pos.rotation.i,
                shape_pos.rotation.j,
                shape_pos.rotation.k,
                shape_pos.rotation.w,
            );
        }
        #[cfg(feature = "dim2")]
        {
            if sensor {
                transform.translation.z = -10.0;
            }
            transform.rotation = Quat::from_rotation_z(shape_pos.rotation.angle());
        }

        let material = StandardMaterial {
            metallic: 0.5,
            roughness: 0.5,
            double_sided: true, // TODO: this doesn't do anything?
            ..StandardMaterial::from(bevy_color)
        };
        let material_handle = materials.add(material);
        let material_weak_handle = material_handle.clone_weak();
        let pbr = PbrBundle {
            mesh,
            material: material_handle,
            transform,
            ..Default::default()
        };

        let mut entity_commands = commands.entity(entity);
        entity_commands.insert_bundle(pbr);

        if sensor {
            entity_commands.insert(Wireframe);
        }

        EntityWithGraphics {
            entity,
            color,
            base_color: color,
            collider,
            delta,
            material: material_weak_handle,
        }
    }

    pub fn despawn(&mut self, commands: &mut Commands) {
        //FIXME: Should this be despawn_recursive?
        commands.entity(self.entity).despawn();
    }
    pub fn select(&mut self, materials: &mut Assets<StandardMaterial>) {
        // NOTE: we don't just call `self.set_color` because that would
        //       overwrite self.base_color too.
        self.color = point![1.0, 0.0, 0.0];
        if let Some(material) = materials.get_mut(&self.material) {
            material.base_color = Color::rgb(self.color.x, self.color.y, self.color.z);
        }
    }

    pub fn unselect(&mut self, materials: &mut Assets<StandardMaterial>) {
        self.set_color(materials, self.base_color);
    }

    pub fn set_color(&mut self, materials: &mut Assets<StandardMaterial>, color: Point3<f32>) {
        if let Some(material) = materials.get_mut(&self.material) {
            material.base_color = Color::rgb(color.x, color.y, color.z);
        }
        self.color = color;
        self.base_color = color;
    }

    pub fn update(&mut self, colliders: &ColliderSet, components: &mut Query<(&mut Transform,)>) {
        if let Some(co) = colliders.get(self.collider) {
            if let Ok(mut pos) = components.get_component_mut::<Transform>(self.entity) {
                let co_pos = co.position() * self.delta;
                pos.translation.x = co_pos.translation.vector.x;
                pos.translation.y = co_pos.translation.vector.y;
                #[cfg(feature = "dim3")]
                {
                    pos.translation.z = co_pos.translation.vector.z;
                    pos.rotation = Quat::from_xyzw(
                        co_pos.rotation.i,
                        co_pos.rotation.j,
                        co_pos.rotation.k,
                        co_pos.rotation.w,
                    );
                }
                #[cfg(feature = "dim2")]
                {
                    pos.rotation = Quat::from_rotation_z(co_pos.rotation.angle());
                }
            }
        }
    }

    pub fn object(&self) -> ColliderHandle {
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
        out.insert(ShapeType::Cuboid, meshes.add(cuboid));

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
        let cuboid = Mesh::from(shape::Cube { size: 2.0 });
        out.insert(ShapeType::Cuboid, meshes.add(cuboid));

        //
        // Ball mesh
        //
        let ball = Mesh::from(shape::Icosphere {
            subdivisions: 2,
            radius: 1.0,
        });
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
fn bevy_mesh_from_polyline(vertices: Vec<Point2<f32>>) -> Mesh {
    let n = vertices.len();
    let idx = (1..n as u32 - 1).map(|i| [0, i, i + 1]).collect();
    let vtx = vertices
        .into_iter()
        .map(|v| Point3::new(v.x, v.y, 0.0))
        .collect();
    bevy_mesh((vtx, idx))
}

#[cfg(feature = "dim2")]
fn bevy_polyline(buffers: (Vec<Point2<f32>>, Option<Vec<[u32; 2]>>)) -> Mesh {
    let (vtx, idx) = buffers;
    // let mut normals: Vec<[f32; 3]> = vec![];
    let mut vertices: Vec<[f32; 3]> = vec![];

    if let Some(idx) = idx {
        for idx in idx {
            let a = vtx[idx[0] as usize];
            let b = vtx[idx[1] as usize];

            vertices.push([a.x, a.y, 0.0]);
            vertices.push([b.x, b.y, 0.0]);
        }
    } else {
        vertices = vtx.iter().map(|v| [v.x, v.y, 0.0]).collect();
    }

    let indices: Vec<_> = (0..vertices.len() as u32).collect();
    let uvs: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0]).collect();
    let normals: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0, 1.0]).collect();

    // Generate the mesh
    let mut mesh = Mesh::new(PrimitiveTopology::LineStrip);
    mesh.set_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices),
    );
    mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.set_indices(Some(Indices::U32(indices)));
    mesh
}

fn bevy_mesh(buffers: (Vec<Point3<f32>>, Vec<[u32; 3]>)) -> Mesh {
    let (vtx, idx) = buffers;
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut vertices: Vec<[f32; 3]> = vec![];

    for idx in idx {
        let a = vtx[idx[0] as usize];
        let b = vtx[idx[1] as usize];
        let c = vtx[idx[2] as usize];

        vertices.push(a.into());
        vertices.push(b.into());
        vertices.push(c.into());
    }

    for vtx in vertices.chunks(3) {
        let a = Point3::from(vtx[0]);
        let b = Point3::from(vtx[1]);
        let c = Point3::from(vtx[2]);
        let n = (b - a).cross(&(c - a)).normalize();
        normals.push(n.into());
        normals.push(n.into());
        normals.push(n.into());
    }

    normals
        .iter_mut()
        .for_each(|n| *n = Vector3::from(*n).normalize().into());
    let indices: Vec<_> = (0..vertices.len() as u32).collect();
    let uvs: Vec<_> = (0..vertices.len()).map(|_| [0.0, 0.0]).collect();

    // Generate the mesh
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::from(vertices),
    );
    mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, VertexAttributeValues::from(normals));
    mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, VertexAttributeValues::from(uvs));
    mesh.set_indices(Some(Indices::U32(indices)));
    mesh
}

fn collider_mesh_scale(co_shape: &dyn Shape) -> Vec3 {
    match co_shape.shape_type() {
        #[cfg(feature = "dim2")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::new(c.half_extents.x, c.half_extents.y, 1.0)
        }
        ShapeType::Ball => {
            let b = co_shape.as_ball().unwrap();
            Vec3::new(b.radius, b.radius, b.radius)
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cuboid => {
            let c = co_shape.as_cuboid().unwrap();
            Vec3::from_slice_unaligned(c.half_extents.as_slice())
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cylinder => {
            let c = co_shape.as_cylinder().unwrap();
            Vec3::new(c.radius, c.half_height, c.radius)
        }
        #[cfg(feature = "dim3")]
        ShapeType::RoundCylinder => {
            let c = &co_shape.as_round_cylinder().unwrap().base_shape;
            Vec3::new(c.radius, c.half_height, c.radius)
        }
        #[cfg(feature = "dim3")]
        ShapeType::Cone => {
            let c = co_shape.as_cone().unwrap();
            Vec3::new(c.radius, c.half_height, c.radius)
        }
        #[cfg(feature = "dim3")]
        ShapeType::RoundCone => {
            let c = &co_shape.as_round_cone().unwrap().base_shape;
            Vec3::new(c.radius, c.half_height, c.radius)
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
            bevy_mesh_from_polyline(poly.base_shape.points().to_vec())
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
            bevy_mesh(poly.base_shape.to_trimesh())
        }
        _ => return None,
    };

    Some(mesh)
}
