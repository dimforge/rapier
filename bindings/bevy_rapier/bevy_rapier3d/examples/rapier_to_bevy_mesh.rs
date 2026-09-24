use bevy::{
    pbr::wireframe::{WireframeConfig, WireframePlugin},
    prelude::*,
};
use bevy_rapier3d::geometry::to_bevy_mesh::ToMeshBuilder;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor::default())
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
            WireframePlugin::default(),
        ))
        .add_systems(Startup, (setup_graphics, setup_physics))
        .add_systems(Update, (toggle_wireframe, toggle_dynamic, rotate))
        .run();
}

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
    ));
}

pub fn setup_physics(
    mut commands: Commands,
    mut assets_meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 0.1;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
    ));
    // light
    commands.spawn((
        PointLight {
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 8.0),
    ));
    /*
     * Create the shapes
     */
    let rad = 0.5;

    let colors = [
        Hsla::hsl(220.0, 1.0, 0.3),
        Hsla::hsl(180.0, 1.0, 0.3),
        Hsla::hsl(260.0, 1.0, 0.7),
    ];

    let colliders: Vec<Collider> = vec![
        Collider::cuboid(rad, rad, rad),
        Collider::ball(rad),
        Collider::capsule_y(rad, rad),
        Collider::cone(rad, rad),
        Collider::cylinder(rad, rad),
        Collider::trimesh(
            vec![
                [-0.5, -0.5, -0.5].into(), // Vertex 0
                [0.5, -0.5, -0.5].into(),  // Vertex 1
                [0.5, 0.5, -0.5].into(),   // Vertex 2
                [-0.5, 0.5, -0.5].into(),  // Vertex 3
                [-0.5, -0.5, 0.5].into(),  // Vertex 4
                [0.5, -0.5, 0.5].into(),   // Vertex 5
                [0.5, 0.5, 0.5].into(),    // Vertex 6
                [-0.5, 0.5, 0.5].into(),   // Vertex 7
            ],
            vec![
                // Back face
                [0, 2, 1],
                [0, 3, 2],
                // Front face
                [4, 5, 6],
                [4, 6, 7],
                // Left face
                [0, 4, 7],
                [0, 7, 3],
                // Right face
                [1, 6, 5],
                [1, 2, 6],
                // Bottom face
                [0, 1, 5],
                [0, 5, 4],
                // Top face
                [3, 6, 2],
                [3, 7, 6],
            ],
        )
        .unwrap(),
        // Unsupported shapes
        Collider::round_cylinder(rad, rad, rad / 10.0),
        Collider::segment(Vec3::new(-rad, 0.0, 0.0), Vec3::new(rad, 0.0, 0.0)),
        Collider::triangle(
            Vec3::new(-rad, -rad, 0.0),
            Vec3::new(rad, -rad, 0.0),
            Vec3::new(0.0, rad, 0.0),
        ),
        Collider::heightfield(
            vec![0.0, 0.0, 0.0, rad, rad, rad, 0.0, 0.0, 0.0],
            3,
            3,
            Vec3::new(rad, rad, rad),
        ),
    ];
    let material = MeshMaterial3d(materials.add(Color::WHITE));
    let colliders_count = colliders.len();
    for (i, collider) in colliders.iter().enumerate() {
        fn get_coordinates(id: usize, column_size: usize) -> Vec3 {
            let x = id / column_size;
            let y = id % column_size;
            let z = 0;
            Vec3::new(x as f32, y as f32, z as f32)
        }
        let column_height = 3;
        // Get int coordinates from index
        let coordinates = get_coordinates(i, column_height);
        // Center the coordinates in x
        let columns = colliders_count / column_height;
        let coordinates = coordinates - Vec3::new(columns as f32 / 2.0, 0.0, 0.0);
        // more space between shapes
        let coordinates = coordinates * (1.0 + rad);
        // Shift up
        let coordinates = coordinates + Vec3::new(0.0, rad + 5.0, 0.0);

        let mut entity = commands.spawn((
            Transform::from_translation(coordinates),
            RigidBody::Fixed,
            collider.clone(),
            ColliderDebugColor(colors[i % 3]),
        ));
        // Transform the parry shape into a bevy mesh.
        let mesh = Mesh::try_from(collider);
        if let Ok(mut mesh) = mesh {
            mesh.compute_normals();
            entity.insert((Mesh3d(assets_meshes.add(mesh)), material.clone()));
        }
    }
    // Alternatively, you can call [`Collider::as_typed_shape`] of `Collider::as_[specificShape]`
    // to retrieve a specialized builder through [`ToMeshBuilder::mesh_builder`]
    let mut mesh = colliders[1].as_ball().unwrap().raw.mesh_builder().build();
    let mut entity = commands.spawn((
        Transform::from_translation(Vec3::new(0.0, 1.0 + rad, 0.0)),
        RigidBody::Fixed,
        colliders[1].clone(),
        ColliderDebugColor(colors[42 % 3]),
    ));
    mesh.compute_normals();
    entity.insert((Mesh3d(assets_meshes.add(mesh)), material.clone()));
}

fn toggle_wireframe(
    mut wireframe_config: ResMut<WireframeConfig>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        wireframe_config.global = !wireframe_config.global;
    }
}

fn toggle_dynamic(mut bodies: Query<&mut RigidBody>, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::KeyD) {
        for mut body in &mut bodies.iter_mut() {
            if body.is_dynamic() {
                *body = RigidBody::Fixed;
            } else {
                *body = RigidBody::Dynamic;
            }
        }
    }
}

fn rotate(mut query: Query<&mut Transform, With<RigidBody>>, time: Res<Time>) {
    for mut transform in &mut query {
        transform.rotate_y(time.delta_secs() / 2.);
        transform.rotate_x(time.delta_secs() / 2.6);
    }
}
