fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let vertices = vec![
        Vector::new(-1.0, -1.0, 0.0),
        Vector::new(1.0, -1.0, 0.0),
        Vector::new(1.0, 1.0, 0.0),
    ];
    let indices = vec![[0, 2, 1]];
    let heights = Array2::new(2, 2, vec![0.0, 1.0, 0.5, 0.0]);
    let scale = Vector::new(1.0, 1.0, 1.0);
    // DOCUSAURUS: Creation start
    use rapier3d::prelude::*;
    use std::f32::consts::PI;

    // The set that will contain our colliders.
    let mut collider_set = ColliderSet::new();

    // Builder for a ball-shaped collider.
    let _ = ColliderBuilder::ball(0.5);
    // Builder for a cuboid-shaped collider.
    let _ = ColliderBuilder::cuboid(0.5, 0.2, 0.1);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `x` coordinate axis.
    let _ = ColliderBuilder::capsule_x(0.5, 0.2);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    let _ = ColliderBuilder::capsule_y(0.5, 0.2);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `z` coordinate axis.
    let _ = ColliderBuilder::capsule_z(0.5, 0.2);
    // Builder for a triangle-mesh-shaped collider.
    let _ = ColliderBuilder::trimesh(vertices, indices);
    // Builder for a heightfield-shaped collider.
    let _ = ColliderBuilder::heightfield(heights, scale);
    // Builder for a collider with the given shape.
    let collider = ColliderBuilder::new(SharedShape::ball(0.5))
        // The collider translation wrt. the body it is attached to.
        // Default: the zero vector.
        .translation(Vector::new(1.0, 2.0, 3.0))
        // The collider rotation wrt. the body it is attached to.
        // Default: the identity rotation.
        .rotation(Vector::new(0.0, PI, 0.0))
        // The collider position wrt. the body it is attached to.
        // Default: the identity isometry.
        .position(Pose::new(Vector::new(1.0, 2.0, 3.0), Vector::new(0.0, PI, 0.0)))
        // The collider density. If non-zero the collider's mass and angular inertia will be added
        // to the inertial properties of the body it is attached to.
        // Default: 1.0
        .density(1.3)
        // The friction coefficient of this collider.
        // Default: ColliderBuilder::default_friction() == 0.5
        .friction(0.8)
        // Whether this collider is a sensor.
        // Default: false
        .sensor(true)
        // All done, actually build the collider.
        .build();

    // Insert the collider into the set, without attaching to a rigid-body.
    let handle = collider_set.insert(collider.clone());
    let rigid_body_handle = rigid_body_set.insert(RigidBodyBuilder::dynamic().build());
    // Or insert the collider into the set and attach it to a rigid-body.
    let handle = collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: VoxelsPoints start
    // A voxels shape from arbitrary points
    let shape = ColliderBuilder::voxels_from_points(
        Vector::new(1.0, 1.0, 1.0),
        &[Vector::new(0.0, 0.0, 0.0), Vector::new(1.0, 1.0, 1.0)],
    );
    // DOCUSAURUS: VoxelsPoints stop

    // DOCUSAURUS: Mass start
    let rigid_body = RigidBodyBuilder::dynamic().build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let collider = ColliderBuilder::cuboid(1.0, 2.0, 3.0).density(2.0).build();
    // Second option: by setting the mass of the collider.
    let collider = ColliderBuilder::cuboid(1.0, 2.0, 3.0).mass(0.8).build();
    // Third option: by setting the mass-properties explicitly.
    let collider = ColliderBuilder::cuboid(1.0, 2.0, 3.0)
        .mass_properties(MassProperties::new(
            Vector::new(0.0, 1.0, 0.0),
            0.5,
            Vector::new(0.3, 0.2, 0.1),
        ))
        .build();
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    let collider_handle =
        collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Mass stop

    // DOCUSAURUS: Position1 start
    /* Set the collider position when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .translation(Vector::new(1.0, 2.0, 3.0))
        .rotation(Vector::new(0.1, 0.2, 0.4))
        // Set both translation and rotation at once.
        .position(Pose::new(
            Vector::new(1.0, 2.0, 3.0),
            Vector::new(0.1, 0.2, 0.4),
        ))
        .build();
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the collider position after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_translation(Vector::new(1.0, 2.0, 3.0));
    collider.set_rotation(Rotation::from_scaled_axis(Vector::new(0.1, 0.2, 0.4)));
    // Set both the translation and rotation at once.
    collider.set_position(Pose::new(
        Vector::new(1.0, 2.0, 3.0),
        Vector::new(0.1, 0.2, 0.4),
    ));
    assert_eq!(collider.translation(), Vector::new(1.0, 2.0, 3.0));
    assert_eq!(
        collider.rotation().to_scaled_axis(),
        Vector::new(0.1, 0.2, 0.4)
    );
    // DOCUSAURUS: Position2 stop

    // DOCUSAURUS: Position3 start
    let rigid_body = RigidBodyBuilder::dynamic().build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    let collider = ColliderBuilder::ball(0.5)
        .translation(Vector::new(1.0, 2.0, 3.0))
        .build();
    // Attach the collider to the rigid-body. The collider's position wrt. the rigid-body
    // is automatically set to the collider current position when this method is called.
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Position3 stop

    // DOCUSAURUS: Position4 start
    /* Set the collider position wrt. its parent after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_position_wrt_parent(Pose::translation(1.0, 2.0, 3.0));
    assert_eq!(
        collider.position_wrt_parent().unwrap().translation,
        Vector::new(1.0, 2.0, 3.0)
    );
    // DOCUSAURUS: Position4 stop
}
