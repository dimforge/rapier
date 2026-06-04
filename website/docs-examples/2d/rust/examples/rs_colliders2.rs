use nalgebra::{Isometry2, UnitComplex};
use rapier2d::{parry::transformation::voxelization::FillMode, prelude::*};

fn main() {
    let vertices = vec![point![-1.0, -1.0], point![1.0, -1.0], point![1.0, 1.0]];
    let indices = vec![[0, 2, 1]];
    let heights = nalgebra::DVector::<Real>::from_vec(vec![0.0, 1.0, 0.5, 0.9]);
    let scale = Vector::new(1.0, 1.0);

    // DOCUSAURUS: Creation start
    use rapier2d::prelude::*;
    use std::f32::consts::PI;

    // The set that will contain our colliders.
    let mut collider_set = ColliderSet::new();

    // Builder for a ball-shaped collider.
    let _ = ColliderBuilder::ball(0.5);
    // Builder for a cuboid-shaped collider.
    let _ = ColliderBuilder::cuboid(0.5, 0.2);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `x` coordinate axis.
    let _ = ColliderBuilder::capsule_x(0.5, 0.2);
    // Builder for a capsule-shaped collider. The capsule principal axis is the `y` coordinate axis.
    let _ = ColliderBuilder::capsule_y(0.5, 0.2);
    // Builder for a triangle-mesh-shaped collider.
    let _ = ColliderBuilder::trimesh(vertices, indices);
    // Builder for a heightfield-shaped collider.
    let _ = ColliderBuilder::heightfield(heights, scale);
    // Builder for a collider with the given shape.
    let collider = ColliderBuilder::new(SharedShape::ball(0.5))
        // The collider translation wrt. the body it is attached to.
        // Default: the zero vector.
        .translation(vector![1.0, 2.0])
        // The collider rotation wrt. the body it is attached to.
        // Default: the identity rotation.
        .rotation(PI)
        // The collider position wrt. the body it is attached to.
        // Default: the identity isometry.
        .position(Isometry::new(vector![1.0, 2.0], PI))
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

    // Insert the collider into the set, without attaching it to a rigid-body.
    let collider_handle = collider_set.insert(collider.clone());

    let mut rigid_body_set = RigidBodySet::new();
    let rigid_body_handle = rigid_body_set.insert(RigidBodyBuilder::dynamic().build());
    // Or insert the collider into the set and attach it to a rigid-body.
    let handle = collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Creation stop

    // DOCUSAURUS: ColliderType1 start
    /* Set the collider type when the collider is created. */
    let collider = ColliderBuilder::ball(0.5).sensor(true).build();
    // DOCUSAURUS: ColliderType1 stop

    // DOCUSAURUS: ColliderType2 start
    /* Set the collider type after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_sensor(true);
    assert!(collider.is_sensor());
    // DOCUSAURUS: ColliderType2 stop

    // DOCUSAURUS: VoxelsPoints start
    // A voxels shape from arbitrary points
    let shape = ColliderBuilder::voxels_from_points(
        Vector::new(1.0, 1.0),
        &[point![0.0, 0.0], point![1.0, 1.0], point![-1.0, 1.0]],
    );
    // DOCUSAURUS: VoxelsPoints stop

    let mesh = vec![point![0.0, 0.0], point![0.0, 10.0]];
    let indices: Vec<_> = (0..mesh.len() as u32)
        .map(|i| [i, (i + 1) % mesh.len() as u32])
        .collect();
    // DOCUSAURUS: VoxelsMesh start
    let shape = SharedShape::voxelized_mesh(&mesh, &indices, 0.2, FillMode::default());
    // DOCUSAURUS: VoxelsMesh stop

    let shape = ColliderBuilder::ball(0.5).shape;
    let pos1 = Isometry2::translation(0.0, 1.0);
    let pos2 = Isometry2::translation(0.0, 1.0);
    // DOCUSAURUS: Compound start
    let _ = ColliderBuilder::compound(vec![(pos1, shape.clone()), (pos2, shape.clone())]);
    // DOCUSAURUS: Compound stop

    // DOCUSAURUS: Mass start
    let rigid_body = RigidBodyBuilder::dynamic().build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    // First option: by setting the density of the collider (or we could just leave
    //               its default value 1.0).
    let collider = ColliderBuilder::cuboid(1.0, 2.0).density(2.0).build();
    // Second option: by setting the mass of the collider.
    let collider = ColliderBuilder::cuboid(1.0, 2.0).mass(0.8).build();
    // Third option: by setting the mass-properties explicitly.
    let collider = ColliderBuilder::cuboid(1.0, 2.0)
        .mass_properties(MassProperties::new(point![0.0, 1.0], 0.5, 0.3))
        .build();
    // When the collider is attached, the rigid-body's mass and angular
    // inertia is automatically updated to take the collider into account.
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Mass stop

    // DOCUSAURUS: Position1 start
    /* Set the collider position when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .translation(vector![1.0, 2.0])
        .rotation(0.4)
        // Set both translation and rotation at once.
        .position(Isometry::new(vector![1.0, 2.0], 0.4))
        .build();
    // DOCUSAURUS: Position1 stop

    // DOCUSAURUS: Position2 start
    /* Set the collider position after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_translation(vector![1.0, 2.0]);
    collider.set_rotation(UnitComplex::new(0.4));
    // Set both the translation and rotation at once.
    collider.set_position(Isometry::new(vector![1.0, 2.0], 0.4));
    assert_eq!(*collider.translation(), vector![1.0, 2.0]);
    assert_eq!(collider.rotation().angle(), 0.4);
    // DOCUSAURUS: Position2 stop

    // DOCUSAURUS: Position3 start
    let rigid_body = RigidBodyBuilder::dynamic().build();
    let rigid_body_handle = rigid_body_set.insert(rigid_body);
    let collider = ColliderBuilder::ball(0.5)
        .translation(vector![1.0, 2.0])
        .build();
    // Attach the collider to the rigid-body. The collider's position wrt. the rigid-body
    // is automatically set to the collider current position when this method is called.
    collider_set.insert_with_parent(collider, rigid_body_handle, &mut rigid_body_set);
    // DOCUSAURUS: Position3 stop

    // DOCUSAURUS: Position4 start
    /* Set the collider position wrt. its parent after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_position_wrt_parent(Isometry::translation(1.0, 2.0));
    assert_eq!(
        collider.position_wrt_parent().unwrap().translation.vector,
        vector![1.0, 2.0]
    );
    // DOCUSAURUS: Position4 stop

    // DOCUSAURUS: Friction1 start
    /* Set the friction coefficient and friction combine rule
    when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .friction(0.7)
        .friction_combine_rule(CoefficientCombineRule::Min)
        .build();
    // DOCUSAURUS: Friction1 stop

    // DOCUSAURUS: Friction2 start
    /* Set the friction coefficient and friction combine rule
    after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_friction(0.7);
    collider.set_friction_combine_rule(CoefficientCombineRule::Min);
    assert_eq!(collider.friction(), 0.7);
    assert_eq!(
        collider.friction_combine_rule(),
        CoefficientCombineRule::Min
    );
    // DOCUSAURUS: Friction2 stop

    // DOCUSAURUS: Restitution1 start
    /* Set the restitution coefficient and restitution combine rule
    when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .restitution(0.7)
        .restitution_combine_rule(CoefficientCombineRule::Min)
        .build();
    // DOCUSAURUS: Restitution1 stop

    // DOCUSAURUS: Restitution2 start
    /* Set the restitution coefficient and restitution combine rule
    after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_restitution(0.7);
    collider.set_restitution_combine_rule(CoefficientCombineRule::Min);
    assert_eq!(collider.restitution(), 0.7);
    assert_eq!(
        collider.restitution_combine_rule(),
        CoefficientCombineRule::Min
    );
    // DOCUSAURUS: Restitution2 stop

    // DOCUSAURUS: Groups1 start
    /* Set the collision groups and solver groups when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .collision_groups(InteractionGroups::new(
            Group::GROUP_1 | Group::GROUP_3 | Group::GROUP_4,
            Group::GROUP_3,
        ))
        .solver_groups(InteractionGroups::new(
            Group::GROUP_1 | Group::GROUP_2,
            Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_4,
        ))
        .build();
    // DOCUSAURUS: Groups1 stop

    // DOCUSAURUS: Groups2 start
    /* Set the collision groups and solver groups after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_collision_groups(InteractionGroups::new(
        Group::GROUP_1 | Group::GROUP_3 | Group::GROUP_4,
        Group::GROUP_3,
    ));
    collider.set_solver_groups(InteractionGroups::new(
        Group::GROUP_1 | Group::GROUP_2,
        Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_4,
    ));
    assert_eq!(
        collider.collision_groups(),
        InteractionGroups::new(
            Group::GROUP_1 | Group::GROUP_3 | Group::GROUP_4,
            Group::GROUP_3
        )
    );
    assert_eq!(
        collider.solver_groups(),
        InteractionGroups::new(
            Group::GROUP_1 | Group::GROUP_2,
            Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_4
        )
    );
    // DOCUSAURUS: Groups2 stop

    // DOCUSAURUS: ActiveCollisionTypes1 start
    /* Set the active collision types when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .active_collision_types(
            ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_FIXED,
        )
        .build();
    // DOCUSAURUS: ActiveCollisionTypes1 stop

    // DOCUSAURUS: ActiveCollisionTypes2 start
    /* Set the active collision types after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_active_collision_types(
        ActiveCollisionTypes::default() | ActiveCollisionTypes::KINEMATIC_FIXED,
    );
    assert!(collider
        .active_collision_types()
        .contains(ActiveCollisionTypes::DYNAMIC_KINEMATIC));
    assert!(collider
        .active_collision_types()
        .contains(ActiveCollisionTypes::KINEMATIC_FIXED));
    // DOCUSAURUS: ActiveCollisionTypes2 stop

    // DOCUSAURUS: ActiveEvents1 start
    /* Set the active events when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();
    // DOCUSAURUS: ActiveEvents1 stop

    // DOCUSAURUS: ActiveEvents2 start
    /* Set the active events after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.set_active_events(ActiveEvents::COLLISION_EVENTS);
    assert!(collider
        .active_events()
        .contains(ActiveEvents::COLLISION_EVENTS));
    // DOCUSAURUS: ActiveEvents2 stop

    // DOCUSAURUS: ActiveHooks1 start
    /* Set the active hooks when the collider is created. */
    let collider = ColliderBuilder::ball(0.5)
        .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS)
        .build();
    // DOCUSAURUS: ActiveHooks1 stop

    // DOCUSAURUS: ActiveHooks2 start
    /* Set the active hooks after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider
        .set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS);
    assert!(collider
        .active_hooks()
        .contains(ActiveHooks::FILTER_CONTACT_PAIRS));
    assert!(collider
        .active_hooks()
        .contains(ActiveHooks::MODIFY_SOLVER_CONTACTS));
    // DOCUSAURUS: ActiveHooks2 stop

    // DOCUSAURUS: UserData1 start
    /* Set the user-data when the collider is created. */
    let collider = ColliderBuilder::ball(0.5).user_data(42).build();
    // DOCUSAURUS: UserData1 stop

    // DOCUSAURUS: UserData2 start
    /* Set the user-data after the collider creation. */
    let collider = collider_set.get_mut(collider_handle).unwrap();
    collider.user_data = 42;
    assert_eq!(collider.user_data, 42);
    // DOCUSAURUS: UserData2 stop
}
