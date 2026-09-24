//! Tests of the URDF, MJCF and mesh loaders.

use bevy::prelude::*;
#[cfg(any(feature = "urdf", feature = "mjcf"))]
use bevy::time::{TimePlugin, TimeUpdateStrategy};

use crate::prelude::*;

/// An app with the given physics plugins, running the physics at a fixed 60Hz timestep.
#[cfg(any(feature = "urdf", feature = "mjcf"))]
fn test_app<M>(plugins: impl bevy::app::Plugins<M>) -> App {
    let mut app = App::new();
    app.add_plugins((TransformPlugin, TimePlugin));
    app.add_plugins(plugins);
    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 60.0,
        substeps: 1,
    });
    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        std::time::Duration::from_secs_f32(1.0 / 60.0),
    ));
    app.finish();
    app
}

/// The world-space translation of a rigid-body in the default context.
#[cfg(any(feature = "urdf", feature = "mjcf"))]
fn body_translation(app: &mut App, entity: Entity) -> Vect {
    let bodies = app
        .world_mut()
        .query::<&RapierRigidBodySet>()
        .single(app.world())
        .unwrap();
    let handle = bodies.entity2body()[&entity];
    bodies.bodies[handle].translation()
}

#[cfg(feature = "urdf")]
mod urdf {
    use super::*;
    use crate::loaders::urdf::*;
    use std::path::Path;

    const ARM: &str = r#"
<robot name="arm">
  <link name="world"/>
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0 0 0.25"/>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
    <visual><geometry><cylinder radius="0.05" length="0.5"/></geometry></visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry><cylinder radius="0.05" length="0.5"/></geometry>
    </collision>
  </link>
  <link name="finger">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <joint name="anchor" type="fixed">
    <parent link="world"/><child link="base"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0.1" rpy="0.5 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
    <dynamics damping="0.3" friction="0.2"/>
  </joint>
  <joint name="finger_joint" type="revolute">
    <parent link="arm"/><child link="finger"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
    <mimic joint="shoulder" multiplier="2" offset="0.1"/>
  </joint>
</robot>
"#;

    fn load() -> UrdfModel {
        UrdfModel::from_str(ARM, UrdfLoaderOptions::default(), Path::new(".")).unwrap()
    }

    #[test]
    fn empty_links_are_tracked() {
        let model = load();
        // The empty `world` link is removed, and `base` is fixed in its place.
        assert_eq!(model.robot.links.len(), 3);
        let names: Vec<_> = (0..3)
            .map(|i| model.urdf_link(i).unwrap().name.as_str())
            .collect();
        assert_eq!(names, ["base", "arm", "finger"]);
        let joints: Vec<_> = (0..model.robot.joints.len())
            .map(|i| model.urdf_joint(i).unwrap().name.as_str())
            .collect();
        assert_eq!(joints, ["shoulder", "finger_joint"]);
    }

    /// `mount` is an empty link between `hinge` and `mount_joint`, which are merged when it is
    /// removed.
    const MOUNTED: &str = r#"
<robot name="mounted">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="mount"/>
  <link name="arm">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="finger">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="mount"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
    <dynamics damping="0.3" friction="0.2"/>
  </joint>
  <joint name="mount_joint" type="fixed">
    <origin xyz="0 0 0.5"/>
    <parent link="mount"/><child link="arm"/>
  </joint>
  <joint name="finger_joint" type="revolute">
    <parent link="arm"/><child link="finger"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
    <mimic joint="hinge" multiplier="2"/>
  </joint>
</robot>
"#;

    #[test]
    fn merged_joints_are_tracked() {
        let model =
            UrdfModel::from_str(MOUNTED, UrdfLoaderOptions::default(), Path::new(".")).unwrap();
        let names: Vec<_> = (0..model.robot.links.len())
            .map(|i| model.urdf_link(i).unwrap().name.as_str())
            .collect();
        assert_eq!(names, ["base", "arm", "finger"]);
        assert_eq!(model.urdf_joint(0).unwrap().name, "mount_joint");
        assert_eq!(model.source_urdf_joint(0).unwrap().name, "hinge");
        assert_eq!(model.source_urdf_joint(1).unwrap().name, "finger_joint");

        let mut app = test_app(RapierPhysicsPlugin::<NoUserData>::default());
        app.update();
        let robot = spawn_urdf_robot(
            &mut app.world_mut().commands(),
            &model,
            &UrdfSpawnOptions {
                multibody: true,
                ..default()
            },
        );
        app.world_mut().flush();

        let arm = robot.links_by_name["arm"];
        assert_eq!(robot.joints_by_name["hinge"], arm);
        assert_eq!(robot.joints_by_name["mount_joint"], arm);
        assert_eq!(app.world().get::<UrdfLinkId>(arm), Some(&UrdfLinkId(2)));
        // The merged joint gets the dynamics of the revolute joint, which drives the mimic.
        let damping = app.world().get::<MultibodyJointDamping>(arm).unwrap();
        assert_eq!(damping.0[JointAxis::AngX as usize], 0.3);
        let finger = robot.links_by_name["finger"];
        let couplings = app.world().get::<MultibodyJointCouplings>(finger).unwrap();
        assert_eq!(couplings.0[0].source, arm);
    }

    fn spawn(multibody: bool) -> (App, SpawnedUrdfRobot) {
        let model = load();
        let mut app = test_app(RapierPhysicsPlugin::<NoUserData>::default());
        app.update();
        let robot = spawn_urdf_robot(
            &mut app.world_mut().commands(),
            &model,
            &UrdfSpawnOptions {
                multibody,
                root_transform: Transform::from_xyz(1.0, 2.0, 3.0),
                ..default()
            },
        );
        app.world_mut().flush();
        (app, robot)
    }

    fn check_robot(app: &mut App, robot: &SpawnedUrdfRobot) {
        assert_eq!(robot.links.len(), 3);
        assert_eq!(robot.joints.len(), 2);
        assert_eq!(robot.joints_by_name["shoulder"], robot.links_by_name["arm"]);
        for (link, colliders) in robot.links.iter().zip(&robot.colliders) {
            assert!(app.world().get::<RapierRigidBodyHandle>(*link).is_some());
            assert_eq!(colliders.len(), 1);
            assert!(app
                .world()
                .get::<RapierColliderHandle>(colliders[0])
                .is_some());
        }
        let base = robot.links_by_name["base"];
        let arm = robot.links_by_name["arm"];
        assert_eq!(app.world().get::<RigidBody>(base), Some(&RigidBody::Fixed));
        assert_eq!(app.world().get::<RigidBody>(arm), Some(&RigidBody::Dynamic));
        assert!(app.world().get::<UrdfLinkVisuals>(arm).is_some());
        assert_eq!(
            app.world().get::<Name>(arm).map(|n| n.as_str()),
            Some("arm")
        );

        // The arm swings around its joint, which stays attached to the base.
        let base_pos = body_translation(app, base);
        assert!((base_pos - Vect::new(1.0, 2.0, 3.0)).length() < 1.0e-5);
        let arm_pos0 = body_translation(app, arm);
        assert!((arm_pos0 - base_pos - Vect::new(0.0, 0.0, 0.1)).length() < 1.0e-4);
        for _ in 0..30 {
            app.update();
        }
        let arm_pos = body_translation(app, arm);
        assert!((arm_pos - arm_pos0).length() < 1.0e-2);
        let finger_pos = body_translation(app, robot.links_by_name["finger"]);
        assert!(((finger_pos - arm_pos).length() - 0.5).abs() < 1.0e-2);
        assert!(finger_pos.is_finite());
    }

    #[test]
    fn spawn_with_impulse_joints() {
        let (mut app, robot) = spawn(false);
        for _ in 0..2 {
            app.update();
        }
        for joint in &robot.joints {
            assert!(app
                .world()
                .get::<RapierImpulseJointHandle>(*joint)
                .is_some());
            assert!(app.world().get::<MultibodyJoint>(*joint).is_none());
        }
        check_robot(&mut app, &robot);
    }

    #[test]
    fn spawn_with_multibody_joints() {
        let (mut app, robot) = spawn(true);
        for _ in 0..2 {
            app.update();
        }
        for joint in &robot.joints {
            assert!(app
                .world()
                .get::<RapierMultibodyJointHandle>(*joint)
                .is_some());
        }
        let shoulder = robot.joints_by_name["shoulder"];
        let damping = app.world().get::<MultibodyJointDamping>(shoulder).unwrap();
        assert_eq!(damping.0[JointAxis::AngX as usize], 0.3);
        let friction = app.world().get::<MultibodyJointFriction>(shoulder).unwrap();
        assert_eq!(friction.0[JointAxis::AngX as usize], 0.2);

        let finger = robot.joints_by_name["finger_joint"];
        let couplings = app.world().get::<MultibodyJointCouplings>(finger).unwrap();
        assert_eq!(
            couplings.0,
            vec![MultibodyJointCoupling::new(
                JointAxis::AngX,
                shoulder,
                JointAxis::AngX,
                2.0,
                0.1
            )]
        );
        check_robot(&mut app, &robot);

        // The mimic coupling holds.
        let coord = |app: &App, e| app.world().get::<MultibodyJointState>(e).unwrap().coords[3];
        let (q1, q2) = (coord(&app, shoulder), coord(&app, finger));
        assert!(q1.abs() > 1.0e-3);
        assert!((q2 - (2.0 * q1 + 0.1)).abs() < 1.0e-2);
    }

    #[test]
    fn spawn_sample_robot() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robots/arm.urdf");
        let model = UrdfModel::from_file(path, UrdfLoaderOptions::default(), None).unwrap();
        for multibody in [true, false] {
            let mut app = test_app(RapierPhysicsPlugin::<NoUserData>::default());
            app.update();
            let robot = spawn_urdf_robot(
                &mut app.world_mut().commands(),
                &model,
                &UrdfSpawnOptions {
                    multibody,
                    ..default()
                },
            );
            app.world_mut().flush();
            // The empty `world` and `tool_center_point` links are removed.
            assert_eq!(robot.links.len(), 7);
            assert_eq!(robot.joints.len(), 6);
            assert!(!robot.links_by_name.contains_key("world"));
            for _ in 0..30 {
                app.update();
            }
            for link in &robot.links {
                assert!(body_translation(&mut app, *link).is_finite());
            }
            let fingers = robot.joints_by_name["right_finger_joint"];
            assert_eq!(
                app.world()
                    .get::<MultibodyJointCouplings>(fingers)
                    .is_some(),
                multibody
            );
        }
    }

    #[test]
    fn despawning_the_root_removes_the_robot() {
        let (mut app, robot) = spawn(true);
        app.update();
        app.world_mut().despawn(robot.root.unwrap());
        app.update();
        let bodies = app
            .world_mut()
            .query::<&RapierRigidBodySet>()
            .single(app.world())
            .unwrap();
        assert_eq!(bodies.bodies.len(), 0);
    }
}

#[cfg(feature = "mjcf")]
mod mjcf {
    use super::*;
    use crate::loaders::mjcf::*;

    const PENDULUM: &str = r#"
<mujoco model="pendulum">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="pole" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5" armature="0.01"
             frictionloss="0.1" stiffness="2"/>
      <geom name="pole_geom" type="capsule" fromto="0 0 0 0.2 0 -0.5" size="0.05" mass="1"/>
      <body name="tip" pos="0.2 0 -0.5">
        <joint name="hinge2" type="hinge" axis="0 1 0"/>
        <geom name="tip_geom" type="sphere" size="0.1" mass="0.5"/>
      </body>
    </body>
    <body name="free_ball" pos="0 0 3">
      <freejoint/>
      <geom name="ball1" type="sphere" size="0.2" mass="1"/>
    </body>
    <body name="free_ball2" pos="0.1 0 3">
      <freejoint/>
      <geom name="ball2" type="sphere" size="0.2" mass="1"/>
    </body>
  </worldbody>
  <contact>
    <exclude body1="pole" body2="tip"/>
    <exclude body1="free_ball" body2="free_ball2"/>
  </contact>
  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

    fn spawn(multibody: bool) -> (App, SpawnedMjcfModel) {
        let (robot, _) = MjcfRobot::from_str(PENDULUM, MjcfLoaderOptions::default(), ".").unwrap();
        spawn_robot(&robot, multibody)
    }

    fn spawn_robot(robot: &MjcfRobot, multibody: bool) -> (App, SpawnedMjcfModel) {
        let mut app = test_app((
            RapierPhysicsPlugin::<MjcfPhysicsHooks>::default(),
            MjcfPlugin::default(),
        ));
        app.update();
        let model = spawn_mjcf_model(
            &mut app.world_mut().commands(),
            robot,
            &MjcfSpawnOptions {
                multibody,
                ..default()
            },
        );
        app.world_mut().flush();
        // The model is Z-up.
        let mut config = app
            .world_mut()
            .query::<&mut RapierConfiguration>()
            .single_mut(app.world_mut())
            .unwrap();
        config.gravity = model.gravity;
        app.update();
        (app, model)
    }

    fn check_model(app: &mut App, model: &SpawnedMjcfModel) {
        // The world body is spawned since the pole is attached to it.
        let world = model.bodies[0].unwrap();
        assert_eq!(app.world().get::<RigidBody>(world), Some(&RigidBody::Fixed));
        let pole = model.bodies_by_name["pole"];
        let tip = model.bodies_by_name["tip"];
        assert!(app.world().get::<RapierRigidBodyHandle>(pole).is_some());
        assert!(app
            .world()
            .get::<RapierColliderHandle>(model.colliders_by_name["tip_geom"])
            .is_some());
        assert_eq!(model.gravity, Vect::new(0.0, 0.0, -9.81));

        // The contact rules are registered, with the hooks enabled on the involved colliders.
        let ball1 = model.colliders_by_name["ball1"];
        let ball2 = model.colliders_by_name["ball2"];
        let filters = app.world().resource::<MjcfContactFilters>();
        assert!(filters.excluded.contains(&(ball1, ball2)));
        assert!(filters.excluded.contains(&(
            model.colliders_by_name["tip_geom"],
            model.colliders_by_name["pole_geom"]
        )));
        assert!(app
            .world()
            .get::<ActiveHooks>(ball1)
            .unwrap()
            .contains(ActiveHooks::FILTER_CONTACT_PAIRS));

        let actuator = model.actuators_by_name["torque"];
        assert_eq!(
            app.world().get::<MjcfActuator>(actuator).unwrap().joint,
            Some(model.joints_by_name["hinge"])
        );

        let tip0 = body_translation(app, tip);
        let ball0 = body_translation(app, model.bodies_by_name["free_ball"]);
        for _ in 0..30 {
            app.update();
        }
        // The pendulum swings, and the excluded overlapping balls fall through each other.
        assert!((body_translation(app, tip) - tip0).length() > 1.0e-3);
        let ball = body_translation(app, model.bodies_by_name["free_ball"]);
        let ball2_pos = body_translation(app, model.bodies_by_name["free_ball2"]);
        assert!((ball.x - ball0.x).abs() < 1.0e-4);
        assert!(ball.z < ball0.z - 0.5);
        assert!((ball2_pos.x - 0.1).abs() < 1.0e-4);
    }

    #[test]
    fn spawn_with_multibody_joints() {
        let (mut app, model) = spawn(true);
        let hinge = model.joints_by_name["hinge"];
        assert!(app
            .world()
            .get::<RapierMultibodyJointHandle>(hinge)
            .is_some());
        assert_eq!(hinge, model.bodies_by_name["pole"]);

        let axis = JointAxis::AngX as usize;
        let world = app.world();
        assert_eq!(
            world.get::<MultibodyJointDamping>(hinge).unwrap().0[axis],
            0.5
        );
        assert_eq!(
            world.get::<MultibodyJointArmature>(hinge).unwrap().0[axis],
            0.01
        );
        assert_eq!(
            world.get::<MultibodyJointFriction>(hinge).unwrap().0[axis],
            0.1
        );
        assert_eq!(
            world.get::<MultibodyJointSprings>(hinge).unwrap().stiffness[axis],
            2.0
        );
        check_model(&mut app, &model);

        // Driving the actuator enables a force-based motor on the joint.
        let actuator = model.actuators_by_name["torque"];
        app.world_mut()
            .get_mut::<MjcfActuator>(actuator)
            .unwrap()
            .ctrl = 3.0;
        app.update();
        let joint = app.world().get::<MultibodyJoint>(hinge).unwrap();
        let motor = joint.data.as_ref().motor(JointAxis::AngX).unwrap();
        assert_eq!(motor.max_force, 3.0);
        assert_eq!(motor.model, MotorModel::ForceBased);
    }

    #[test]
    fn spawn_sample_scene() {
        for multibody in [true, false] {
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robots/cart_pole.xml");
            let (robot, _) = MjcfRobot::from_file(path, MjcfLoaderOptions::default()).unwrap();
            let (mut app, model) = spawn_robot(&robot, multibody);
            assert_eq!(model.actuators.len(), 2);
            // Zero-length timesteps (e.g. the first frame with a variable timestep) are skipped, so
            // the infinite motor forces of `rapier3d-mjcf` don't produce NaN impulse bounds.
            app.insert_resource(TimestepMode::Fixed {
                dt: 0.0,
                substeps: 1,
            });
            app.update();
            app.insert_resource(TimestepMode::Fixed {
                dt: 1.0 / 60.0,
                substeps: 1,
            });
            for actuator in &model.actuators {
                let mut actuator = app.world_mut().get_mut::<MjcfActuator>(*actuator).unwrap();
                assert!(actuator.joint.is_some());
                actuator.ctrl = 0.5;
            }
            for _ in 0..60 {
                app.update();
            }
            for body in model.bodies.iter().flatten() {
                assert!(body_translation(&mut app, *body).is_finite());
            }
        }
    }

    #[test]
    fn spawn_with_impulse_joints() {
        let (mut app, model) = spawn(false);
        let hinge = model.joints_by_name["hinge"];
        assert!(app.world().get::<RapierImpulseJointHandle>(hinge).is_some());
        assert!(app.world().get::<MultibodyJointDamping>(hinge).is_none());
        check_model(&mut app, &model);
    }
}

#[cfg(feature = "meshloader")]
mod meshloader {
    use super::*;
    use crate::loaders::meshloader::*;

    const CUBE_OBJ: &str = "
v -1 -1 -1
v 1 -1 -1
v 1 1 -1
v -1 1 -1
v -1 -1 1
v 1 -1 1
v 1 1 1
v -1 1 1
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
";

    const TRIANGLE_STL: &str = "solid tri
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid tri
";

    fn write_temp(name: &str, content: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bevy_rapier_meshloader_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        std::fs::write(&path, content).unwrap();
        path
    }

    #[test]
    fn collider_from_obj_file() {
        let path = write_temp("cube.obj", CUBE_OBJ);
        let hull =
            Collider::from_mesh_file(&path, &MeshConverter::ConvexHull, Vect::splat(0.5)).unwrap();
        let aabb = hull.raw.compute_local_aabb();
        assert!((aabb.maxs - Vect::splat(0.5)).length() < 1.0e-5);
        assert!(hull.as_convex_polyhedron().is_some());

        let colliders =
            load_mesh_file_colliders(&path, &MeshConverter::TriMesh, Vect::new(1.0, 2.0, 3.0))
                .unwrap();
        assert_eq!(colliders.len(), 1);
        let trimesh = colliders[0].collider.as_trimesh().unwrap();
        assert_eq!(trimesh.raw.indices().len(), 12);
        assert_eq!(colliders[0].transform, Transform::IDENTITY);
        // The OBJ loader duplicates the vertices of each face.
        assert_eq!(colliders[0].raw_mesh.vertices.len(), 36);

        // Offset shapes are kept as compound shapes.
        let obb = Collider::from_mesh_file(&path, &MeshConverter::Obb, Vect::ONE).unwrap();
        assert!(obb.as_compound().is_some());
    }

    #[test]
    fn collider_from_stl_file() {
        let path = write_temp("triangle.stl", TRIANGLE_STL);
        let collider = Collider::from_mesh_file(&path, &MeshConverter::TriMesh, Vect::ONE).unwrap();
        assert_eq!(collider.as_trimesh().unwrap().raw.indices().len(), 1);

        assert!(Collider::from_mesh_file(
            path.with_extension("missing.stl"),
            &MeshConverter::TriMesh,
            Vect::ONE
        )
        .is_err());
    }

    #[cfg(feature = "to-bevy-mesh")]
    #[test]
    fn raw_mesh_to_bevy() {
        let path = write_temp("cube_mesh.obj", CUBE_OBJ);
        let colliders =
            load_mesh_file_colliders(&path, &MeshConverter::TriMesh, Vect::ONE).unwrap();
        let mesh = raw_mesh_to_bevy_mesh(&colliders[0].raw_mesh, Vect::splat(2.0));
        assert_eq!(mesh.count_vertices(), 36);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_some());
    }
}
