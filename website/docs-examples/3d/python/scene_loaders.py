import math
from pathlib import Path

import rapier3d as rp
from rapier3d.loaders import mesh, mjcf, urdf

# The snippets run from `website/docs-examples/3d/python`: the assets are those of the Rapier repository.
URDF_PATH = "../../../../assets/3d/T12/urdf/T12.URDF"
MJCF_PATH = "../../../../assets/3d/agility_cassie/scene.xml"
MESH_PATH = "../../../../assets/3d/chair.obj"


def load_urdf(path):
    # DOCUSAURUS: Urdf start
    world = rp.PhysicsWorld()

    # Read the robot (`path` is the path of the URDF file), then insert its links and joints into the world.
    robot, _ = urdf.UrdfRobot.from_file(path)
    handles = robot.insert_using_multibody_joints(world.rigid_bodies, world.colliders, world.multibody_joints)
    print(f"The robot has {len(handles.links)} links.")
    # DOCUSAURUS: Urdf stop
    return world, handles


def load_urdf_with_options(path):
    world = rp.PhysicsWorld()
    # DOCUSAURUS: UrdfOptions start
    options = urdf.UrdfLoaderOptions(
        # Whether colliders are created from the collision shapes of the links.
        # Default: True
        create_colliders_from_collision_shapes=True,
        # Whether colliders are created from the visual shapes of the links.
        # Default: False
        create_colliders_from_visual_shapes=False,
        # Whether the mass properties declared by the links are applied to their rigid-bodies.
        # Default: True
        apply_imported_mass_props=True,
        # Whether the colliders of two links attached by a joint can collide.
        # Default: False
        enable_joint_collisions=False,
        # Whether the root links are fixed rigid-bodies.
        # Default: False
        make_roots_fixed=True,
        # The pose applied to the whole robot, e.g., to convert its Z-up convention to Y-up.
        # Default: the identity pose.
        shift=rp.Isometry3(rotation=rp.Rotation3.from_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)),
        # The rigid-body every link is created from (before the URDF data is applied).
        # Default: None (a dynamic rigid-body).
        rigid_body_blueprint=rp.RigidBody.dynamic(),
    )
    robot, _ = urdf.UrdfRobot.from_file(path, options)
    # Insert the robot with impulse joints. The insertion consumes `robot`.
    impulse_robot = robot.insert_using_impulse_joints(world.rigid_bodies, world.colliders, world.impulse_joints)

    # Read the robot again to insert it 10 units further, with multibody joints.
    robot, _ = urdf.UrdfRobot.from_file(path, options)
    robot.append_transform(rp.Isometry3(translation=(10.0, 0.0, 0.0)))
    multibody_robot = robot.insert_using_multibody_joints(
        world.rigid_bodies,
        world.colliders,
        world.multibody_joints,
        urdf.UrdfMultibodyOptions.DISABLE_SELF_CONTACTS,
    )
    # DOCUSAURUS: UrdfOptions stop

    # Check that the defaults documented above are accurate.
    defaults = urdf.UrdfLoaderOptions()
    assert defaults.create_colliders_from_collision_shapes
    assert not defaults.create_colliders_from_visual_shapes
    assert defaults.apply_imported_mass_props
    assert not defaults.enable_joint_collisions
    assert not defaults.make_roots_fixed
    assert defaults.rigid_body_blueprint is None
    assert len(impulse_robot.links) == len(multibody_robot.links)

    for _ in range(10):
        world.step()


def load_urdf_from_string(urdf_xml, mesh_dir):
    world = rp.PhysicsWorld()
    # DOCUSAURUS: UrdfString start
    # Read the robot from a string containing its URDF description. The relative paths of its meshes are resolved
    # from `mesh_dir` (the current directory by default).
    robot, description = urdf.UrdfRobot.from_str(urdf_xml, mesh_dir=mesh_dir)
    # The parsed URDF description lists the names of the links and joints of the robot.
    print("Links:", [link.name for link in description.links])
    print("Joints:", [joint.name for joint in description.joints])
    # DOCUSAURUS: UrdfString stop
    robot.insert_using_impulse_joints(world.rigid_bodies, world.colliders, world.impulse_joints)


def load_mjcf(path):
    # DOCUSAURUS: Mjcf start
    world = rp.PhysicsWorld()

    # Read the model (`path` is the path of the MJCF file), then insert its bodies and joints into the world.
    robot, _model = mjcf.MjcfRobot.from_file(path)
    handles = robot.insert_using_impulse_joints(world.rigid_bodies, world.colliders, world.impulse_joints)
    # DOCUSAURUS: Mjcf stop
    assert any(body is not None for body in handles.bodies)
    for _ in range(10):
        world.step()


def load_mjcf_with_actuators(path):
    world = rp.PhysicsWorld()
    # DOCUSAURUS: MjcfActuators start
    # Rotate the model to convert its Z-up convention to Y-up.
    options = mjcf.MjcfLoaderOptions(
        shift=rp.Isometry3(rotation=rp.Rotation3.from_axis_angle((1.0, 0.0, 0.0), -math.pi / 2.0))
    )
    robot, _ = mjcf.MjcfRobot.from_file(path, options)
    # The gravity declared by the model isn't applied automatically. It is expressed in the frame of the
    # model file, so we rotate it like `options.shift` rotated the model.
    world.gravity = options.shift.rotation.transform_vector(robot.gravity)
    # Unlike the URDF loader, joints are generally inserted as multibody joints (like in MuJoCo).
    handles = robot.insert_using_multibody_joints(
        world.rigid_bodies,
        world.colliders,
        world.multibody_joints,
        world.impulse_joints,
        mjcf.MjcfMultibodyOptions.SKIP_LOOP_CLOSURES | mjcf.MjcfMultibodyOptions.DISABLE_SELF_CONTACTS,
    )

    # Drive the actuators of the model: one control input per actuator.
    print("Actuators:", [actuator.name for actuator in handles.actuators])
    controls = [0.0] * len(handles.actuators)
    controls[0] = 0.5
    handles.apply_controls(world.rigid_bodies, world.multibody_joints, controls, gain_scale=1.0)

    # Reset the robot to the first keyframe declared by the model (if any), given by its index or its name.
    if handles.keyframe_names:
        handles.apply_keyframe(world.rigid_bodies, world.multibody_joints, 0)
        # The control inputs holding the robot in the pose of this keyframe.
        controls = handles.keyframe_controls(0)
        handles.apply_controls(world.rigid_bodies, world.multibody_joints, controls)
    # DOCUSAURUS: MjcfActuators stop
    assert world.gravity.y < -9.0
    for _ in range(10):
        world.step()


def load_mjcf_with_contact_hooks(path):
    world = rp.PhysicsWorld()
    robot, _ = mjcf.MjcfRobot.from_file(path)
    handles = robot.insert_using_multibody_joints(
        world.rigid_bodies, world.colliders, world.multibody_joints, world.impulse_joints
    )
    # DOCUSAURUS: MjcfContactHooks start
    # The contact rules of the model, applied by physics hooks.
    world.physics_hooks = handles.contact_hooks()
    world.step()
    # DOCUSAURUS: MjcfContactHooks stop
    world.step()


def load_mesh(path):
    # DOCUSAURUS: Meshes start
    world = rp.PhysicsWorld()

    # Every mesh of the file becomes one shape, converted here into its convex hull.
    shapes = mesh.load_from_path(path, converter=rp.MeshConverter.CONVEX_HULL, scale=1.0)
    for shape in shapes:
        # The meshes that failed to be converted are given as exceptions instead.
        if isinstance(shape, Exception):
            print("Mesh conversion failed:", shape)
            continue
        world.add_collider(rp.Collider.new(shape.shape).position(shape.pose))
    # DOCUSAURUS: Meshes stop
    assert len(world.colliders) == len(shapes)


load_urdf(URDF_PATH)
load_urdf_with_options(URDF_PATH)
load_urdf_from_string(Path(URDF_PATH).read_text(), str(Path(URDF_PATH).parent))
load_mjcf(MJCF_PATH)
load_mjcf_with_actuators(MJCF_PATH)
load_mjcf_with_contact_hooks(MJCF_PATH)
load_mesh(MESH_PATH)
