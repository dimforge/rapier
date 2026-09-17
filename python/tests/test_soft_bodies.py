"""Soft-body tests (3D, f32): builders, the set, particles, clusters, deformable
colliders, tearing and cutting, events, settings and snapshots."""

from __future__ import annotations

import pickle

import numpy as np
import pytest

import rapier3d as rp


def _ground_world():
    w = rp.PhysicsWorld(gravity=(0, -9.81, 0))
    w.colliders.insert(rp.Collider.cuboid(20, 0.1, 20).build())
    return w


def _cloth(n=8, **kwargs):
    return rp.SoftBody.cloth((-1, 2, -1), (0.25, 0, 0), (0, 0, 0.25), n, n, **kwargs)


# ---- builders --------------------------------------------------------------


def test_generators_and_kwargs():
    cloth = _cloth(pinned_particles=[0, 3], softness=(30.0, 1.0), particle_mass=0.05)
    assert isinstance(cloth, rp.SoftBodyBuilder)
    assert cloth.num_particles == 64
    assert cloth.positions.shape == (64, 3)
    assert cloth.current_edges.shape[1] == 2
    rope = rp.SoftBody.rope((0, 1, 0), (1, 1, 0), 10)
    assert rope.num_particles == 10
    cube = rp.SoftBody.cuboid((0, 1, 0), (0.5, 0.5, 0.5), 3, 3, 3)
    assert cube.num_particles == 27
    assert cube.current_cells.shape[1] == 4
    sphere = rp.SoftBody.sphere((0, 1, 0), 0.5, 1)
    assert sphere.num_particles > 12
    with pytest.raises(TypeError):
        rp.SoftBody.rope((0, 1, 0), (1, 1, 0), 10, not_a_kwarg=3)


def test_builder_chaining_returns_new_builder():
    base = _cloth()
    heavier = base.particle_mass(2.0).softness(rp.SpringCoefficients(10.0, 1.0))
    assert isinstance(heavier, rp.SoftBodyBuilder)
    assert heavier is not base
    pinned = base.pinned_particles(np.array([0, 1], dtype=np.uint32))
    assert isinstance(pinned, rp.SoftBodyBuilder)


def test_material_kwargs_and_fields():
    m = rp.SoftBodyMaterial(young_modulus=2.0e3, poisson_ratio=0.3, tear_strain=0.4)
    assert m.young_modulus == pytest.approx(2.0e3)
    assert m.tear_strain == pytest.approx(0.4)
    assert m.tears()
    m.tear_strain = None
    assert m.tear_strain is None
    assert not m.tears()
    m.edge_softness = (12.0, 0.5)
    assert m.edge_softness.natural_frequency == pytest.approx(12.0)
    u = rp.SoftBodyMaterial.uniform((20.0, 1.0))
    assert u.bend_softness.natural_frequency == pytest.approx(20.0)
    lam, mu = m.lame_parameters()
    assert lam > 0 and mu > 0
    with pytest.raises(TypeError):
        rp.SoftBodyMaterial(nope=1)


def test_custom_positions_builder():
    positions = np.array([[0, 1, 0], [1, 1, 0], [0, 1, 1]], dtype=np.float32)
    b = rp.SoftBodyBuilder(positions).edges([[0, 1], [1, 2]]).cells(np.zeros((0, 4), dtype=np.uint32))
    assert b.num_particles == 3
    assert b.current_edges.shape == (2, 2)


# ---- set and views -----------------------------------------------------------


def test_insert_creates_root_body_and_colliders():
    w = _ground_world()
    nb, nc = len(w.rigid_bodies), len(w.colliders)
    h = w.add_soft_body(_cloth(pinned_particles=[0]))
    assert h in w.soft_bodies
    assert len(w.soft_bodies) == 1
    sb = w.soft_bodies[h]
    assert sb.num_particles == 64
    root = w.rigid_bodies[sb.root_body]
    assert root.is_soft_frame
    assert root.body_type == rp.RigidBodyType.SOFT_FRAME
    assert root.soft_body == h
    assert root.soft_cluster == 0
    assert len(w.rigid_bodies) == nb + 1
    assert len(w.colliders) == nc + 1
    surface = w.colliders[sb.collision_mesh().collider]
    assert surface.deformable_mesh_ref.body == h
    assert sb.particle_positions.shape == (64, 3)
    assert sb.boundary.shape[1] == 3
    assert sb.particle(0).is_pinned
    assert not sb.particle(1).is_pinned
    for handle, body in w.soft_bodies:
        assert handle == h and body.num_particles == 64
    assert w.soft_bodies.handles() == [h]
    assert w.soft_bodies.get(rp.SoftBodyHandle.invalid()) is None
    with pytest.raises(rp.InvalidHandle):
        w.soft_bodies[rp.SoftBodyHandle.invalid()]


def test_cloth_falls_and_pinned_corner_stays():
    w = _ground_world()
    h = w.add_soft_body(_cloth(pinned_particles=[0], softness=(30.0, 1.0)))
    sb = w.soft_bodies[h]
    p0 = sb.particle_position(0)
    before = sb.center_of_mass.y
    for _ in range(60):
        w.step()
    assert sb.center_of_mass.y < before - 0.2
    after = sb.particle_position(0)
    assert abs(after.x - p0.x) < 1e-5 and abs(after.y - p0.y) < 1e-5
    assert sb.particle_velocities.shape == (64, 3)
    assert sb.topology_version == 0


def test_particle_setters_and_impulses():
    w = rp.PhysicsWorld(gravity=(0, 0, 0))
    h = w.add_soft_body(rp.SoftBody.rope((0, 1, 0), (1, 1, 0), 5))
    sb = w.soft_bodies[h]
    sb.set_particle_position(2, (0.5, 3.0, 0.0))
    assert sb.particle_position(2).y == pytest.approx(3.0)
    sb.set_particle_velocity(2, (0, 1, 0))
    assert sb.particle_velocity(2).y == pytest.approx(1.0)
    sb.set_particle_pinned(0, True)
    assert sb.particle(0).is_pinned
    sb.apply_impulse((0, 1, 0))
    sb.apply_particle_impulse(1, (0, 1, 0))
    sb.apply_impulse_at_point((0, 1, 0), (0.5, 1, 0), 1.0)
    sb.apply_radial_impulse((0.5, 1, 0), 1.0, 2.0)
    sb.add_force((0, 1, 0))
    sb.add_particle_force(1, (0, 1, 0))
    sb.reset_forces()
    # Kinematic targets drive pinned particles only.
    sb.set_particle_pinned(4, True)
    sb.set_particle_kinematic_target(4, (1, 2, 0))
    w.step()
    assert sb.particle_position(4).y > 1.5
    with pytest.raises(IndexError):
        sb.set_particle_position(99, (0, 0, 0))


def test_attach_particle_to_rigid_body():
    w = rp.PhysicsWorld(gravity=(0, -9.81, 0))
    h = w.add_soft_body(rp.SoftBody.rope((0, 3, 0), (2, 3, 0), 10, pinned_particles=[0], softness=(40.0, 1.0)))
    sb = w.soft_bodies[h]
    last = sb.particle_position(9)
    weight = w.add_body(rp.RigidBody.dynamic(translation=(last.x, last.y - 0.3, last.z)),
                        colliders=[rp.Collider.ball(0.2).density(2.0)])
    sb.attach_particle(9, weight, w.rigid_bodies)
    assert len(sb.particle_attachments) == 1
    assert sb.particle_attachments[0].body == weight
    for _ in range(60):
        w.step()
    # The weight hangs from the rope: it stays near the rope's last particle.
    tip = sb.particle_position(9)
    wp = w.rigid_bodies[weight].translation
    assert abs(wp.x - tip.x) < 0.6 and abs(wp.y - tip.y) < 0.6
    assert sb.detach_particle(9)
    assert not sb.detach_particle(9)


def test_material_and_volume_views():
    w = rp.PhysicsWorld(gravity=(0, 0, 0))
    h = w.add_soft_body(rp.SoftBody.sphere((0, 1, 0), 0.5, 1, softness=(20.0, 1.0), volume_factor=1.2))
    sb = w.soft_bodies[h]
    assert sb.volume_preservation_enabled
    assert sb.volume_factor == pytest.approx(1.2)
    assert sb.volume == pytest.approx(sb.rest_volume)
    assert len(sb.volume_pieces) >= 1
    m = sb.material
    m.edge_softness = (5.0, 1.0)
    sb.material = m
    assert sb.material.edge_softness.natural_frequency == pytest.approx(5.0)
    sb.volume_factor = 1.5
    assert sb.volume_factor == pytest.approx(1.5)
    sb.enable_volume_preservation(False)
    assert not sb.volume_preservation_enabled
    jelly = w.add_soft_body(rp.SoftBody.cuboid((3, 1, 0), (0.5, 0.5, 0.5), 3, 3, 3,
                                                cell_model=rp.SoftBodyCellModel.COROTATIONAL))
    jb = w.soft_bodies[jelly]
    assert jb.cell_model == rp.SoftBodyCellModel.COROTATIONAL
    assert jb.cells.shape == (jb.num_cells, 4)
    assert jb.cell(0).rest_volume > 0
    assert jb.cell(0).plastic_stretch().shape == (3, 3)
    assert jb.mass == pytest.approx(27.0)
    jb.reset_plasticity()
    jb.user_data = 42
    assert jb.user_data == 42


# ---- clusters and deformable colliders -----------------------------------------


def _jelly_world():
    w = _ground_world()
    h = w.add_soft_body(rp.SoftBody.cuboid((0, 1.2, 0), (0.6, 0.6, 0.6), 4, 4, 4,
                                            cell_model=rp.SoftBodyCellModel.COROTATIONAL,
                                            material=rp.SoftBodyMaterial(young_modulus=4e3, poisson_ratio=0.4),
                                            particle_mass=0.1))
    return w, h


def test_cluster_proxy_joint_and_removal():
    w, h = _jelly_world()
    sb = w.soft_bodies[h]
    top = [i for i in range(sb.num_particles) if sb.particle_position(i).y > 1.5]
    assert len(top) == 16
    cluster = w.add_soft_body_cluster(h, top)
    assert cluster == 1
    info = sb.cluster(cluster)
    assert info.is_live and sorted(info.particles) == sorted(top)
    proxy = sb.cluster_proxy(cluster)
    assert w.rigid_bodies[proxy].soft_cluster == cluster
    assert sb.num_live_clusters == 2
    plate = w.add_body(rp.RigidBody.dynamic(translation=(0, 1.9, 0)),
                       colliders=[rp.Collider.cuboid(0.7, 0.05, 0.7).density(0.4)])
    joint = w.impulse_joints.insert(plate, proxy, rp.FixedJointBuilder().local_anchor1((0, -0.1, 0)).build(), True)
    sb.set_cluster_stiffness_scale(cluster, 2.0)
    sb.set_cluster_edge_softness(cluster, (50.0, 1.0))
    sb.set_cluster_edge_softness(cluster, None)
    sb.set_cluster_tear_resistance(cluster, 2.0)
    sb.enable_cluster_shape_matching(cluster, True)
    for _ in range(30):
        w.step()
    assert w.rigid_bodies[plate].translation.y > 1.0
    assert w.remove_soft_body_cluster(h, cluster)
    assert joint not in w.impulse_joints
    assert proxy not in w.rigid_bodies
    assert not w.remove_soft_body_cluster(h, cluster)
    assert w.add_soft_body_cluster(h, [999]) is None


def test_cluster_pinned_and_kinematic_target():
    w, h = _jelly_world()
    sb = w.soft_bodies[h]
    bottom = [i for i in range(sb.num_particles) if sb.particle_position(i).y < 0.7]
    cluster = w.add_soft_body_cluster(h, bottom)
    sb.set_cluster_pinned(cluster, True)
    assert sb.particle(bottom[0]).is_pinned
    sb.set_cluster_kinematic_target(cluster, rp.Isometry3(translation=(0.5, 0.6, 0.0)))
    w.step()
    assert sb.particle_position(bottom[0]).x > -0.6


def test_deformable_collider_binding():
    w, h = _jelly_world()
    sb = w.soft_bodies[h]
    root = sb.root_body
    origin = w.rigid_bodies[root].translation
    c = sb.center_of_mass
    r = 1.0
    verts = np.array([[c.x + r, c.y, c.z], [c.x - r, c.y, c.z], [c.x, c.y + r, c.z],
                      [c.x, c.y - r, c.z], [c.x, c.y, c.z + r], [c.x, c.y, c.z - r]], dtype=np.float32)
    verts -= np.array([origin.x, origin.y, origin.z], dtype=np.float32)
    idx = np.array([[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]], dtype=np.uint32)
    skin = w.insert_deformable(rp.Collider.trimesh(verts, idx, rp.TriMeshFlags.DEFORMABLE).sensor(True),
                               rp.SoftMeshBinding.skinned(), root)
    ref = w.colliders[skin].deformable_mesh_ref
    assert ref.body == h and ref.id.cluster == 0
    meshes = sb.meshes
    assert len(meshes) == 2
    mesh = sb.mesh_of(skin)
    assert mesh.is_skinned and mesh.vertex_count == 6 and mesh.indices.shape == (8, 3)
    assert sb.mesh(ref.id).collider == skin
    top_before = mesh.vertices[2, 1]
    for _ in range(30):
        w.step()
    assert sb.mesh_of(skin).vertices[2, 1] < top_before
    # A plain (non-deformable) mesh cannot be bound.
    with pytest.raises(rp.SoftBindingError):
        w.insert_deformable(rp.Collider.trimesh(verts, idx), rp.SoftMeshBinding.skinned(), root)
    # Neither can a parent that is not a cluster proxy.
    ball = w.add_body(rp.RigidBody.dynamic(translation=(5, 5, 5)))
    with pytest.raises(rp.SoftBindingError):
        w.insert_deformable(rp.Collider.trimesh(verts, idx, rp.TriMeshFlags.DEFORMABLE),
                            rp.SoftMeshBinding.skinned(), ball)
    w.remove_collider(skin)
    assert len(sb.meshes) == 1
    direct = rp.SoftMeshBinding.direct([0, 1, 2]).self_contacts(True)
    assert isinstance(direct, rp.SoftMeshBinding)
    assert isinstance(rp.SoftMeshBinding.direct_by_position(0.1), rp.SoftMeshBinding)


# ---- tearing and cutting ---------------------------------------------------------


def test_cut_splits_cloth_into_pieces():
    w = _ground_world()
    n = 8
    h = w.add_soft_body(_cloth(n, pinned_particles=[0, n * (n - 1)], softness=(30.0, 1.0)))
    sb = w.soft_bodies[h]
    ev = w.cut_soft_body(h, ((-0.1, -10, -10), (-0.1, 10, 0), (-0.1, -10, 10)))
    assert ev is not None
    assert ev.soft_body == h
    assert ev.torn_edges.shape[1] == 2 and ev.torn_edges.shape[0] > 0
    assert len(ev.pieces) == 2
    assert len(w.soft_bodies) == 2
    piece = [p for p in ev.pieces if p.soft_body != h][0]
    assert w.soft_bodies[piece.soft_body].origin == h
    assert sb.pieces == [piece.soft_body]
    dest = ev.particle_destination(n * n - 1)
    assert dest is not None and dest[0] in w.soft_bodies
    assert sb.topology_version >= 1
    assert ev.seeds().shape[1] == 2
    assert len(ev.bodies()) == 2
    for _ in range(10):
        w.step()
    # Nothing changes when the blade misses.
    assert w.cut_soft_body(h, ((50, 0, 0), (50, 1, 0), (50, 0, 1))) is None


def test_tear_requests_and_events():
    w = _ground_world()
    h = w.add_soft_body(_cloth(6, pinned_particles=[0], softness=(30.0, 1.0)))
    sb = w.soft_bodies[h]
    collector = rp.ChannelEventCollector()
    w.event_handler = collector
    sb.tear_edge(3)
    assert sb.has_pending_tears
    w.step()
    events = collector.drain_soft_body_tear_events()
    assert len(events) == 1 and events[0].soft_body == h and events[0].torn_edges.shape[0] == 1
    assert not sb.has_pending_tears

    class Handler:
        def __init__(self):
            self.events = []

        def handle_collision_event(self, *args):
            pass

        def handle_contact_force_event(self, *args):
            pass

        def handle_soft_body_tear_event(self, soft_bodies, event):
            self.events.append(event)

    handler = Handler()
    w.event_handler = handler
    ev = w.tear_soft_body(h, [10], [])
    assert ev is not None and ev.torn_edges.shape[0] == 1
    assert handler.events == []  # immediate tears return their event instead
    jelly = w.add_soft_body(rp.SoftBody.cuboid((3, 1, 0), (0.5, 0.5, 0.5), 3, 3, 3,
                                                cell_model=rp.SoftBodyCellModel.COROTATIONAL))
    w.soft_bodies[jelly].tear_cell(0)
    w.step()
    assert len(handler.events) == 1 and handler.events[0].soft_body == jelly
    assert handler.events[0].torn_cells.shape[0] >= 1


def test_strain_tearing_splits_a_stretched_rope():
    w = rp.PhysicsWorld(gravity=(0, 0, 0))
    h = w.add_soft_body(rp.SoftBody.rope((0, 1, 0), (1, 1, 0), 6, pinned_particles=[0, 5],
                                         softness=(60.0, 1.0), tear_strain=0.3, min_piece=1))
    collector = rp.ChannelEventCollector()
    w.event_handler = collector
    events = []
    # Pull the pinned end away until an edge passes the tear strain.
    for k in range(60):
        w.soft_bodies[h].set_particle_kinematic_target(5, (1.0 + 0.05 * (k + 1), 1, 0))
        w.step()
        events += collector.drain_soft_body_tear_events()
        if events:
            break
    assert events, "the stretched rope should have torn"
    assert events[0].soft_body == h
    assert len(w.soft_bodies) == 2


# ---- settings, events, snapshots ------------------------------------------------


def test_integration_parameters_soft_settings():
    ip = rp.IntegrationParameters()
    s = ip.soft_bodies
    assert isinstance(s, rp.SoftBodiesSettings)
    assert s.resweep_strain == pytest.approx(0.75)
    assert s.max_extra_substeps == 4
    s.max_extra_substeps = 2
    s.contact_stiffening = 3.0
    rec = s.recovery
    rec.crossing_repulsion = False
    rec.overlap_patch_constraints = rp.SoftPatchConstraints.KEEP
    s.recovery = rec
    ip.soft_bodies = s
    assert ip.soft_bodies.max_extra_substeps == 2
    assert ip.soft_bodies.contact_stiffening == pytest.approx(3.0)
    assert not ip.soft_bodies.recovery.crossing_repulsion
    ps = rp.SoftBodyParticleSettings()
    assert ps.additional_pgs_iterations == 3
    b = rp.RigidBody.dynamic(additional_pgs_iterations=2).build()
    assert b.additional_pgs_iterations == 2


def test_snapshot_round_trip_keeps_soft_bodies():
    w = _ground_world()
    h = w.add_soft_body(_cloth(6, pinned_particles=[0]))
    for _ in range(10):
        w.step()
    snap = w.snapshot()
    w2 = rp.PhysicsWorld.restore(snap)
    assert len(w2.soft_bodies) == 1
    a = w.soft_bodies[h].particle_positions
    b = w2.soft_bodies[h].particle_positions
    assert np.allclose(a, b)
    assert w2.rigid_bodies[w2.soft_bodies[h].root_body].is_soft_frame
    w2.step()
    w3 = rp.PhysicsWorld.restore_json(w.snapshot_json())
    assert len(w3.soft_bodies) == 1
    assert np.allclose(w3.soft_bodies[h].particle_positions, a)
    assert pickle.loads(pickle.dumps(h)) == h
    assert hash(h) == hash(rp.SoftBodyHandle.from_raw_parts(*h.into_raw_parts()))


def test_removals():
    w = _ground_world()
    h = w.add_soft_body(_cloth(6))
    sb = w.soft_bodies[h]
    root = sb.root_body
    nb, nc = len(w.rigid_bodies), len(w.colliders)
    removed = w.remove_soft_body(h)
    assert isinstance(removed, rp.SoftBody) and removed.num_particles == 36
    assert h not in w.soft_bodies
    assert root not in w.rigid_bodies
    assert len(w.rigid_bodies) == nb - 1 and len(w.colliders) == nc - 1
    assert w.remove_soft_body(h) is None
    # Removing the root proxy through the rigid-body set removes the soft body.
    h2 = w.add_soft_body(_cloth(6))
    root2 = w.soft_bodies[h2].root_body
    w.remove_body(root2)
    assert h2 not in w.soft_bodies
    # The low-level sets accept an explicit soft-body set.
    h3 = w.add_soft_body(_cloth(6))
    surface = w.soft_bodies[h3].collision_mesh().collider
    w.colliders.remove(surface, w.islands, w.rigid_bodies, soft_bodies=w.soft_bodies)
    assert w.soft_bodies[h3].collision_mesh() is None
    w.step()


def test_debug_render_soft_bodies():
    w = _ground_world()
    w.add_soft_body(_cloth(6))
    pipeline = rp.DebugRenderPipeline(rp.DebugRenderMode.SOFT_BODIES)
    lines, colors, objects = pipeline.render_to_arrays(
        w.rigid_bodies, w.colliders, w.impulse_joints, w.multibody_joints, w.narrow_phase, w.soft_bodies
    )
    assert lines.shape[0] > 0 and lines.shape[1:] == (2, 3)
    assert rp.DebugRenderObject.SOFT_BODY.kind in set(objects.tolist())
    # Without the soft bodies, nothing is drawn for them.
    lines2, _, objects2 = pipeline.render_to_arrays(
        w.rigid_bodies, w.colliders, w.impulse_joints, w.multibody_joints, w.narrow_phase
    )
    assert rp.DebugRenderObject.SOFT_BODY.kind not in set(objects2.tolist())
    style = rp.DebugRenderStyle()
    assert isinstance(style.soft_body_element_color, rp.DebugColor)


def test_low_level_pipeline_step_with_soft_bodies():
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    soft = rp.SoftBodySet()
    h = soft.insert(_cloth(6, pinned_particles=[0]), bodies, colliders)
    assert h in soft and len(soft) == 1 and not soft.is_empty()
    islands = rp.IslandManager()
    bp = rp.BroadPhaseBvh()
    np_ = rp.NarrowPhase()
    ij = rp.ImpulseJointSet()
    mj = rp.MultibodyJointSet()
    ccd = rp.CCDSolver()
    params = rp.IntegrationParameters()
    pipeline = rp.PhysicsPipeline()
    before = soft[h].center_of_mass.y
    for _ in range(20):
        pipeline.step((0, -9.81, 0), params, islands, bp, np_, bodies, colliders, ij, mj, ccd,
                      soft_bodies=soft)
    assert soft[h].center_of_mass.y < before
    soft.wake_up(h, bodies, True)
    removed = soft.remove(h, islands, bodies, colliders, ij, mj)
    assert removed is not None and len(soft) == 0
