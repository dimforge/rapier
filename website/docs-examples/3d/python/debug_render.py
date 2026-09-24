import rapier3d as rp


# DOCUSAURUS: DebugRenderBackend start
# The backend receives the lines to be drawn. A real one would push them to the renderer of the
# application instead of collecting them.
class LineCollector:
    def __init__(self):
        self.lines = []

    def draw_line(self, object, a, b, color):
        # The color is given in the HSLA format: `color.rgba` converts it to RGBA.
        self.lines.append((a, b, color.rgba))
# DOCUSAURUS: DebugRenderBackend stop


world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.add_collider(rp.Collider.cuboid(100.0, 0.1, 100.0))
world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)),
    colliders=[rp.Collider.ball(0.5)],
)

# DOCUSAURUS: DebugRender start
# The style gives the colors and sizes, the mode selects what is drawn.
debug_render = rp.DebugRenderPipeline(
    style=rp.DebugRenderStyle(),
    mode=rp.DebugRenderMode.COLLIDER_SHAPES | rp.DebugRenderMode.CONTACTS,
)
backend = LineCollector()

for _ in range(10):
    world.step()

    # The debug-rendering is done after the step, once per frame to be drawn.
    backend.lines.clear()
    debug_render.render(
        world.rigid_bodies,
        world.colliders,
        world.impulse_joints,
        world.multibody_joints,
        world.narrow_phase,
        backend,
        soft_bodies=world.soft_bodies,
    )

print(f"{len(backend.lines)} lines to draw")
# DOCUSAURUS: DebugRender stop
assert len(backend.lines) > 0

# DOCUSAURUS: DebugRenderArrays start
# All the lines are computed without calling back into Python.
lines, colors, objects = debug_render.render_to_arrays(
    world.rigid_bodies,
    world.colliders,
    world.impulse_joints,
    world.multibody_joints,
    world.narrow_phase,
    soft_bodies=world.soft_bodies,
)
# `lines` has the shape (N, 2, 3): the two end points of each line.
# `colors` has the shape (N, 4): the RGBA color of each line.
# `objects` has the shape (N,): the kind of object each line belongs to.
collider_lines = lines[objects == rp.DebugRenderObject.COLLIDER.kind]
print(f"{len(collider_lines)} lines for the collider shapes")
# DOCUSAURUS: DebugRenderArrays stop
assert lines.shape == (len(backend.lines), 2, 3)
assert colors.shape == (len(backend.lines), 4)
assert len(collider_lines) > 0

# DOCUSAURUS: DebugRenderStyle start
# The style is modified in place: the next renders take it into account.
debug_render.style.subdivisions = 40  # Smoother curved shapes.
debug_render.style.contact_normal_length = 0.5
debug_render.style.collider_fixed_color = rp.DebugColor.from_rgba(0.5, 0.5, 0.5, 1.0)

# Also draw the AABBs of the colliders.
debug_render.mode = debug_render.mode | rp.DebugRenderMode.COLLIDER_AABBS
# DOCUSAURUS: DebugRenderStyle stop
assert debug_render.style.subdivisions == 40
assert debug_render.style.contact_normal_length == 0.5
assert rp.DebugRenderMode.COLLIDER_AABBS in debug_render.mode
