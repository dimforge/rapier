use na::{Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, HalfSpace, SharedShape};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();

    /*
     * Ground
     */
    let rigid_body = RigidBodyBuilder::new_static().build();
    let handle = bodies.insert(rigid_body);
    let halfspace = SharedShape::new(HalfSpace::new(Vector3::y_axis()));
    let collider = ColliderBuilder::new(halfspace).build();
    colliders.insert(collider, handle, &mut bodies);

    let mut curr_y = 0.0;
    let mut curr_width = 10_000.0;

    for _ in 0..12 {
        let curr_height = 0.1f32.min(curr_width);
        curr_y += curr_height * 4.0;

        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(0.0, curr_y, 0.0)
            .build();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::cuboid(curr_width, curr_height, curr_width).build();
        colliders.insert(collider, handle, &mut bodies);

        curr_width /= 5.0;
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, joints);
    testbed.look_at(Point3::new(10.0, 10.0, 10.0), Point3::origin());
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Boxes", init_world)]);
    testbed.run()
}
