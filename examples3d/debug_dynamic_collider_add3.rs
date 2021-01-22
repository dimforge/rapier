use na::{Isometry3, Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet};
use rapier3d::geometry::{ColliderBuilder, ColliderSet};
use rapier_testbed3d::Testbed;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    /*
     * Ground.
     */
    let ground_size = 20.0;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::new_static()
        .translation(0.0, -ground_height, 0.0)
        .build();
    let ground_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, 0.4)
        .friction(0.15)
        // .restitution(0.5)
        .build();
    colliders.insert(collider, ground_handle, &mut bodies);

    /*
     * Rolling ball
     */
    let ball_rad = 0.1;
    let rb = RigidBodyBuilder::new_dynamic()
        .translation(0.0, 0.2, 0.0)
        .linvel(10.0, 0.0, 0.0)
        .build();
    let ball_handle = bodies.insert(rb);
    let collider = ColliderBuilder::ball(ball_rad).density(100.0).build();
    colliders.insert(collider, ball_handle, &mut bodies);

    let mut linvel = Vector3::zeros();
    let mut angvel = Vector3::zeros();
    let mut pos = Isometry3::identity();
    let mut step = 0;
    let mut extra_colliders = Vec::new();
    let snapped_frame = 51;

    testbed.add_callback(move |mut window, mut graphics, physics, _, _| {
        step += 1;

        // Add a bigger ball collider
        let collider = ColliderBuilder::ball(ball_rad + 0.01 * (step as f32))
            .density(100.0)
            .build();
        let new_ball_collider_handle =
            physics
                .colliders
                .insert(collider, ball_handle, &mut physics.bodies);

        if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
            graphics.add_collider(window, new_ball_collider_handle, &physics.colliders);
        }

        extra_colliders.push(new_ball_collider_handle);

        // Snap the ball velocity or restore it.
        let ball = physics.bodies.get_mut(ball_handle).unwrap();

        if step == snapped_frame {
            linvel = *ball.linvel();
            angvel = *ball.angvel();
            pos = *ball.position();
        }

        if step == 100 {
            ball.set_linvel(linvel, true);
            ball.set_angvel(angvel, true);
            ball.set_position(pos, true);
            step = snapped_frame;

            for handle in &extra_colliders {
                physics.colliders.remove(*handle, &mut physics.bodies, true);
            }

            extra_colliders.clear();
        }

        // Remove then re-add the ground collider.
        // let ground = physics.bodies.get_mut(ground_handle).unwrap();
        // ground.set_position(Isometry3::translation(0.0, step as f32 * 0.001, 0.0), false);
        // let coll = physics
        //     .colliders
        //     .remove(ground_collider_handle, &mut physics.bodies, true)
        //     .unwrap();
        let coll = ColliderBuilder::cuboid(ground_size, ground_height + step as f32 * 0.01, 0.4)
            .friction(0.15)
            .build();
        let new_ground_collider_handle =
            physics
                .colliders
                .insert(coll, ground_handle, &mut physics.bodies);

        if let (Some(graphics), Some(window)) = (&mut graphics, &mut window) {
            graphics.add_collider(window, new_ground_collider_handle, &physics.colliders);
        }

        extra_colliders.push(new_ground_collider_handle);
    });

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
