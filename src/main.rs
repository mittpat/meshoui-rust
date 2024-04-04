use blue_engine::{header::Engine, primitive_shapes::cube};
use rapier3d::prelude::*;

fn main() {
    // rapier3d
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
  
    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    collider_set.insert(collider);
  
    /* Create the bouncing ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 100.0, 0.0])
            .build();
    let collider = ColliderBuilder::ball(0.5).restitution(0.975).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);
  
    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();


    // blue_engine
    let mut engine = Engine::new().expect("win");
    cube("Cube", &mut engine.renderer, &mut engine.objects).unwrap();
    engine
        .objects
        .get_mut("Cube")
        .unwrap()
        .set_uniform_color(96f32/255f32, 160f32/255f32, 226f32/255f32, 1f32)
        .unwrap();

    let radius = 25f32;
    let speed = 0.25f32;
    let start = std::time::SystemTime::now();
    engine
        .update_loop(move |_, _, objects, _, camera, _| {

            // rapier3d
            if (std::time::SystemTime::now().duration_since(start)).unwrap().as_secs_f32() > 0.0166
            {
                physics_pipeline.step(
                    &gravity,
                    &integration_parameters,
                    &mut island_manager,
                    &mut broad_phase,
                    &mut narrow_phase,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut impulse_joint_set,
                    &mut multibody_joint_set,
                    &mut ccd_solver,
                    Some(&mut query_pipeline),
                    &physics_hooks,
                    &event_handler,
                );
            }

            // blue_engine
            let camx = (speed * start.elapsed().unwrap().as_secs_f32()).sin() * radius;
            let camy = (speed * start.elapsed().unwrap().as_secs_f32()).sin() * radius;
            let camz = (speed * start.elapsed().unwrap().as_secs_f32()).cos() * radius;
            camera
                .set_position(camx, camy, camz)
                .expect("Couldn't update the camera eye");

            let cube = objects.get_mut("Cube").unwrap();

            // rapier3d
            let ball_body = &rigid_body_set[ball_body_handle];

            // blue_engine
            cube.set_position(ball_body.translation().x, ball_body.translation().y, ball_body.translation().z);
        })
        .expect("Error during update loop");
}