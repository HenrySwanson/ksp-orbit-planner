use kiss3d::camera::ArcBall;
use kiss3d::event::WindowEvent;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3, Vector3};

use crate::universe::Universe;

struct CameraTransform {
    scale: f32,
}

pub fn draw_scene(mut universe: Universe) {
    // Set up window and camera
    let mut window = Window::new("Kiss3d: cube");
    window.set_light(Light::StickToCamera);
    window.set_framerate_limit(Some(60));

    let mut camera = ArcBall::new(Point3::new(0.0, -1.0, 1.0), Point3::origin());
    let mut camera_transform = CameraTransform { scale: 1.0e-9 };

    // Set up bodies
    let mut spheres = Vec::with_capacity(universe.bodies.len());
    for (id, body) in universe.bodies.iter().enumerate() {
        // This is where per-body reference frame stuff happens
        let position = universe.get_body_position(id);

        let mut sphere = window.add_sphere(261_600_000.0 * camera_transform.scale);
        sphere.set_color(0.0, 1.0, 0.0);
        prepare_body(&mut sphere, &position, &camera_transform);

        // TODO draw orbit

        spheres.push(sphere);
    }

    loop {
        // Draw grid
        draw_grid(&mut window, 20, 1.0, &Point3::new(0.5, 0.5, 0.5));

        // Handle events
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Scroll(_, off, _) => {
                    let dist = camera.dist();
                    if off > 0.0 {
                        camera.set_dist(dist * 1.5);
                    } else {
                        camera.set_dist(dist / 1.5);
                    }
                    event.inhibited = true;
                }
                _ => {}
            }
        }

        // Place camera
        let kerbin_location = universe.get_body_position(1);
        //camera.set_at(shrink_and_cast(&kerbin_location));

        // Render
        if !window.render_with_camera(&mut camera) {
            break;
        }

        // Update state
        for body in universe.bodies.iter_mut() {
            let mut state = match &mut body.state {
                Some(state) => state,
                None => continue,
            };

            state.advance_t(138984.0 / 600.0);
        }

        // Update scene
        for (id, body) in universe.bodies.iter().enumerate() {
            let position = universe.get_body_position(id);
            prepare_body(&mut spheres[id], &position, &camera_transform);
        }
    }
}

// Because there's no way to scale the whole scene, we'll manually transform object
// positions in order to put things in front of the camera correctly. This is going to
// be aggressively non-performant :(
// I suppose this code should live in the camera proper? Since the camera can
// take on planet-centered reference frames, it needs to be smart.
fn prepare_body(body: &mut SceneNode, position: &Point3<f64>, camera: &CameraTransform) {
    let mut position = Point3::new(position.x as f32, position.y as f32, position.z as f32);
    position *= camera.scale;
    body.set_local_translation(Translation3::from(position.coords));
}

fn draw_grid(window: &mut Window, num_squares: i32, square_size: f32, color: &Point3<f32>) {
    let max_coord = square_size * (num_squares as f32);
    for i in (-num_squares)..(num_squares + 1) {
        let coord = square_size * (i as f32);
        // horizontal
        window.draw_line(
            &Point3::new(-max_coord, coord, 0.0),
            &Point3::new(max_coord, coord, 0.0),
            &color,
        );
        // vertical
        window.draw_line(
            &Point3::new(coord, -max_coord, 0.0),
            &Point3::new(coord, max_coord, 0.0),
            &color,
        );
    }
}

fn draw_loop(window: &mut Window, pts: &[Point3<f32>], color: &Point3<f32>) {
    for (i, start) in pts.iter().enumerate() {
        let end = pts[(i + 1) % pts.len()];

        window.draw_line(&start, &end, &color);
    }
}

fn draw_path(window: &mut Window, pts: &[Point3<f32>], color: &Point3<f32>) {
    for (i, start) in pts.iter().enumerate() {
        if i == pts.len() - 1 {
            break;
        }

        let end = pts[i + 1];

        window.draw_line(&start, &end, &color);
    }
}
