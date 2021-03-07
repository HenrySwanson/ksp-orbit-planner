extern crate kiss3d;
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3, Vector3};

use crate::state::State;

pub fn draw_scene(bodies: &mut [State]) {
    // Set up window and camera
    let mut window = Window::new("Kiss3d: cube");
    window.set_light(Light::StickToCamera);
    window.set_framerate_limit(Some(60));

    let mut camera = ArcBall::new(Point3::new(0.0, -20.0, 20.0), Point3::origin());

    // Set up bodies
    let mut spheres = Vec::with_capacity(bodies.len());
    for body in bodies.iter() {
        // This is where per-body reference frame stuff happens
        let position = Point3::origin() + body.get_position();

        let mut sphere = window.add_sphere(1.0);
        sphere.set_color(0.0, 1.0, 0.0);
        sphere.set_local_translation(Translation3::from(shrink_and_cast(&position).coords));
        // TODO draw orbit

        spheres.push(sphere);
    }

    while window.render_with_camera(&mut camera) {
        // Draw grid
        draw_grid(&mut window, 20, 1.0, &Point3::new(0.5, 0.5, 0.5));

        // Post-render
        for (i, body) in bodies.iter_mut().enumerate() {
            if body.mu == 0.0 {
                continue; // bad hack till parents work right
            }

            body.advance_t(9_203_544.6 / 600.0);

            let position = Point3::origin() + body.get_position();
            spheres[i].set_local_translation(Translation3::from(shrink_and_cast(&position).coords))
        }
    }
}

fn shrink_and_cast(p: &Point3<f64>) -> Point3<f32> {
    let p = p / 1e9;
    Point3::new(p.x as f32, p.y as f32, p.z as f32)
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
