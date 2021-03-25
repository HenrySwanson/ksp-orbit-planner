use kiss3d::light::Light;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3, Vector3};

use std::f64::consts::PI;

use crate::camera::CustomCamera;
use crate::universe::Universe;

const TIMESTEP: f64 = 138984.0 / 600.0;

pub fn draw_scene(mut universe: Universe) {
    // Set up window and camera
    let mut window = Window::new("Kiss3d: cube");
    window.set_light(Light::StickToCamera);
    window.set_framerate_limit(Some(60));

    let mut camera = CustomCamera::new(2.0e9);

    // Set up objects to draw
    let mut spheres = Vec::with_capacity(universe.bodies.len());
    let mut orbit_paths = Vec::with_capacity(universe.bodies.len());

    for (id, body) in universe.bodies.iter().enumerate() {
        // Make the sphere to represent the body
        let mut sphere = window.add_sphere(body.info.radius);
        let color = &body.info.color;
        sphere.set_color(color.x, color.y, color.z);
        let position: Point3<f32> = na::convert(universe.get_body_position(id));
        sphere.set_local_translation(Translation3::from(position.coords));

        // Compute the orbit path
        let orbit = universe.get_body_orbit(id);
        let points: Vec<Vector3<f32>> = match orbit {
            Some(orbit) => {
                (0..=100_usize)
                    // start at -180 so that open orbits work right
                    .map(|i| -PI + (2.0 * PI) * (i as f64) / 100.0)
                    .map(|theta| orbit.get_position_at_theta(theta))
                    .filter_map(|option_v| option_v.map(na::convert))
                    .collect()
            }
            None => vec![],
        };

        spheres.push(sphere);
        orbit_paths.push(points);
    }

    loop {
        // Draw grid
        draw_grid(&mut window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw orbits
        for (id, body) in universe.bodies.iter().enumerate() {
            let parent_id = match &body.state {
                Some(state) => state.get_parent_id(),
                None => continue,
            };

            let parent_position: Point3<f32> = na::convert(universe.get_body_position(parent_id));
            let orbit_path: Vec<_> = orbit_paths[id]
                .iter()
                .map(|x| parent_position + x)
                .collect();
            draw_path(&mut window, &orbit_path, &body.info.color);
        }

        // Place camera
        //let kerbin_location = universe.get_body_position(1);
        //camera.set_at(camera_transform.apply_to(&kerbin_location));

        // Render
        if !window.render_with_camera(&mut camera) {
            break;
        }

        // Update state
        for body in universe.bodies.iter_mut() {
            let state = match &mut body.state {
                Some(state) => state,
                None => continue,
            };

            state.advance_t(TIMESTEP);
        }

        // Update scene
        for (id, sphere) in spheres.iter_mut().enumerate() {
            let position: Point3<f32> = na::convert(universe.get_body_position(id));
            sphere.set_local_translation(Translation3::from(position.coords));
        }
    }
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
    draw_path(window, pts, color);
    let length = pts.len();
    if length > 2 {
        window.draw_line(&pts[length - 1], &pts[0], color);
    }
}

fn draw_path(window: &mut Window, pts: &[Point3<f32>], color: &Point3<f32>) {
    for pair in pts.windows(2) {
        window.draw_line(&pair[0], &pair[1], &color);
    }
}
