use kiss3d::light::Light;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3};

use crate::camera::CustomCamera;
use crate::universe::Universe;

pub fn draw_scene(mut universe: Universe) {
    // Set up window and camera
    let mut window = Window::new("Kiss3d: cube");
    window.set_light(Light::StickToCamera);
    window.set_framerate_limit(Some(60));

    let mut camera = CustomCamera::new(2.0e9);

    // Set up bodies
    let mut spheres = Vec::with_capacity(universe.bodies.len());
    for body in universe.bodies.iter() {
        let mut sphere = window.add_sphere(body.info.radius);
        let color = &body.info.color;
        sphere.set_color(color.x, color.y, color.z);

        // TODO draw orbit

        spheres.push(sphere);
    }

    loop {
        // Draw grid
        draw_grid(&mut window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

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

            state.advance_t(138984.0 / 600.0);
        }

        // Update scene
        for (id, _) in universe.bodies.iter().enumerate() {
            let position: Point3<f32> = na::convert(universe.get_body_position(id));
            spheres[id].set_local_translation(Translation3::from(position.coords));
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
