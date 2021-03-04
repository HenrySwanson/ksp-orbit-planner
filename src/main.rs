extern crate kiss3d;

mod state;
mod stumpff;

use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Vector3};

use crate::state::State;

fn main() {
    // // Set up window and camera
    // let mut window = Window::new("Kiss3d: cube");
    // let mut camera = ArcBall::new(Point3::new(0.0, -10.0, 10.0), Point3::origin());
    // window.set_light(Light::StickToCamera);

    // // Create Kerbin
    // let mut kerbin = window.add_sphere(1.0);
    // kerbin.set_color(0.0, 1.0, 0.0);
    // // Compute circle path to draw
    // let circle_pts = construct_circle(2.0, 100);

    // // Render loop
    // while window.render_with_camera(&mut camera) {
    //     draw_grid(&mut window, 20, 1.0, &Point3::new(0.5, 0.5, 0.5));
    //     draw_loop(&mut window, &circle_pts, &Point3::new(1.0, 0.0, 0.0));
    // }

    // Build the Earth and see if the orbit simulation is right.
    let orbit_radius = 1.496e11;
    let orbit_velocity = 29806.0;
    let sun_mu = 1.327e20;
    let expected_period = 3.154e7;

    let mut state = State::new(
        Vector3::x() * orbit_radius,
        Vector3::y() * orbit_velocity,
        0.0,
        sun_mu,
    );

    let N = 4;
    let root_beta = (-2.0 * state.get_energy()).sqrt();
    let delta_s = 2.0 * std::f64::consts::PI / (N as f64) / root_beta;

    for i in 0..=N {
        println!(
            "Should be {}/{} around the Sun.\n
            Position: {}\n
            Velocity: {}",
            i,
            N,
            state.get_position(),
            state.get_velocity()
        );
        state.advance_s(delta_s);
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
    for (i, pt) in pts.iter().enumerate() {
        let j = (i + 1) % pts.len();
        let pt2 = pts[j];

        window.draw_line(&pt, &pt2, &color);
    }
}

fn construct_circle(radius: f32, npoints: usize) -> Vec<Point3<f32>> {
    let mut points = Vec::with_capacity(npoints);
    for i in 0..npoints {
        let theta = 2.0 * std::f32::consts::PI * (i as f32) / (npoints as f32);
        let pt = Point3::new(radius * theta.cos(), radius * theta.sin(), 0.0);
        points.push(pt);
    }

    points
}
