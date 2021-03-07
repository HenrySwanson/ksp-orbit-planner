extern crate kiss3d;

mod consts;
mod simple_render;
mod state;
mod stumpff;

use kiss3d::nalgebra as na;
use na::Vector3;

use crate::consts::{get_circular_velocity, KERBIN_ORBIT_RADIUS, KERBOL_MU};
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

    let kerbol = State::new(Vector3::x() * 0.0, Vector3::y() * 0.0, 0.0, 0.0);
    let kerbin = State::new(
        Vector3::x() * KERBIN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(KERBIN_ORBIT_RADIUS, KERBOL_MU),
        0.0,
        KERBOL_MU,
    );
    let kerbin2 = State::new(
        Vector3::x() * 4.0 * KERBIN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(4.0 * KERBIN_ORBIT_RADIUS, KERBOL_MU),
        0.0,
        KERBOL_MU,
    );
    simple_render::draw_scene(&mut [kerbol, kerbin, kerbin2]);
}
