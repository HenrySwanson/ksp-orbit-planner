extern crate kiss3d;

mod body;
mod camera;
mod consts;
mod simple_render;
mod state;
mod stumpff;
mod universe;

use kiss3d::nalgebra as na;
use na::Vector3;

use crate::body::{Body, BodyInfo};
use crate::consts::{get_circular_velocity, KERBIN_ORBIT_RADIUS, KERBOL_MU};
use crate::state::State;
use crate::universe::Universe;

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

    let mut u = Universe::new(BodyInfo {
        mu: KERBOL_MU,
        radius: 261_600_000.0,
        color: Vector3::new(1.0, 1.0, 0.0),
    });

    let kerbin = u.add_body(
        BodyInfo {
            mu: 3.5316000e12,
            radius: 600_000.0,
            color: Vector3::new(0.0, 1.0, 0.0),
        },
        Vector3::x() * KERBIN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(KERBIN_ORBIT_RADIUS, KERBOL_MU),
        u.root_body,
    );

    let mun = u.add_body(
        BodyInfo {
            mu: 6.5138398e10,
            radius: 200_000.0,
            color: Vector3::new(0.3, 0.3, 0.3),
        },
        Vector3::x() * 12_000_000.0,
        Vector3::y() * get_circular_velocity(12_000_000.0, 3.5316000e12),
        kerbin,
    );
    simple_render::draw_scene(u);
}
