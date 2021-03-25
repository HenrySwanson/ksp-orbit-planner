extern crate kiss3d;

mod body;
mod camera;
mod consts;
mod orbit;
mod simple_render;
mod state;
mod stumpff;
mod universe;

use kiss3d::nalgebra as na;
use na::{Point3, Vector3};

use crate::body::BodyInfo;
use crate::consts::get_circular_velocity;
use crate::universe::Universe;

fn main() {
    let mut u = Universe::new(BodyInfo {
        mu: consts::KERBOL_MU,
        radius: consts::KERBOL_RADIUS,
        color: Point3::new(1.0, 1.0, 0.0),
    });

    let kerbin = u.add_body(
        BodyInfo {
            mu: consts::KERBIN_MU,
            radius: consts::KERBIN_RADIUS,
            color: Point3::new(0.0, 1.0, 0.0),
        },
        Vector3::x() * consts::KERBIN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(consts::KERBIN_ORBIT_RADIUS, consts::KERBOL_MU),
        u.root_body,
    );

    let _mun = u.add_body(
        BodyInfo {
            mu: consts::MUN_MU,
            radius: consts::MUN_RADIUS,
            color: Point3::new(1.0, 0.3, 0.3),
        },
        Vector3::x() * consts::MUN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(consts::MUN_ORBIT_RADIUS, consts::KERBIN_MU),
        kerbin,
    );
    simple_render::draw_scene(u);
}
