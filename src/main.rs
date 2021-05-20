extern crate kiss3d;

mod camera;
mod consts;
mod orbit;
mod simple_render;
mod state;
mod stumpff;
mod universe;

use kiss3d::nalgebra as na;
use na::{Point3, Rotation3, Vector3};

use crate::consts::get_circular_velocity;
use crate::universe::{BodyInfo, Universe};

fn main() {
    let mut u = Universe::new();

    let kerbol = u.add_fixed_body(BodyInfo {
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
        kerbol,
    );

    let mun = u.add_body(
        BodyInfo {
            mu: consts::MUN_MU,
            radius: consts::MUN_RADIUS,
            color: Point3::new(0.7, 0.7, 0.7),
        },
        Vector3::x() * consts::MUN_ORBIT_RADIUS,
        Vector3::y() * get_circular_velocity(consts::MUN_ORBIT_RADIUS, consts::KERBIN_MU),
        kerbin,
    );

    // TODO: the wiki says argument of periapsis is 38 degrees, but i don't see why it matters
    // since it's a circular orbit. maybe for initial position?
    let minimus_transform = get_transform_for_kepler_angles(
        consts::MINIMUS_ORBIT_INCL_DEG.to_radians(),
        consts::MINIMUS_ORBIT_LAN_DEG.to_radians(),
    );
    let minimus_velocity = get_circular_velocity(consts::MINIMUS_ORBIT_RADIUS, consts::KERBIN_MU);
    let minimus = u.add_body(
        BodyInfo {
            mu: consts::MINIMUS_MU,
            radius: consts::MINIMUS_RADIUS,
            color: Point3::new(0.8, 0.6, 1.0),
        },
        minimus_transform * Vector3::x() * consts::MINIMUS_ORBIT_RADIUS,
        minimus_transform * Vector3::y() * minimus_velocity,
        kerbin,
    );

    simple_render::draw_scene(u);
}

// TODO: this should eventually go to "orbit", shouldn't it...
fn get_transform_for_kepler_angles(inclination: f64, lan: f64) -> Rotation3<f64> {
    Rotation3::from_axis_angle(&Vector3::z_axis(), lan)
        * Rotation3::from_axis_angle(&Vector3::x_axis(), inclination)
}
