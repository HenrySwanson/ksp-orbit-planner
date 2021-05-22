extern crate kiss3d;

#[allow(dead_code)] // TODO... hmm
mod consts;

mod camera;
mod orbit;
mod root_finding;
mod simple_render;
mod state;
mod stumpff;
mod universe;

use kiss3d::nalgebra as na;
use na::{Point3, Rotation3, Vector3};

use std::collections::HashMap;
use std::fs;

use crate::universe::{BodyInfo, Universe};

fn main() {
    let u = read_file("ksp-bodies.txt");
    simple_render::draw_scene(u);
}

fn read_file(filename: &str) -> Universe {
    let mut universe = Universe::new();

    let mut name_to_id = HashMap::new();
    let mut name_to_mu = HashMap::new();

    // Read lines, skipping header
    for line in fs::read_to_string(filename).unwrap().lines().skip(1) {
        let mut fields = line.split_ascii_whitespace();

        // TODO lol this can't be the best way to do this. make something proper. later.
        macro_rules! next_string {
            () => {
                fields.next().unwrap()
            };
        };

        macro_rules! next_f64 {
            () => {
                fields.next().unwrap().parse::<f64>().unwrap()
            };
        };

        // Get name
        let name = next_string!();

        // Get body-info
        let mu = next_f64!();
        let body_info = BodyInfo {
            mu,
            radius: next_f64!() as f32,
            color: parse_color(next_string!()),
        };

        // Figure out what our orbit is
        let parent = next_string!();

        let id = if parent == "-" {
            universe.add_fixed_body(body_info)
        } else {
            let (a, ecc, incl, lan, argp, maae) = (
                next_f64!(),
                next_f64!(),
                next_f64!().to_radians(),
                next_f64!().to_radians(),
                next_f64!().to_radians(),
                next_f64!(), // already in radians!
            );
            let (position, velocity) = get_state(a, ecc, incl, lan, argp, maae, name_to_mu[parent]);

            let parent_id = name_to_id[parent];
            universe.add_body(body_info, position, velocity, parent_id)
        };
        name_to_id.insert(name, id);
        name_to_mu.insert(name, mu);
    }

    universe
}

fn parse_color(s: &str) -> Point3<f32> {
    assert_eq!(s.len(), 6);
    let r = u8::from_str_radix(&s[0..2], 16).unwrap();
    let g = u8::from_str_radix(&s[2..4], 16).unwrap();
    let b = u8::from_str_radix(&s[4..6], 16).unwrap();

    Point3::new(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
}

// TODO this whole thing belongs to be cleaned up and put near Orbit.rs
// Also should handle more than just ellipses
fn get_state(
    a: f64,
    ecc: f64,
    incl: f64,
    lan: f64,
    argp: f64,
    maae: f64,
    mu: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    assert!(ecc < 1.0, "Currently can only load elliptic orbits");
    
    // Compute the eccentric anomaly
    // M = E - e sin E, which doesn't have a closed form, so let's do some rootfinding
    use crate::root_finding::{find_root_bracket, newton_plus_bisection};
    let kepler = |x: f64| -> f64 { x - ecc * x.sin() - maae };
    let kepler_der = |x: f64| -> f64 { 1.0 - ecc * x.cos() };

    let bracket = find_root_bracket(kepler, maae, ecc + 0.1);
    let ecc_anomaly = newton_plus_bisection(|x| (kepler(x), kepler_der(x)), bracket, 100);

    // Now get the true anomaly
    let tan_half_ecc = (ecc_anomaly / 2.0).tan();
    let tan_half_theta = ((1.0 + ecc) / (1.0 - ecc)).sqrt() * tan_half_ecc;
    let true_anomaly = 2.0 * tan_half_theta.atan();

    // Compute the raw size and shape of the orbit
    let r = a * (1.0 - ecc * ecc_anomaly.cos());
    let v_sq = mu * (2.0 / r - 1.0 / a); // vis-viva
    let h_sq = mu * a * (1.0 - ecc * ecc);

    // Since h = r x v, we can find v_perp = h / r
    let vy_sq = h_sq / r / r;
    let vx_sq = na::clamp(v_sq - vy_sq, 0.0, v_sq);

    // Find position and velocity, ignoring orientation
    let position = r * Vector3::x();
    let velocity = Vector3::new(vx_sq.sqrt(), vy_sq.sqrt(), 0.0);

    // We have an orbit in the xy plane where the periapsis is -true_anomaly away from the
    // x-axis. So first, we rotate it around z until the periapsis is at argp away from
    // the x-axis. Then we tilt incl around the x-axis, and lastly one final turn around
    // z to get the ascending node pointing the right way.
    let transform = Rotation3::from_axis_angle(&Vector3::z_axis(), lan)
        * Rotation3::from_axis_angle(&Vector3::x_axis(), incl)
        * Rotation3::from_axis_angle(&Vector3::z_axis(), true_anomaly + argp);

    (transform * position, transform * velocity)
}
