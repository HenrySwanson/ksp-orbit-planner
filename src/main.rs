extern crate kiss3d;

mod anomaly;
mod camera;
mod consts;
mod orbit;
mod root_finding;
mod simple_render;
mod state;
mod stumpff;
mod universe;

use kiss3d::nalgebra as na;
use na::Point3;

use std::collections::HashMap;
use std::fs;

use crate::orbit::Orbit;
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
            let parent_id = name_to_id[parent];
            let parent_mu = name_to_mu[parent];

            let (a, ecc, incl, lan, argp, maae) = (
                next_f64!(),
                next_f64!(),
                next_f64!().to_radians(),
                next_f64!().to_radians(),
                next_f64!().to_radians(),
                next_f64!(), // already in radians!
            );

            assert!(ecc < 1.0, "Currently can only load elliptic orbits");

            let orbit = Orbit::from_kepler(a, ecc, incl, lan, argp, parent_mu);
            let theta = anomaly::mean_to_true(maae, ecc);
            let (position, velocity) = orbit.get_state_at_theta(theta);

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
