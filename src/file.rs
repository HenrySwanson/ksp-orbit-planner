use std::collections::HashMap;
use std::f64::consts::PI;
use std::fs;

use nalgebra::Point3;

use crate::astro::orbit::{Orbit, PointMass};
use crate::model::orrery::{BodyInfo, Orrery};

pub fn read_file(filename: &str) -> Orrery {
    let mut orrery = Orrery::new();

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
        }

        macro_rules! next_f64 {
            () => {
                fields.next().unwrap().parse::<f64>().unwrap()
            };
        }

        // Get name
        let name = next_string!();

        // Get body-info
        let mu = next_f64!();
        let body_info = BodyInfo {
            name: name.to_owned(),
            mu,
            radius: next_f64!() as f32,
            color: parse_color(next_string!()),
        };

        // Figure out what our orbit is
        let parent = next_string!();

        let id = if parent == "-" {
            orrery.add_fixed_body(body_info)
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

            let orbit =
                Orbit::from_kepler(PointMass::with_mu(parent_mu), (), a, ecc, incl, lan, argp);
            // M = 2pi/P (t - t_periapse)
            let time_since_periapsis = maae * orbit.period().unwrap() / 2.0 / PI;
            let time_at_periapsis = -time_since_periapsis;

            orrery.add_body(body_info, orbit, time_at_periapsis, parent_id)
        };
        name_to_id.insert(name, id);
        name_to_mu.insert(name, mu);
    }

    orrery
}

fn parse_color(s: &str) -> Point3<f32> {
    assert_eq!(s.len(), 6);
    let r = u8::from_str_radix(&s[0..2], 16).unwrap();
    let g = u8::from_str_radix(&s[2..4], 16).unwrap();
    let b = u8::from_str_radix(&s[4..6], 16).unwrap();

    Point3::new(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::model::orrery::BodyID;

    #[test]
    fn test() {
        let orrery = read_file("ksp-bodies.txt");
        let eeloo = orrery.orbit_of_body(BodyID(16)).unwrap().orbit().clone();
        assert_eq!(eeloo.primary().info.name, "Kerbol");
        assert_eq!(eeloo.secondary().info.name, "Eeloo");

        assert_relative_eq!(eeloo.semimajor_axis(), 90_118_820_000.0);
        assert_relative_eq!(eeloo.eccentricity(), 0.26);
        assert_relative_eq!(eeloo.inclination().to_degrees(), 6.15, max_relative = 1e-14);
        assert_relative_eq!(
            eeloo.arg_periapse().to_degrees(),
            260.0,
            max_relative = 1e-14
        );
        assert_relative_eq!(
            eeloo.long_asc_node().to_degrees(),
            50.0,
            max_relative = 1e-14
        );
    }
}
