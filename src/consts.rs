use std::f64::consts::PI;

// Taken from the KSP wiki
pub const KERBOL_MU: f64 = 1.1723328e18;
pub const KERBOL_RADIUS: f32 = 261_600_000.0;

pub const KERBIN_MU: f64 = 3.5316000e12;
pub const KERBIN_RADIUS: f32 = 600_000.0;
pub const KERBIN_ORBIT_RADIUS: f64 = 13_599_840_256.0;
pub const KERBIN_ORBIT_PERIOD: f64 = 9_203_544.6;

pub const MUN_MU: f64 = 6.5138398e10;
pub const MUN_RADIUS: f32 = 200_000.0;
pub const MUN_ORBIT_RADIUS: f64 = 12_000_000.0;

pub fn get_circular_velocity(radius: f64, mu: f64) -> f64 {
    (mu / radius).sqrt()
}

pub fn get_period(a: f64, mu: f64) -> f64 {
    (4.0 * PI * PI * a.powi(3) / mu).sqrt()
}
