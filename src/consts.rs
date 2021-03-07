use std::f64::consts::PI;

// Taken from the KSP wiki
pub const KERBIN_ORBIT_RADIUS: f64 = 13_599_840_256.0;
pub const KERBIN_ORBIT_PERIOD: f64 = 9_203_544.6;
pub const KERBOL_MU: f64 = 1.1723328e18;

pub fn get_circular_velocity(radius: f64, mu: f64) -> f64 {
    (mu / radius).sqrt()
}

pub fn get_period(a: f64, mu: f64) -> f64 {
    (4.0 * PI * PI * a.powi(3) / mu).sqrt()
}
