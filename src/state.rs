use kiss3d::nalgebra as na;
use na::Vector3;

use crate::stumpff::stumpff_G;

pub struct State {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    time: f64,
    mu: f64, // TODO: move this elsewhere
}

#[allow(non_snake_case)]
impl State {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, time: f64, mu: f64) -> Self {
        State {
            position,
            velocity,
            time,
            mu,
        }
    }

    pub fn get_position(&self) -> &Vector3<f64> {
        &self.position
    }

    pub fn get_velocity(&self) -> &Vector3<f64> {
        &self.velocity
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn get_energy(&self) -> f64 {
        // KE = 1/2 v^2, PE = - mu/r
        self.velocity.norm_squared() / 2.0 - self.mu / self.position.norm()
    }

    pub fn advance_s(&mut self, delta_s: f64) {
        let beta = -2.0 * self.get_energy();
        let mu = self.mu;
        let G: [f64; 4] = stumpff_G(beta, delta_s);

        let r_0 = self.position.norm();
        let r_dot_0 = self.position.dot(&self.velocity) / r_0;

        let f = 1.0 - mu / r_0 * G[2];
        let g = r_0 * G[1] + r_0 * r_dot_0 * G[2];

        let new_position = f * self.position + g * self.velocity;
        let new_r = new_position.norm();

        let f_dot = -mu / r_0 / new_r * G[1];
        let g_dot = r_0 / new_r * (G[0] + r_dot_0 * G[1]);

        let new_velocity = f_dot * self.position + g_dot * self.velocity;
        let delta_t = r_0 * G[1] + r_0 * r_dot_0 * G[2] + mu * G[3];

        self.position = new_position;
        self.velocity = new_velocity;
        self.time += delta_t;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::PI;

    // Taken from the KSP wiki
    const KERBIN_ORBIT_RADIUS: f64 = 13_599_840_256.0;
    const KERBIN_ORBIT_PERIOD: f64 = 9_203_544.6;
    const KERBOL_MU: f64 = 1.1723328e18;

    const KERBIN_ORBIT_VELOCITY: f64 = 9284.5; // had to increase precision over the wiki

    fn assert_close(expected: f64, actual: f64, tolerance: f64) {
        let difference = actual - expected;
        if difference.abs() >= tolerance * expected.abs() {
            panic!("Floats expected to be close: {} vs. {}", actual, expected);
        }
    }

    // We'll count this as a success if the difference between the vectors is small,
    // relative to the length of the expected vector.
    fn assert_vectors_close(expected: &Vector3<f64>, actual: &Vector3<f64>, tolerance: f64) {
        let difference = actual - expected;
        if difference.norm() >= tolerance * expected.norm() {
            panic!(
                "Vectors were not as close as expected!\n
                Expected: {}\n
                Received: {}\n
                Difference: {}\n
                Relative difference: {:e}",
                expected,
                actual,
                difference,
                difference.norm() / expected.norm(),
            );
        }
    }

    #[test]
    fn test_kerbin() {
        // Build Kerbin and see if the orbit simulation is right.
        // Kerbin's orbit is perfectly circular.
        let initial_position = Vector3::x() * KERBIN_ORBIT_RADIUS;
        let initial_velocity = Vector3::y() * KERBIN_ORBIT_VELOCITY;

        let mut state = State::new(initial_position, initial_velocity, 0.0, KERBOL_MU);

        // Advance for one full orbit.
        // This is a circular orbit, so s is proportional to theta. Specifically,
        // s = theta / sqrt(beta).
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();
        state.advance_s(s);

        // We expect these to be extremely close, since we got s from the orbit itself
        assert_vectors_close(&initial_position, state.get_position(), 1e-14);
        assert_vectors_close(&initial_velocity, state.get_velocity(), 1e-14);

        // Time is a little fuzzier, because the velocity constant (and thus this orbit) isn't
        // perfect.
        let computed_period = (4.0 * PI * PI * KERBIN_ORBIT_RADIUS.powi(3) / KERBOL_MU).sqrt();
        println!(
            "Periods:\n\
            \t(defined)   {}\n\
            \t(computed)  {}\n\
            \t(simulated) {}",
            KERBIN_ORBIT_PERIOD,
            computed_period,
            state.get_time()
        );
        assert_close(KERBIN_ORBIT_PERIOD, state.get_time(), 1e-6);
    }

    #[test]
    fn test_perfect_circle() {
        // Build a perfectly circular orbit, and test what it looks like as we move
        // incrementally around.
        let radius = 10.0 * KERBIN_ORBIT_RADIUS; // some number
        let velocity = (KERBOL_MU / radius).sqrt(); // v^2 = mu (2/r - 1/a) = mu / r

        let initial_position = Vector3::x() * radius;
        let initial_velocity = Vector3::z() * velocity;
        let mut state = State::new(initial_position, initial_velocity, 0.0, KERBOL_MU);

        // Compute s for a whole orbit. Since r doesn't change, s varies linearly with t.
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();

        let num_points = 1000;
        for i in 0..num_points {
            let num_points = num_points as f64;

            state.advance_s(s / num_points);

            let theta = 2.0 * PI * (i + 1) as f64 / num_points;
            let expected = radius * Vector3::new(theta.cos(), 0.0, theta.sin());
            assert_vectors_close(&expected, state.get_position(), 1e-14);
        }

        // Check that the expected amount of time has elapsed. Not sure why we get slightly
        // less precision here, but whatever. Maybe it's just adding something to itself
        // a thousand times.
        let computed_period = 2.0 * PI * radius / velocity;
        assert_close(computed_period, state.get_time(), 1e-12);
    }
}
