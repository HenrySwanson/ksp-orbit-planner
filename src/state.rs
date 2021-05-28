use kiss3d::nalgebra as na;
use na::Vector3;

use crate::orbit::Orbit;
use crate::root_finding::{find_root_bracket, newton_plus_bisection};
use crate::stumpff::stumpff_G;

pub struct CartesianState {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    parent_mu: f64,
    // TODO: maybe track time that we couldn't account for in advance_s?
}

impl CartesianState {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, parent_mu: f64) -> Self {
        CartesianState {
            position,
            velocity,
            parent_mu,
        }
    }

    pub fn get_position(&self) -> &Vector3<f64> {
        &self.position
    }

    pub fn get_velocity(&self) -> &Vector3<f64> {
        &self.velocity
    }

    pub fn get_mu(&self) -> f64 {
        self.parent_mu
    }

    pub fn get_orbit(&self) -> Orbit {
        Orbit::from_cartesian(self.get_position(), self.get_velocity(), self.get_mu())
    }

    pub fn get_energy(&self) -> f64 {
        // KE = 1/2 v^2, PE = - mu/r
        self.velocity.norm_squared() / 2.0 - self.parent_mu / self.position.norm()
    }

    #[allow(non_snake_case)]
    pub fn advance_s(&mut self, delta_s: f64) -> f64 {
        let beta = -2.0 * self.get_energy();
        let mu = self.parent_mu;
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

        // Return the elapsed time, in case anyone's interested
        delta_t
    }

    #[allow(non_snake_case)]
    pub fn advance_t(&mut self, delta_t: f64) {
        // We find the delta_s corresponding to the given delta_t, and advance using that.
        // Since ds/dt = 1/r, s and t are monotonically related, so there's a unique solution.

        // Grab some constants
        let beta = -2.0 * self.get_energy();
        let mu = self.parent_mu;
        let r_0 = self.position.norm();
        let r_dot_0 = self.position.dot(&self.velocity) / r_0;

        // We want to find a root of this function, which is monotonically increasing:
        //  r_0 * G_1(β, s) + r_0 * r_dot_0 * G_2(β, s) + mu * G_3(β, s) - delta_t
        let f_and_f_prime = |s: f64| {
            let G = stumpff_G(beta, s);
            let f = r_0 * G[1] + r_0 * r_dot_0 * G[2] + mu * G[3] - delta_t;
            let f_prime = r_0 * G[0] + r_0 * r_dot_0 * G[1] + mu * G[2];
            (f, f_prime)
        };

        // TODO if these fail, we need to log the parameters somewhere :\
        let bracket = find_root_bracket(|x| f_and_f_prime(x).0, delta_t / r_0, delta_t / r_0);
        let root = newton_plus_bisection(f_and_f_prime, bracket, 100);

        self.advance_s(root);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::consts::{
        get_circular_velocity, get_period, KERBIN_ORBIT_PERIOD, KERBIN_ORBIT_RADIUS, KERBOL_MU,
    };

    use std::f64::consts::PI;

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
        let initial_velocity = Vector3::y() * get_circular_velocity(KERBIN_ORBIT_RADIUS, KERBOL_MU);

        let mut state = CartesianState::new(initial_position, initial_velocity, KERBOL_MU);
        let mut elapsed_time = 0.0;

        // Advance for one full orbit.
        // This is a circular orbit, so s is proportional to theta. Specifically,
        // s = theta / sqrt(beta).
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();
        elapsed_time += state.advance_s(s);

        // We expect these to be extremely close, since we got s from the orbit itself
        assert_vectors_close(&initial_position, state.get_position(), 1e-14);
        assert_vectors_close(&initial_velocity, state.get_velocity(), 1e-14);

        // Time is a little fuzzier, because the velocity constant (and thus this orbit) isn't
        // perfect.
        let computed_period = get_period(KERBIN_ORBIT_RADIUS, KERBOL_MU);
        println!(
            "Periods:\n\
            \t(defined)   {}\n\
            \t(computed)  {}\n\
            \t(simulated) {}",
            KERBIN_ORBIT_PERIOD, computed_period, elapsed_time,
        );
        assert_close(KERBIN_ORBIT_PERIOD, elapsed_time, 1e-6);
    }

    #[test]
    fn test_perfect_circle() {
        // Build a perfectly circular orbit, and test what it looks like as we move
        // incrementally around.
        let radius = 10.0 * KERBIN_ORBIT_RADIUS; // some number
        let velocity = (KERBOL_MU / radius).sqrt(); // v^2 = mu (2/r - 1/a) = mu / r

        let initial_position = Vector3::x() * radius;
        let initial_velocity = Vector3::z() * velocity;
        let mut state = CartesianState::new(initial_position, initial_velocity, KERBOL_MU);
        let mut elapsed_time = 0.0;

        // Compute s for a whole orbit. Since r doesn't change, s varies linearly with t.
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();

        let num_points = 1000;
        for i in 0..num_points {
            let num_points = num_points as f64;

            elapsed_time += state.advance_s(s / num_points);

            let theta = 2.0 * PI * (i + 1) as f64 / num_points;
            let expected = radius * Vector3::new(theta.cos(), 0.0, theta.sin());
            assert_vectors_close(&expected, state.get_position(), 1e-14);
        }

        // Check that the expected amount of time has elapsed. Not sure why we get slightly
        // less precision here, but whatever. Maybe it's just adding something to itself
        // a thousand times.
        let computed_period = 2.0 * PI * radius / velocity;
        assert_close(computed_period, elapsed_time, 1e-12);
    }
}
