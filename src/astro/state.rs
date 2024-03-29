use nalgebra::Vector3;

use super::{HasMass, Orbit};

#[derive(Debug, Clone)]
pub struct CartesianState<P> {
    primary: P,
    // TODO secondary: S,
    position: Vector3<f64>,
    velocity: Vector3<f64>,
}

impl<P> CartesianState<P> {
    pub fn new(primary: P, position: Vector3<f64>, velocity: Vector3<f64>) -> Self {
        CartesianState {
            primary,
            position,
            velocity,
        }
    }

    pub fn position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn velocity(&self) -> Vector3<f64> {
        self.velocity
    }
}

impl<P: HasMass> CartesianState<P> {
    pub fn to_orbit(&self) -> Orbit<&P, ()> {
        Orbit::from_cartesian(&self.primary, (), &self.position, &self.velocity)
    }

    pub fn into_orbit(self) -> Orbit<P, ()> {
        Orbit::from_cartesian(self.primary, (), &self.position, &self.velocity)
    }
}

// TODO: see how many of these are actually needed outside testing
#[cfg(test)]
impl<P: HasMass> CartesianState<P> {
    fn energy(&self) -> f64 {
        // KE = 1/2 v^2, PE = - mu/r
        self.velocity.norm_squared() / 2.0 - self.primary.mu() / self.position.norm()
    }

    #[allow(non_snake_case)]
    fn update_s_mut(&mut self, delta_s: f64) -> f64 {
        use crate::math::stumpff::stumpff_G;

        let beta = -2.0 * self.energy();
        let mu = self.primary.mu();
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
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use approx::assert_relative_eq;

    use super::*;
    use crate::astro::PointMass;
    use crate::consts::{
        get_circular_velocity, get_period, KERBIN_ORBIT_PERIOD, KERBIN_ORBIT_RADIUS, KERBOL_MU,
    };
    use crate::testing_utils::assert_vectors_close;

    #[test]
    fn test_kerbin() {
        // Build Kerbin and see if the orbit simulation is right.
        // Kerbin's orbit is perfectly circular.
        let initial_position = Vector3::x() * KERBIN_ORBIT_RADIUS;
        let initial_velocity = Vector3::y() * get_circular_velocity(KERBIN_ORBIT_RADIUS, KERBOL_MU);

        let mut state = CartesianState::new(
            PointMass::with_mu(KERBOL_MU),
            initial_position,
            initial_velocity,
        );
        let mut elapsed_time = 0.0;

        // Advance for one full orbit.
        // This is a circular orbit, so s is proportional to theta. Specifically,
        // s = theta / sqrt(beta).
        let beta = -2.0 * state.energy();
        let s = 2.0 * PI / beta.sqrt();
        elapsed_time += state.update_s_mut(s);

        // We expect these to be extremely close, since we got s from the orbit itself
        assert_vectors_close(&initial_position, &state.position(), 1e-14);
        assert_vectors_close(&initial_velocity, &state.velocity(), 1e-14);

        // Time is a little fuzzier, because the velocity constant (and thus this orbit)
        // isn't perfect.
        let computed_period = get_period(KERBIN_ORBIT_RADIUS, KERBOL_MU);
        println!(
            "Periods:\n\
            \t(defined)   {}\n\
            \t(computed)  {}\n\
            \t(simulated) {}",
            KERBIN_ORBIT_PERIOD, computed_period, elapsed_time,
        );
        assert_relative_eq!(KERBIN_ORBIT_PERIOD, elapsed_time, max_relative = 1e-6);
    }

    #[test]
    fn test_perfect_circle() {
        // Build a perfectly circular orbit, and test what it looks like as we move
        // incrementally around.
        let radius = 10.0 * KERBIN_ORBIT_RADIUS; // some number
        let velocity = (KERBOL_MU / radius).sqrt(); // v^2 = mu (2/r - 1/a) = mu / r

        let initial_position = Vector3::x() * radius;
        let initial_velocity = Vector3::z() * velocity;
        let mut state = CartesianState::new(
            PointMass::with_mu(KERBOL_MU),
            initial_position,
            initial_velocity,
        );
        let mut elapsed_time = 0.0;

        // Compute s for a whole orbit. Since r doesn't change, s varies linearly with
        // t.
        let beta = -2.0 * state.energy();
        let s = 2.0 * PI / beta.sqrt();

        let num_points = 1000;
        for i in 0..num_points {
            let num_points = num_points as f64;

            elapsed_time += state.update_s_mut(s / num_points);

            let theta = 2.0 * PI * (i + 1) as f64 / num_points;
            let expected = radius * Vector3::new(theta.cos(), 0.0, theta.sin());
            assert_vectors_close(&expected, &state.position(), 1e-14);
        }

        // Check that the expected amount of time has elapsed. Not sure why we get
        // slightly less precision here, but whatever. Maybe it's just adding
        // something to itself a thousand times.
        let computed_period = 2.0 * PI * radius / velocity;
        assert_relative_eq!(computed_period, elapsed_time, max_relative = 1e-12);
    }
}
