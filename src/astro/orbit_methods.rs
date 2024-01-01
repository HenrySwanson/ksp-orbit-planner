use nalgebra::Vector3;

use super::{HasMass, OrbitBase};
use crate::astro::state::CartesianState;
use crate::math::root_finding::{find_root_bracket, newton_plus_bisection};
use crate::math::stumpff::stumpff_G;

const NUM_ITERATIONS_DELTA_T: usize = 2000;

impl<P, S, E> OrbitBase<P, S, E> {
    pub fn get_position_at_theta(&self, theta: f64) -> Option<Vector3<f64>> {
        if self.semilatus_rectum() == 0.0 {
            // Radial orbits
            return None;
        }

        let denominator = 1.0 + self.eccentricity() * theta.cos();
        if denominator <= 0.0 {
            // Happens when we're hyperbolic/parabolic, and we're larger than max anomaly
            return None;
        }

        let radius = self.semilatus_rectum() / denominator;
        let position = radius * Vector3::new(theta.cos(), theta.sin(), 0.0);

        Some(self.rotation() * position)
    }
}

impl<P: HasMass, S, E> OrbitBase<P, S, E> {
    #[allow(non_snake_case)]
    pub fn get_state_native_frame(&self, s: f64) -> CartesianState<&P> {
        // Note: this function is only exposed because it makes rendering faster. Would
        // be nice to have an alternative... :\

        let mu = self.primary().mu();
        let beta = self.beta();
        let h = self.angular_momentum();
        let G: [f64; 4] = stumpff_G(beta, s);

        // Get the position at s
        let x = self.periapsis() - mu * G[2];
        let y = h * G[1];
        let r = (x * x + y * y).sqrt();
        let vx = -mu / r * G[1];
        let vy = h / r * G[0];

        let position = Vector3::new(x, y, 0.0);
        let velocity = Vector3::new(vx, vy, 0.0);

        CartesianState::new(self.primary(), position, velocity)
    }

    pub fn get_state_at_universal_anomaly(&self, s: f64) -> CartesianState<&P> {
        let native_state = self.get_state_native_frame(s);

        let position = self.rotation() * native_state.position();
        let velocity = self.rotation() * native_state.velocity();

        CartesianState::new(self.primary(), position, velocity)
    }

    pub fn get_state_at_tsp(&self, time_since_periapsis: f64) -> CartesianState<&P> {
        // First we find the s corresponding to the time
        let s = self.tsp_to_s(time_since_periapsis);
        self.get_state_at_universal_anomaly(s)
    }

    pub fn get_state_at_theta(&self, theta: f64) -> (Vector3<f64>, Vector3<f64>) {
        // Taken from https://www.mathworks.com/matlabcentral/fileexchange/35455-convert-keplerian-orbital-elements-to-a-state-vector
        let p = self.semilatus_rectum();
        let ecc = self.eccentricity();
        let mu = self.primary().mu();
        let rotation = self.rotation();

        let position = self.get_position_at_theta(theta).unwrap();
        let velocity =
            rotation * ((mu / p).sqrt() * Vector3::new(-theta.sin(), ecc + theta.cos(), 0.0));

        (position, velocity)
    }

    /// Returns t(s) and t'(s), evaluted at the given point, where
    /// t(s) = r_0 * G_1(β, s) + r_0 * r_dot_0 * G_2(β, s) + mu * G_3(β, s)
    #[allow(non_snake_case)]
    fn ts_and_derivative(&self, s: f64) -> (f64, f64) {
        // Grab some constants
        let beta = -2.0 * self.energy();
        let mu = self.primary().mu();
        let r_p = self.periapsis();

        let G = stumpff_G(beta, s);
        let t = r_p * G[1] + mu * G[3];
        let t_prime = r_p * G[0] + mu * G[2];

        (t, t_prime)
    }

    #[allow(non_snake_case)]
    pub fn tsp_to_s(&self, time_since_periapsis: f64) -> f64 {
        if time_since_periapsis == 0.0 {
            return 0.0;
        }

        // We want to find a root of this function, which is monotonically increasing:
        let f_and_f_prime = |s: f64| {
            let (t, t_prime) = self.ts_and_derivative(s);
            (t - time_since_periapsis, t_prime)
        };

        // TODO if these fail, we need to log the parameters somewhere :\
        let center = time_since_periapsis / self.periapsis();
        let bracket = find_root_bracket(
            |x| f_and_f_prime(x).0,
            center,
            center,
            NUM_ITERATIONS_DELTA_T,
        );
        newton_plus_bisection(f_and_f_prime, bracket, NUM_ITERATIONS_DELTA_T)
    }

    pub fn s_to_tsp(&self, s: f64) -> f64 {
        self.ts_and_derivative(s).0
    }

    #[allow(non_snake_case)]
    pub fn get_s_at_radius(&self, radius: f64) -> Option<f64> {
        // We can actually solve this one exactly, using the relationship between r and
        // s

        // Grab some constants
        let mu = self.primary().mu();
        let ecc = self.eccentricity();
        let beta = -2.0 * self.energy();
        let r_p = self.periapsis();

        // r = r_p + mu e G_2, so...
        let desired_G2 = (radius - r_p) / mu / ecc;

        // Split into cases
        let s = match beta.partial_cmp(&0.0).unwrap() {
            std::cmp::Ordering::Less => {
                // Hyperbola: G2 = (1 - cosh(s sqrt -beta)) / beta
                let tmp = 1.0 - desired_G2 * beta;
                // Is this in range for acosh?
                if tmp < 1.0 {
                    return None;
                }
                tmp.acosh() / (-beta).sqrt()
            }
            std::cmp::Ordering::Equal => {
                // Parabola: G2 = s^2 / 2
                (desired_G2 * 2.0).sqrt()
            }
            std::cmp::Ordering::Greater => {
                // Ellipse: G2 = (1 - cos(s sqrt beta)) / beta
                let tmp = 1.0 - desired_G2 * beta;
                // Is this in range for acos?
                if tmp.abs() > 1.0 {
                    return None;
                }
                tmp.acos() / beta.sqrt()
            }
        };

        Some(s)
    }
}
