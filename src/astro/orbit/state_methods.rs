use nalgebra::Vector3;

use super::{HasMass, Orbit};
use crate::astro::state::CartesianState;
use crate::math::anomaly;
use crate::math::root_finding::{find_root_bracket, newton_plus_bisection};
use crate::math::stumpff::stumpff_G;

const NUM_ITERATIONS_DELTA_T: usize = 2000;

impl<P, S> Orbit<P, S> {
    pub fn get_position_at_theta(&self, theta: f64) -> Option<Vector3<f64>> {
        if self.slr == 0.0 {
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

impl<P: HasMass, S> Orbit<P, S> {
    pub fn true_to_universal(&self, true_anomaly: f64) -> f64 {
        let eccentricity = self.eccentricity();
        let energy = self.energy();
        if eccentricity < 1.0 {
            let ecc = anomaly::true_to_eccentric(true_anomaly, eccentricity);
            anomaly::eccentric_to_universal(ecc, energy)
        } else if eccentricity > 1.0 {
            let hyp = anomaly::true_to_hyperbolic(true_anomaly, eccentricity);
            anomaly::hyperbolic_to_universal(hyp, energy)
        } else {
            let para = anomaly::true_to_parabolic(true_anomaly);
            anomaly::parabolic_to_universal(para, self.angular_momentum(), self.primary.mu())
        }
    }

    pub fn universal_to_true(&self, universal_anomaly: f64) -> f64 {
        let eccentricity = self.eccentricity();
        let energy = self.energy();
        if eccentricity < 1.0 {
            let ecc = anomaly::universal_to_eccentric(universal_anomaly, energy);
            anomaly::eccentric_to_true(ecc, eccentricity)
        } else if eccentricity > 1.0 {
            let hyp = anomaly::universal_to_hyperbolic(universal_anomaly, energy);
            anomaly::hyperbolic_to_true(hyp, eccentricity)
        } else {
            let para = anomaly::universal_to_parabolic(
                universal_anomaly,
                self.angular_momentum(),
                self.primary.mu(),
            );
            anomaly::parabolic_to_true(para)
        }
    }

    #[allow(non_snake_case)]
    pub fn get_state_native_frame(&self, s: f64) -> CartesianState<&P> {
        // Note: this function is only exposed because it makes rendering faster. Would
        // be nice to have an alternative... :\

        let mu = self.primary.mu();
        let beta = mu * self.alpha;
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

        CartesianState::new(&self.primary, position, velocity)
    }

    pub fn get_state_at_universal_anomaly(&self, s: f64) -> CartesianState<&P> {
        let native_state = self.get_state_native_frame(s);

        let position = self.rotation * native_state.position();
        let velocity = self.rotation * native_state.velocity();

        CartesianState::new(&self.primary, position, velocity)
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
        let mu = self.primary.mu();
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
        let mu = self.primary.mu();
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

    pub fn get_s_at_radius(&self, radius: f64) -> Option<f64> {
        // TODO this doesn't work well for radial orbits, try to adjust it so that it
        // does

        // Since (h^2/mu) / (1 + e cos theta) = r, we can invert that to get
        // a desired theta, which will always be in the first or second quadrant
        let cos_theta = (self.semilatus_rectum() / radius - 1.0) / self.eccentricity();
        if cos_theta.abs() > 1.0 {
            return None;
        }

        let theta = cos_theta.acos();
        Some(self.true_to_universal(theta))
    }
}
