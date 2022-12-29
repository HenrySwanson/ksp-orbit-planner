use nalgebra::Vector3;

use crate::{
    math::{anomaly, stumpff::stumpff_G},
    orrery::CartesianState,
};

use super::{HasMass, Orbit};

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
    pub fn get_state_native_frame(&self, s: f64) -> CartesianState {
        // Note: this function is only exposed because it makes rendering faster. Would be
        // nice to have an alternative... :\

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

        CartesianState::new(position, velocity, mu)
    }

    pub fn get_state_at_universal_anomaly(&self, s: f64) -> CartesianState {
        let native_state = self.get_state_native_frame(s);

        let position = self.rotation * native_state.position();
        let velocity = self.rotation * native_state.velocity();

        CartesianState::new(position, velocity, self.primary.mu())
    }

    pub fn get_state_at_tsp(&self, time_since_periapsis: f64) -> CartesianState {
        // TODO: ugly hack, fix this
        self.get_state_at_universal_anomaly(0.0)
            .update_t(time_since_periapsis)
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

    pub fn tsp_to_s(&self, time_since_periapsis: f64) -> f64 {
        // TODO: ugly hack, fix this
        self.get_state_at_universal_anomaly(0.0)
            .delta_t_to_s(time_since_periapsis)
    }
}
