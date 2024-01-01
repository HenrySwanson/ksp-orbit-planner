use super::{HasMass, Orbit};
use crate::astro::state::CartesianState;

#[derive(Debug, Clone)]
pub struct TimedOrbit<P, S> {
    orbit: Orbit<P, S>,
    time_at_periapsis: f64,
}

impl<P, S> TimedOrbit<P, S> {
    pub fn from_orbit(orbit: Orbit<P, S>, time_at_periapsis: f64) -> Self {
        Self {
            orbit,
            time_at_periapsis,
        }
    }

    pub fn orbit(&self) -> &Orbit<P, S> {
        &self.orbit
    }

    pub fn with_primary<P2>(self, new_primary: P2) -> TimedOrbit<P2, S> {
        TimedOrbit {
            orbit: self.orbit.with_primary(new_primary),
            time_at_periapsis: self.time_at_periapsis,
        }
    }

    pub fn with_secondary<S2>(self, new_secondary: S2) -> TimedOrbit<P, S2> {
        TimedOrbit {
            orbit: self.orbit.with_secondary(new_secondary),
            time_at_periapsis: self.time_at_periapsis,
        }
    }
}

impl<P: HasMass, S> TimedOrbit<P, S> {
    pub fn state_at_time(&self, time: f64) -> CartesianState<&P> {
        self.orbit.get_state_at_tsp(time - self.time_at_periapsis)
    }

    pub fn s_at_time(&self, time: f64) -> f64 {
        self.orbit.tsp_to_s(time - self.time_at_periapsis)
    }

    pub fn time_at_s(&self, s: f64) -> f64 {
        self.time_at_periapsis + self.orbit.s_to_tsp(s)
    }
}

impl<P: HasMass> TimedOrbit<P, ()> {
    pub fn from_state(state: CartesianState<P>, current_time: f64) -> Self {
        // Save for later
        let position = state.position();

        // Getting the orbit we can do without much fuss
        let orbit = state.into_orbit();

        // Finding the anomaly around the orbit takes some more work. Note that
        // the value of the anomaly for some orbits (e.g. circular) depends on which
        // rotation we took.
        let pos_in_plane = orbit.rotation.inverse_transform_vector(&position);

        // TODO: find something that works for radial orbits!
        let theta = pos_in_plane.y.atan2(pos_in_plane.x);
        let tan_half_theta = (theta / 2.0).tan();
        let h = orbit.angular_momentum();
        let r_p = orbit.periapsis();
        let g2_over_g1 = r_p / h * tan_half_theta;

        let beta: f64 = orbit.beta();
        let beta_sqrt = beta.abs().sqrt();
        let s = if beta > 0.0 {
            // Elliptic: g2/g1 = tan(s sqrt(beta) / 2) / sqrt(beta)
            (g2_over_g1 * beta_sqrt).atan() * 2.0 / beta_sqrt
        } else if beta < 0.0 {
            // Hyperbolic: g2/g1 = tanh(s sqrt(-beta) / 2) / sqrt(-beta)
            (g2_over_g1 * beta_sqrt).atanh() * 2.0 / beta_sqrt
        } else {
            // Parabolic: s = h/mu tan_half_theta, and r_p = h^2/2mu, so
            // g2/g1 = r_p/h mu/h s = s/2
            2.0 * tan_half_theta
        };

        let time_since_periapsis = orbit.s_to_tsp(s);
        Self::from_orbit(orbit, current_time - time_since_periapsis)
    }
}
