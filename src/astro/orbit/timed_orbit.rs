use crate::orrery::CartesianState;

use super::{HasMass, Orbit};

#[derive(Debug, Clone)]
pub struct TimedOrbit<P, S> {
    orbit: Orbit<P, S>,
    time_at_periapsis: f64,
}

impl<P: HasMass, S> TimedOrbit<P, S> {
    pub fn from_orbit(orbit: Orbit<P, S>, time_at_periapsis: f64) -> Self {
        Self {
            orbit,
            time_at_periapsis,
        }
    }

    pub fn orbit(&self) -> &Orbit<P, S> {
        &self.orbit
    }

    pub fn state_at_time(&self, time: f64) -> CartesianState {
        self.orbit.get_state_at_tsp(time - self.time_at_periapsis)
    }

    pub fn s_at_time(&self, time: f64) -> f64 {
        self.orbit.tsp_to_s(time - self.time_at_periapsis)
    }

    pub fn time_at_s(&self, s: f64) -> f64 {
        self.time_at_periapsis + self.orbit.s_to_tsp(s)
    }
}
