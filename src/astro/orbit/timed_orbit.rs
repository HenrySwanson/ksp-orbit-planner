use crate::astro::state::CartesianState;

use super::{HasMass, Orbit};

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

// TODO: how can i drop this clone bound?
impl<P: HasMass + Clone, S> TimedOrbit<P, S> {
    pub fn state_at_time(&self, time: f64) -> CartesianState<P>
    where
        P: Clone,
    {
        self.orbit.get_state_at_tsp(time - self.time_at_periapsis)
    }

    pub fn s_at_time(&self, time: f64) -> f64 {
        self.orbit.tsp_to_s(time - self.time_at_periapsis)
    }

    pub fn time_at_s(&self, s: f64) -> f64 {
        self.time_at_periapsis + self.orbit.s_to_tsp(s)
    }
}

impl<P: HasMass + Clone> TimedOrbit<P, ()> {
    // TODO: adjust with primary and secondary
    pub fn from_state(state: CartesianState<P>, current_time: f64) -> Self {
        let orbit = state.get_orbit();
        let time_since_periapsis = orbit.s_to_tsp(state.get_universal_anomaly());
        Self::from_orbit(orbit, current_time - time_since_periapsis)
    }
}
