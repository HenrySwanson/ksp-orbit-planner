use nalgebra::Point3;

use crate::astro::orbit::{Orbit, PointMass, TimedOrbit};

use super::{state::CartesianState, OrbitPatch};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

// All the immutable info about a body
#[derive(Debug, Clone)]
pub struct BodyInfo {
    pub name: String,
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

#[derive(Debug, Clone)]
pub enum BodyState {
    FixedAtOrigin,
    Orbiting(OrbitingData),
}

// TODO: come up with better name? generalize to ships?
#[derive(Debug, Clone)]
pub struct OrbitingData {
    parent_id: BodyID,
    orbit: TimedOrbit<PointMass, ()>,
    // TODO: can we drop this and get it straight from the orrery?
    current_time: f64,
}

#[derive(Debug, Clone)]
pub struct Body {
    pub id: BodyID,
    pub info: BodyInfo,
    pub state: BodyState,
}

impl Body {
    pub fn parent_id(&self) -> Option<BodyID> {
        self.get_orbiting_data().map(|x| x.parent_id)
    }

    pub fn state(&self) -> Option<CartesianState> {
        self.get_orbiting_data().map(|x| x.state())
    }

    pub fn get_orbiting_data(&self) -> Option<&OrbitingData> {
        match &self.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting(x) => Some(x),
        }
    }
}

impl OrbitingData {
    pub fn from_orbit(
        parent_id: BodyID,
        orbit: Orbit<PointMass, ()>,
        time_since_periapsis: f64,
        current_time: f64,
    ) -> Self {
        Self {
            parent_id,
            orbit: TimedOrbit::from_orbit(orbit, current_time - time_since_periapsis),
            current_time,
        }
    }

    pub fn from_state(parent_id: BodyID, state: CartesianState, current_time: f64) -> Self {
        let orbit = state.get_orbit();
        let time_since_periapsis = orbit.s_to_tsp(state.get_universal_anomaly());
        Self::from_orbit(parent_id, orbit, time_since_periapsis, current_time)
    }

    pub fn get_orbit_patch(&self) -> OrbitPatch {
        let start_anomaly = self.orbit.s_at_time(self.current_time);

        OrbitPatch {
            orbit: self.orbit(),
            start_anomaly,
            end_anomaly: None,
            parent_id: self.parent_id,
        }
    }

    pub fn parent_id(&self) -> BodyID {
        self.parent_id
    }

    pub fn state(&self) -> CartesianState {
        self.orbit.state_at_time(self.current_time)
    }

    pub fn state_at_time(&self, time: f64) -> CartesianState {
        self.orbit.state_at_time(time)
    }

    pub fn timed_orbit(&self) -> &TimedOrbit<PointMass, ()> {
        &self.orbit
    }

    pub fn orbit(&self) -> Orbit<PointMass, ()> {
        self.orbit.orbit().clone()
    }

    pub fn update_t_mut(&mut self, delta_t: f64) {
        self.current_time += delta_t;
    }
}
