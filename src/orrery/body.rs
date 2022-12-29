use nalgebra::Point3;

use crate::astro::orbit::{Orbit, PointMass};

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
    state: CartesianState,
    orbit: Orbit<PointMass, ()>,
    time_at_periapsis: f64,
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

    pub fn state(&self) -> Option<&CartesianState> {
        self.get_orbiting_data().map(|x| &x.state)
    }

    pub fn get_orbiting_data(&self) -> Option<&OrbitingData> {
        match &self.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting(x) => Some(x),
        }
    }
}

impl OrbitingData {
    pub fn new(parent_id: BodyID, state: CartesianState, current_time: f64) -> Self {
        let orbit = state.get_orbit();
        let s = state.get_universal_anomaly();
        let time_until_periapsis = state.delta_s_to_t(-s);

        Self {
            parent_id,
            state,
            orbit,
            time_at_periapsis: current_time + time_until_periapsis,
            current_time,
        }
    }

    pub fn get_orbit_patch(&self) -> OrbitPatch {
        let time_since_periapsis = self.current_time - self.time_at_periapsis;
        let orbit = self.get_orbit();
        let start_anomaly = self.orbit.tsp_to_s(time_since_periapsis);

        OrbitPatch {
            orbit,
            start_anomaly,
            end_anomaly: None,
            parent_id: self.parent_id,
        }
    }

    pub fn parent_id(&self) -> BodyID {
        self.parent_id
    }

    pub fn state(&self) -> CartesianState {
        let time_since_periapsis = self.current_time - self.time_at_periapsis;
        let state = self.orbit.get_state_at_tsp(time_since_periapsis);

        {
            let p1 = self.state.position();
            let p2 = state.position();
            let diff = (p1 - p2) / p1.norm().max(p2.norm());
            if diff.norm() > 1e-7 {
                println!("pos diff: {:?}", diff)
            }
        }

        state
    }

    pub fn get_orbit(&self) -> Orbit<PointMass, ()> {
        self.orbit.clone()
    }

    pub fn update_t_mut(&mut self, delta_t: f64) {
        self.state.update_t_mut(delta_t);
        self.current_time += delta_t;
    }
}
