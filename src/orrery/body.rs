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
    pub fn new(parent_id: BodyID, state: CartesianState) -> Self {
        Self { parent_id, state }
    }

    pub fn get_orbit_patch(&self) -> OrbitPatch {
        let orbit = self.get_orbit();
        let start_anomaly = self.state.get_universal_anomaly();

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
        self.state.clone()
    }

    pub fn get_orbit(&self) -> Orbit<PointMass, ()> {
        self.state.get_orbit()
    }

    pub fn update_t_mut(&mut self, delta_t: f64) {
        self.state.update_t_mut(delta_t)
    }
}
