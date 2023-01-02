use nalgebra::Point3;

use crate::astro::{
    orbit::{HasMass, Orbit, PointMass, TimedOrbit},
    state::CartesianState,
};

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

#[derive(Debug, Clone)]
pub struct PrimaryBody {
    pub id: BodyID,
    pub mu: f64,
}

// TODO: come up with better name? generalize to ships?
#[derive(Debug, Clone)]
pub struct OrbitingData {
    parent_id: BodyID,
    orbit: TimedOrbit<PointMass, ()>,
}

#[derive(Debug, Clone)]
pub struct Body {
    pub id: BodyID,
    pub info: BodyInfo,
    state: BodyState,
}

impl HasMass for PrimaryBody {
    fn mu(&self) -> f64 {
        self.mu
    }
}

impl Body {
    pub fn new(id: BodyID, info: BodyInfo, state: BodyState) -> Self {
        Self { id, info, state }
    }

    pub fn parent_id(&self) -> Option<BodyID> {
        self.odata().map(|odata| odata.parent_id)
    }

    pub fn orbit(&self) -> Option<TimedOrbit<PrimaryBody, ()>> {
        self.odata().map(|odata| {
            odata.timed_orbit().clone().with_primary(PrimaryBody {
                id: odata.parent_id,
                mu: odata.orbit().primary().mu(),
            })
        })
    }

    fn odata(&self) -> Option<&OrbitingData> {
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
        time_at_periapsis: f64,
    ) -> Self {
        Self {
            parent_id,
            orbit: TimedOrbit::from_orbit(orbit, time_at_periapsis),
        }
    }

    pub fn from_state(parent_id: BodyID, state: CartesianState, current_time: f64) -> Self {
        let orbit = state.get_orbit();
        let time_since_periapsis = orbit.s_to_tsp(state.get_universal_anomaly());
        Self::from_orbit(parent_id, orbit, current_time - time_since_periapsis)
    }

    pub fn parent_id(&self) -> BodyID {
        self.parent_id
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
}
