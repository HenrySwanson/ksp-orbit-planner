mod camera;
mod controller;
mod renderer;
mod renderers;
mod simulation;
mod view;

pub use simulation::Simulation;

use crate::astro::orbit::{PhysicalOrbit, PointMass, TimedOrbit};
use crate::model::orrery::{Body, BodyID};

// TODO: re-evaluate if we need this
#[derive(Debug, Clone)]
pub struct OrbitPatch {
    pub orbit: PhysicalOrbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub parent_id: BodyID,
}

impl OrbitPatch {
    pub fn new<S: Clone>(orbit: &TimedOrbit<Body, S>, start_time: f64) -> OrbitPatch {
        let start_anomaly = orbit.s_at_time(start_time);
        let parent_id = orbit.orbit().primary().id;

        Self {
            orbit: orbit
                .orbit()
                .clone()
                .map_primary(|p| PointMass::with_mu(p.info.mu))
                .with_secondary(()),
            start_anomaly,
            end_anomaly: None,
            parent_id,
        }
    }
}
