mod camera;
mod renderer;
mod renderers;
mod simulation;

pub use simulation::Simulation;

use crate::astro::orbit::{PhysicalOrbit, PointMass, TimedOrbit};
use crate::orrery::{BodyID, PrimaryBody};

// TODO: re-evaluate if we need this
#[derive(Debug, Clone)]
pub struct OrbitPatch {
    pub orbit: PhysicalOrbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub parent_id: BodyID,
}

impl OrbitPatch {
    pub fn new(orbit: &TimedOrbit<PrimaryBody, ()>, start_time: f64) -> OrbitPatch {
        let start_anomaly = orbit.s_at_time(start_time);
        let parent_id = orbit.orbit().primary().id;
        let mu = orbit.orbit().primary().mu;

        Self {
            orbit: orbit.orbit().clone().with_primary(PointMass::with_mu(mu)),
            start_anomaly,
            end_anomaly: None,
            parent_id,
        }
    }
}
