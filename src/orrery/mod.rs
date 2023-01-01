use crate::astro::orbit::PhysicalOrbit;

mod body;
mod orrery;
mod ship;

// TODO restrict imports!
pub use body::{Body, BodyID, BodyInfo, BodyState};
pub use orrery::{Frame, FramedState, Orrery};
pub use ship::{Ship, ShipID};

// TODO: re-evaluate if we need this
#[derive(Debug, Clone)]
pub struct OrbitPatch {
    pub orbit: PhysicalOrbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub parent_id: BodyID,
}
