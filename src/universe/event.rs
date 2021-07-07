use nalgebra::Point3;

use super::universe::BodyID;

#[derive(Debug, Clone)]
pub struct Event {
    pub kind: EventKind,
    pub time: f64,
    pub anomaly: f64,
    pub location: Point3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub enum EventKind {
    EnteringSOI(BodyID),
    ExitingSOI,
}
