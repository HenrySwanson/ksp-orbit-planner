use nalgebra::Point3;

use super::body::BodyID;
use super::ship::ShipID;

#[derive(Debug, Clone, Copy)]
pub enum EventKind {
    EnteringSOI(BodyID),
    ExitingSOI,
}

#[derive(Debug, Clone)]
pub struct EventPoint {
    pub time: f64,
    pub anomaly: f64,
    pub location: Point3<f64>,
}

#[derive(Debug, Clone)]
pub struct Event {
    pub ship_id: ShipID,
    pub kind: EventKind,
    pub point: EventPoint,
}

#[derive(Debug, Clone)]
pub struct ReverseEvent {
    pub event: Event,
    pub previous_soi_body: Option<BodyID>,
}

impl EventPoint {
    pub fn compare_time(&self, other: &EventPoint) -> std::cmp::Ordering {
        self.time.partial_cmp(&other.time).unwrap()
    }
}
