use nalgebra::Point3;

use super::body::BodyID;
use super::ship::ShipID;

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

#[derive(Debug, Clone)]
pub struct ReverseEvent {
    pub event: Event,
    pub ship_id: ShipID,
    pub previous_soi_body: Option<BodyID>,
}

impl Event {
    pub fn compare_time(&self, other: &Event) -> std::cmp::Ordering {
        self.time.partial_cmp(&other.time).unwrap()
    }
}
