use std::borrow::Borrow;

use nalgebra::Point3;

use crate::orrery::BodyID;
use crate::orrery::ShipID;

#[derive(Debug, Clone)]
pub struct SOIChange {
    pub old: BodyID,
    pub new: BodyID,
}

#[derive(Debug, Clone)]
pub enum EventData {
    EnteringSOI(SOIChange),
    ExitingSOI(SOIChange),
}

#[derive(Debug, Clone)]
pub struct EventPoint {
    pub time: f64,
    pub anomaly: f64,
    pub location: Point3<f64>,
}

impl EventPoint {
    pub fn compare_time(&self, other: &EventPoint) -> std::cmp::Ordering {
        self.time.partial_cmp(&other.time).unwrap()
    }
}

#[derive(Debug, Clone)]
pub struct Event {
    pub ship_id: ShipID,
    pub data: EventData,
    pub point: EventPoint,
}

pub fn first_event<B: Borrow<Event>>(it: impl Iterator<Item = B>) -> Option<B> {
    it.min_by(|a, b| a.borrow().point.compare_time(&b.borrow().point))
}
