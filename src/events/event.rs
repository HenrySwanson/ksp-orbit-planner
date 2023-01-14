use std::borrow::Borrow;

use nalgebra::Point3;

use crate::orrery::BodyID;
use crate::orrery::ShipID;

#[derive(Debug, Clone, PartialEq)]
pub struct SOIChange {
    pub old: BodyID,
    pub new: BodyID,
}

#[derive(Debug, Clone, PartialEq)]
pub enum EventData {
    EnteringSOI(SOIChange),
    ExitingSOI(SOIChange),
}

/// Used for tracking the type of event within [UpcomingEvents]. Events with different
/// tags will have their search horizons tracked separately.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum EventTag {
    EscapeSOI,
    EncounterSOI(BodyID),
}

impl EventData {
    pub fn tag(&self) -> EventTag {
        match &self {
            EventData::EnteringSOI(soi_change) => EventTag::EncounterSOI(soi_change.new),
            EventData::ExitingSOI(_) => EventTag::EscapeSOI,
        }
    }
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
