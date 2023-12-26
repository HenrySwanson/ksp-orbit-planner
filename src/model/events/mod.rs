use std::borrow::Borrow;

use nalgebra::Point3;

use crate::model::orrery::{BodyID, ShipID};

mod intervals;
mod soi_change;

pub use soi_change::{search_for_soi_encounter, search_for_soi_escape};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SOIChange {
    pub old: BodyID,
    pub new: BodyID,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EventData {
    EnteringSOI(SOIChange),
    ExitingSOI(SOIChange),
}

/// Used for tracking the type of event within [UpcomingEvents]. Events with
/// different tags will have their search horizons tracked separately.
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

#[derive(Debug)]
pub enum SearchResult {
    Found(Event),
    NotFound(f64),
    Never,
}

impl SearchResult {
    pub fn event(&self) -> Option<&Event> {
        match self {
            SearchResult::Found(event) => Some(event),
            SearchResult::NotFound(_) => None,
            SearchResult::Never => None,
        }
    }
}
