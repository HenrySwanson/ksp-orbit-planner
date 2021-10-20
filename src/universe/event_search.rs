use std::collections::HashMap;

use super::body::BodyID;
use super::event::{first_event, Event, EventData};
use super::ship::ShipID;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum EventTag {
    EscapeSOI,
    EncounterSOI(BodyID),
}

impl EventTag {
    pub fn from_event(event: &Event) -> Self {
        match &event.data {
            EventData::EnteringSOI(soi_change) => EventTag::EncounterSOI(soi_change.new),
            EventData::ExitingSOI(_) => EventTag::EscapeSOI,
        }
    }
}

pub struct UpcomingEvents {
    ship_map: HashMap<ShipID, UpcomingEventsInner>,
}

impl UpcomingEvents {
    pub fn new() -> Self {
        UpcomingEvents {
            ship_map: HashMap::new(),
        }
    }

    pub fn get_next_event(&self, id: ShipID) -> Option<&Event> {
        self.ship_map
            .get(&id)
            .and_then(|inner| inner.get_next_event())
    }

    pub fn get_next_event_global(&self) -> Option<&Event> {
        first_event(self.ship_map.values().flat_map(|inner| inner.map.values()))
    }

    pub fn insert_event(&mut self, id: ShipID, event: Event) {
        let inner = self.ship_map.entry(id).or_insert(UpcomingEventsInner::new());
        inner.insert(event);
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

struct UpcomingEventsInner {
    map: HashMap<EventTag, Event>,
}

impl UpcomingEventsInner {
    fn new() -> Self {
        UpcomingEventsInner {
            map: HashMap::new()
        }
    }

    fn get_next_event(&self) -> Option<&Event> {
        first_event(self.map.values())
    }

    fn insert(&mut self, event: Event) {
        self.map.insert(EventTag::from_event(&event), event);
    }
}
