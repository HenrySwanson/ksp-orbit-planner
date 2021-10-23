use std::collections::HashMap;

use super::event::{first_event, Event, EventData};
use crate::orrery::BodyID;
use crate::orrery::ShipID;

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

pub enum EventSearch {
    Found(Event),
    NotFound(f64),
    Never,
}

impl EventSearch {
    pub fn event(&self) -> Option<&Event> {
        match self {
            EventSearch::Found(event) => Some(event),
            EventSearch::NotFound(_) => None,
            EventSearch::Never => None,
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
        first_event(
            self.ship_map
                .values()
                .flat_map(|inner| inner.get_next_event()),
        )
    }

    pub fn insert_event(&mut self, id: ShipID, event: Event) {
        let inner = self
            .ship_map
            .entry(id)
            .or_insert(UpcomingEventsInner::new());
        inner.insert(event);
    }

    pub fn update(
        &mut self,
        id: ShipID,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce() -> EventSearch,
    ) {
        let inner = self
            .ship_map
            .entry(id)
            .or_insert(UpcomingEventsInner::new());
        inner.update(tag, end_time, search_fn);
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

struct UpcomingEventsInner {
    map: HashMap<EventTag, EventSearch>,
}

impl UpcomingEventsInner {
    fn new() -> Self {
        UpcomingEventsInner {
            map: HashMap::new(),
        }
    }

    fn get_next_event(&self) -> Option<&Event> {
        first_event(self.map.values().filter_map(|search| search.event()))
    }

    fn insert(&mut self, event: Event) {
        self.map
            .insert(EventTag::from_event(&event), EventSearch::Found(event));
    }

    fn update(&mut self, tag: EventTag, end_time: f64, search_fn: impl FnOnce() -> EventSearch) {
        let searched_until = match self.map.get(&tag) {
            None => None,
            Some(EventSearch::Found(_)) => return,
            Some(EventSearch::NotFound(ts)) => Some(*ts),
            Some(EventSearch::Never) => return,
        };

        // Perform a search unless we've already searched this far
        if !searched_until.map_or(false, |ts| ts > end_time) {
            self.map.insert(tag, search_fn());
        }
    }
}
