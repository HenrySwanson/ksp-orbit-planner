use std::collections::HashMap;

use crate::orrery::ShipID;

use super::event::{first_event, Event, EventTag};

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

#[derive(Debug, Default)]
pub struct UpcomingEvents {
    ship_map: HashMap<ShipID, UpcomingEventsInner>,
}

impl UpcomingEvents {
    pub fn new() -> Self {
        Default::default()
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
            .or_insert_with(UpcomingEventsInner::new);
        inner.insert(event);
    }

    pub fn update(
        &mut self,
        id: ShipID,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce(Option<f64>) -> SearchResult,
    ) {
        let inner = self
            .ship_map
            .entry(id)
            .or_insert_with(UpcomingEventsInner::new);
        inner.update(tag, end_time, search_fn);
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

#[derive(Debug, Default)]
struct UpcomingEventsInner {
    map: HashMap<EventTag, SearchResult>,
}

impl UpcomingEventsInner {
    fn new() -> Self {
        Default::default()
    }

    fn get_next_event(&self) -> Option<&Event> {
        first_event(self.map.values().filter_map(|search| search.event()))
    }

    fn insert(&mut self, event: Event) {
        self.map
            .insert(EventTag::from_event(&event), SearchResult::Found(event));
    }

    fn update(
        &mut self,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce(Option<f64>) -> SearchResult,
    ) {
        let searched_until = match self.map.get(&tag) {
            None => None,
            Some(SearchResult::Found(_)) => return,
            Some(SearchResult::NotFound(ts)) => Some(*ts),
            Some(SearchResult::Never) => return,
        };

        // Perform a search if we haven't already searched far enough
        if searched_until.map_or(true, |ts| ts <= end_time) {
            self.map.insert(tag, search_fn(searched_until));
        }
    }
}
