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
        self.ship_map.get(&id)?.get_next_event()
    }

    pub fn get_next_event_global(&self) -> Option<&Event> {
        first_event(
            self.ship_map
                .values()
                .flat_map(UpcomingEventsInner::get_next_event),
        )
    }

    pub fn insert_event(&mut self, id: ShipID, event: Event) {
        self.get_inner_mut(id).insert(event);
    }

    pub fn update(
        &mut self,
        id: ShipID,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce(Option<f64>) -> SearchResult,
    ) {
        self.get_inner_mut(id).update(tag, end_time, search_fn);
    }

    fn get_inner_mut(&mut self, id: ShipID) -> &mut UpcomingEventsInner {
        self.ship_map
            .entry(id)
            .or_insert_with(UpcomingEventsInner::new)
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

// TODO: re-think how this works. we only need to store one event in the end!
// still though, multiple search horizons. maybe there's a different way to do this
#[derive(Debug, Default)]
struct UpcomingEventsInner {
    map: HashMap<EventTag, SearchResult>,
}

impl UpcomingEventsInner {
    fn new() -> Self {
        Default::default()
    }

    fn get_next_event(&self) -> Option<&Event> {
        first_event(self.map.values().filter_map(SearchResult::event))
    }

    fn insert(&mut self, event: Event) {
        self.map
            .insert(event.data.tag(), SearchResult::Found(event));
    }

    fn update(
        &mut self,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce(Option<f64>) -> SearchResult,
    ) {
        // If we've already found an event, or if we know no such event can occur,
        // then we bail out. Otherwise, get the start of our search window.
        let searched_until = match self.map.get(&tag) {
            None => None,
            Some(SearchResult::Found(_)) => return,
            Some(SearchResult::NotFound(ts)) => Some(*ts),
            Some(SearchResult::Never) => return,
        };

        // Perform a search if we haven't already searched far enough
        if searched_until.map_or(true, |ts| ts <= end_time) {
            let search_result = search_fn(searched_until);
            self.map.insert(tag, search_result);
        }
    }
}
