use std::collections::HashMap;

use crate::model::events::{first_event, Event, EventTag, SearchResult};
use crate::model::orrery::ShipID;

#[derive(Debug, Default)]
pub struct UpcomingEvents {
    ship_map: HashMap<ShipID, UpcomingEventsInner>,
    start_time: f64,
}

impl UpcomingEvents {
    pub fn new(start_time: f64) -> Self {
        Self {
            ship_map: HashMap::new(),
            start_time,
        }
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
        search_fn: impl FnOnce(f64, f64) -> SearchResult,
    ) {
        self.get_inner_mut(id).update(tag, end_time, search_fn);
    }

    fn get_inner_mut(&mut self, id: ShipID) -> &mut UpcomingEventsInner {
        let start_time = self.start_time;
        self.ship_map
            .entry(id)
            .or_insert_with(|| UpcomingEventsInner::new(start_time))
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

// TODO: re-think how this works. we only need to store one event in the end!
// still though, multiple search horizons. maybe there's a different way to do this
#[derive(Debug)]
struct UpcomingEventsInner {
    map: HashMap<EventTag, SearchResult>,
    start_time: f64,
}

impl UpcomingEventsInner {
    fn new(start_time: f64) -> Self {
        Self {
            map: HashMap::new(),
            start_time,
        }
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
        search_fn: impl FnOnce(f64, f64) -> SearchResult,
    ) {
        // If we've already found an event, or if we know no such event can occur,
        // then we bail out. Otherwise, get the start of our search window.
        let search_start = match self.map.get(&tag) {
            None => self.start_time,
            Some(SearchResult::Found(_)) => return,
            Some(SearchResult::NotFound(ts)) => *ts,
            Some(SearchResult::Never) => return,
        };

        // Perform a search if we haven't already searched far enough
        if search_start <= end_time {
            let search_result = search_fn(search_start, end_time);
            self.map.insert(tag, search_result);
        }
    }
}
