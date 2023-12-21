use std::collections::HashMap;

use crate::model::events::{first_event, Event, EventTag, SearchResult};
use crate::model::orrery::ShipID;

/// When we search for events, we often want to remember that we've already
/// searched a particular time window, and we want to cache that information
/// for later. This struct manages this cached information.
#[derive(Debug)]
pub struct EventSearchHorizons {
    horizons: HashMap<(ShipID, EventTag), SearchResult>,
    start_time: f64,
}

impl EventSearchHorizons {
    pub fn new(start_time: f64) -> Self {
        Self {
            horizons: HashMap::new(),
            start_time,
        }
    }

    pub fn get_next_event(&self) -> Option<&Event> {
        first_event(self.horizons.values().filter_map(SearchResult::event))
    }

    pub fn search_until(
        &mut self,
        ship_id: ShipID,
        tag: EventTag,
        end_time: f64,
        search_fn: impl FnOnce(f64, f64) -> SearchResult,
    ) {
        let key = (ship_id, tag);

        // If we've already found an event, or if we know no such event can occur,
        // then we bail out. Otherwise, get the start of our search window.
        let search_start = match self.horizons.get(&key) {
            None => self.start_time,
            Some(SearchResult::Found(_)) => return,
            Some(SearchResult::NotFound(ts)) => *ts,
            Some(SearchResult::Never) => return,
        };

        // Perform a search if we haven't already searched far enough
        if search_start <= end_time {
            let search_result = search_fn(search_start, end_time);
            self.horizons.insert(key, search_result);
        }
    }
}
