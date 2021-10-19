use std::collections::HashMap;

use super::event::Event;
use super::ship::ShipID;

pub struct UpcomingEvents {
    ship_map: HashMap<ShipID, Event>,
}

impl UpcomingEvents {
    pub fn new() -> Self {
        UpcomingEvents {
            ship_map: HashMap::new(),
        }
    }

    pub fn get_next_event(&self, id: ShipID) -> Option<&Event> {
        self.ship_map.get(&id)
    }

    pub fn get_next_event_global(&self) -> Option<&Event> {
        first_event(self.ship_map.values())
    }

    pub fn insert_event(&mut self, id: ShipID, event: Event) {
        self.ship_map.insert(id, event);
    }

    pub fn clear_events(&mut self, id: ShipID) {
        self.ship_map.remove(&id);
    }
}

fn first_event<'a>(it: impl Iterator<Item = &'a Event>) -> Option<&'a Event> {
    it.min_by(|a, b| a.point.compare_time(&b.point))
}
