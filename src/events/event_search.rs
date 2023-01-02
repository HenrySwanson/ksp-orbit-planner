use super::event::{first_event, Event, EventData, EventPoint, SOIChange};

use crate::astro::orbit::TimedOrbit;
use crate::math::root_finding::{bisection, Bracket};
use crate::orrery::{BodyID, Orrery, ShipID};

use nalgebra::Point3;
use std::collections::HashMap;

const NUM_ITERATIONS_SOI_ENCOUNTER: usize = 1000;

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

// TODO maybe these should be folded into UpcomingEvents? IDK
pub fn search_for_soi_escape(orrery: &Orrery, ship_id: ShipID) -> SearchResult {
    let ship_orbit = orrery.orbit_of_ship(ship_id);

    let current_body = ship_orbit.orbit().primary().id;
    let current_body_orbit = match orrery.orbit_of_body(current_body) {
        Some(o) => o,
        // We can never escape the Sun
        None => return SearchResult::Never,
    };
    let soi_radius = current_body_orbit.orbit().soi_radius();

    let parent_body = current_body_orbit.orbit().primary().id;

    let escape_s = match ship_orbit.orbit().get_s_at_radius(soi_radius) {
        Some(s) => s,
        None => return SearchResult::Never,
    };
    let escape_time = ship_orbit.time_at_s(escape_s);
    let new_state = ship_orbit.orbit().get_state_at_universal_anomaly(escape_s);

    let event = Event {
        ship_id,
        data: EventData::ExitingSOI(SOIChange {
            old: current_body,
            new: parent_body,
        }),
        point: EventPoint {
            time: escape_time,
            anomaly: escape_s,
            location: Point3::from(new_state.position()),
        },
    };

    SearchResult::Found(event)
}

pub fn search_for_soi_encounter(
    orrery: &Orrery,
    ship_id: ShipID,
    target_id: BodyID,
    start_time: f64,
    end_time: f64,
) -> SearchResult {
    assert!(
        start_time <= end_time,
        "Reversed window: {} > {}",
        start_time,
        end_time
    );
    let window = end_time - start_time;

    let ship_orbit = orrery.orbit_of_ship(ship_id);
    let parent_id = ship_orbit.orbit().primary().id;

    // Check whether this body and ship are co-orbiting
    let target_orbit = match orrery.orbit_of_body(target_id) {
        Some(o) => o,
        // Can't encounter the Sun
        None => return SearchResult::Never,
    };
    if target_orbit.orbit().primary().id != parent_id {
        return SearchResult::Never;
    }

    let soi_radius = target_orbit.orbit().soi_radius();

    // Quick check: if one orbit is much smaller than the other, then there's no chance of
    // intersection, so we can skip the rest of the search.
    fn pe_ap_check<P1, P2, S1, S2>(
        o1: &TimedOrbit<P1, S1>,
        o2: &TimedOrbit<P2, S2>,
        soi_radius: f64,
    ) -> bool {
        let o1_ap = o1.orbit().apoapsis();
        o1_ap.is_some() && o1_ap.unwrap() + soi_radius < o2.orbit().periapsis()
    }

    if pe_ap_check(&ship_orbit, &target_orbit, soi_radius)
        || pe_ap_check(&target_orbit, &ship_orbit, soi_radius)
    {
        return SearchResult::Never;
    }

    // How far should we search?
    // If the ship is in a closed orbit, search over one period. Otherwise,
    // search until we get too far away from the target body.
    // let ship_period = ship.state.get_orbit().period();
    // let window = ship_period.unwrap_or_else(|| {
    //     let target_apoapsis = target_state.get_orbit().apoapsis();
    //     // Second orbit must be closed
    //     assert!(target_apoapsis > 0.0);
    //     // Unwrap should succeed because this is an open orbit
    //     ship.state.get_t_until_radius(target_apoapsis).unwrap()
    // });
    // let window = f64::max(min_window, window);

    // let current_time = orrery.get_time();
    // let end_time = current_time + window;

    // Method for checking distance between bodies
    let check_distance = |delta_t| {
        let time = start_time + delta_t;
        let new_ship_pos = ship_orbit.state_at_time(time).position();
        let new_target_pos = target_orbit.state_at_time(time).position();
        (new_ship_pos - new_target_pos).norm()
    };

    // TODO have better algorithm than this!

    // We know their maximum relative velocity
    let ship_max_velocity = ship_orbit.orbit().periapsis_velocity();
    let target_max_velocity = target_orbit.orbit().periapsis_velocity();
    let max_rel_velocity = ship_max_velocity.abs() + target_max_velocity.abs();
    let current_distance = check_distance(0.0);

    // If we can't possibly catch up, then return no event.
    if current_distance - soi_radius > max_rel_velocity * window {
        return SearchResult::NotFound(end_time);
    }

    // Just step by step look for an intersection
    let num_slices = 1000;
    let slice_duration = window / (num_slices as f64);
    let mut encounter_time = None;
    for i in 0..num_slices {
        let t = i as f64 * slice_duration;
        if check_distance(t) < soi_radius {
            encounter_time = Some(t);
            break;
        }
    }

    // If we didn't get close enough, return no event
    let t2 = match encounter_time {
        Some(t) => t,
        None => return SearchResult::NotFound(end_time),
    };

    // Edge case: if t2 is zero, we may have just exited an SOI, and shouldn't
    // return an event, since we'd then get into a loop
    // TODO use direction to detect this instead!
    if t2 == 0.0 {
        return SearchResult::NotFound(end_time);
    }

    // Now we narrow in on the point using binary search.
    let t1 = t2 - slice_duration;
    let entry_time = bisection(
        |t| check_distance(t) - soi_radius,
        Bracket::new(t1, t2),
        NUM_ITERATIONS_SOI_ENCOUNTER,
    );

    // Lastly, figure out anomaly and position at that point
    let new_state = ship_orbit.state_at_time(start_time + entry_time);

    let event = Event {
        ship_id,
        data: EventData::EnteringSOI(SOIChange {
            old: parent_id,
            new: target_id,
        }),
        point: EventPoint {
            time: start_time + entry_time,
            anomaly: new_state.get_universal_anomaly(),
            location: Point3::from(new_state.position()),
        },
    };
    SearchResult::Found(event)
}
