use super::event::{first_event, Event, EventData, EventPoint, SOIChange};

use crate::math::root_finding::{bisection, Bracket};
use crate::orrery::{BodyID, Orbit, Orrery, ShipID};

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

#[derive(Debug)]
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

#[derive(Debug)]
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

// TODO maybe these should be folded into UpcomingEvents? IDK
pub fn search_for_soi_escape(orrery: &Orrery, ship_id: ShipID) -> EventSearch {
    let ship = orrery.get_ship(ship_id);
    let current_body = ship.parent_id;
    let parent_body = match orrery.get_body(current_body).parent_id() {
        Some(x) => x,
        None => return EventSearch::Never,
    };

    let soi_radius = orrery
        .get_soi_radius(current_body)
        .expect("Body with parent should have SOI");

    let delta_s = match ship.state.get_s_until_radius(soi_radius) {
        Some(s) => s,
        None => return EventSearch::Never,
    };

    let delta_t = ship.state.delta_s_to_t(delta_s);
    let new_state = ship.state.update_s(delta_s);

    let event = Event {
        ship_id,
        data: EventData::ExitingSOI(SOIChange {
            old: current_body,
            new: parent_body,
        }),
        point: EventPoint {
            time: orrery.get_time() + delta_t,
            anomaly: ship.state.get_universal_anomaly() + delta_s,
            location: Point3::from(new_state.position()),
        },
    };

    EventSearch::Found(event)
}

pub fn search_for_soi_encounter(
    orrery: &Orrery,
    ship_id: ShipID,
    target_id: BodyID,
    min_window: f64,
) -> EventSearch {
    let ship = orrery.get_ship(ship_id);
    let parent_id = ship.parent_id;

    // Check whether this body and ship are co-orbiting
    let target_body = orrery.get_body(target_id);
    match target_body.parent_id() {
        Some(x) if x == parent_id => {}
        _ => return EventSearch::Never,
    }

    let soi_radius = orrery.get_soi_radius(target_id).unwrap();
    let target_state = target_body.state().expect("Cannot SOI encounter the sun");

    // Get their orbits
    let self_orbit = ship.state.get_orbit();
    let planet_orbit = target_state.get_orbit();

    // Quick check: if one orbit is much smaller than the other, then we can skip the rest
    let pe_ap_check =
        |o1: &Orbit, o2: &Orbit| o1.apoapsis() > 0.0 && o1.apoapsis() + soi_radius < o2.periapsis();
    if pe_ap_check(&self_orbit, &planet_orbit) || pe_ap_check(&planet_orbit, &self_orbit) {
        return EventSearch::Never;
    }

    // How far should we search?
    // If the ship is in a closed orbit, search over one period. Otherwise,
    // search until we get too far away from the target body.
    let ship_period = ship.state.get_orbit().period();
    let window = ship_period.unwrap_or_else(|| {
        let target_apoapsis = target_state.get_orbit().apoapsis();
        // Second orbit must be closed
        assert!(target_apoapsis > 0.0);
        // Unwrap should succeed because this is an open orbit
        ship.state.get_t_until_radius(target_apoapsis).unwrap()
    });
    let window = f64::max(min_window, window);

    let current_time = orrery.get_time();
    let end_time = current_time + window;

    // Method for checking distance between bodies
    let check_distance = |time| {
        let new_self_pos = ship.state.update_t(time).position();
        let new_target_pos = target_state.update_t(time).position();
        (new_self_pos - new_target_pos).norm()
    };

    // TODO have better algorithm than this!

    // We know their maximum relative velocity
    let mu = ship.state.mu();
    let self_max_velocity = mu * (1.0 + self_orbit.eccentricity()) / self_orbit.angular_momentum();
    let planet_max_velocity =
        mu * (1.0 + planet_orbit.eccentricity()) / planet_orbit.angular_momentum();
    let max_rel_velocity = self_max_velocity.abs() + planet_max_velocity.abs();
    let current_distance = check_distance(0.0);

    // If we can't possibly catch up, then return no event.
    if current_distance - soi_radius > max_rel_velocity * window {
        return EventSearch::NotFound(end_time);
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
        None => return EventSearch::NotFound(end_time),
    };

    // Edge case: if t2 is zero, we may have just exited an SOI, and shouldn't
    // return an event, since we'd then get into a loop
    // TODO use direction to detect this instead!
    if t2 == 0.0 {
        return EventSearch::NotFound(end_time);
    }

    // Now we narrow in on the point using binary search.
    let t1 = t2 - slice_duration;
    let entry_time = bisection(
        |t| check_distance(t) - soi_radius,
        Bracket::new(t1, t2),
        NUM_ITERATIONS_SOI_ENCOUNTER,
    );

    // Lastly, figure out anomaly and position at that point
    // TODO: new s can end up less than old s if we're not careful :\
    let new_state = ship.state.update_t(entry_time);

    let event = Event {
        ship_id,
        data: EventData::EnteringSOI(SOIChange {
            old: parent_id,
            new: target_id,
        }),
        point: EventPoint {
            time: current_time + entry_time,
            anomaly: new_state.get_universal_anomaly(),
            location: Point3::from(new_state.position()),
        },
    };
    EventSearch::Found(event)
}
