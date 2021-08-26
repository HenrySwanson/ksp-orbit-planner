use std::collections::HashMap;

use super::body::{Body, BodyID, BodyInfo, BodyState};
use super::event::{Event, ReverseEvent};
use super::orbit::OrbitPatch;
use super::orrery::{FramedState, Orrery};
use super::ship::{Ship, ShipID};

pub struct BodyRef<'u> {
    universe: &'u Universe,
    pub id: BodyID,
    body: &'u Body,
}

pub struct ShipRef<'u> {
    universe: &'u Universe,
    pub id: ShipID,
    ship: &'u Ship,
}

pub struct Universe {
    pub orrery: Orrery,
    upcoming_events: HashMap<ShipID, Event>,
    rev_event_stack: Vec<ReverseEvent>,
}

impl<'u> BodyRef<'u> {
    pub fn info(&self) -> &BodyInfo {
        &self.body.info
    }

    // note -- the FramedState can outlive the BodyRef! But it can't outlive the
    // universe (lifetime 'u)
    pub fn state(&self) -> FramedState<'u> {
        self.universe.orrery.get_body_state(self.id)
    }

    pub fn get_orbit(&self) -> Option<OrbitPatch> {
        let (state, parent_id) = match &self.body.state {
            BodyState::FixedAtOrigin => return None,
            BodyState::Orbiting { state, parent_id } => (state, *parent_id),
        };

        let orbit = state.get_orbit();
        let start_anomaly = state.get_universal_anomaly();

        let patch = OrbitPatch {
            orbit,
            start_anomaly,
            end_anomaly: None,
            parent_id,
        };
        Some(patch)
    }

    pub fn get_parent_id(&self) -> Option<BodyID> {
        match self.body.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { parent_id, .. } => Some(parent_id),
        }
    }
}

impl<'u> ShipRef<'u> {
    pub fn state(&self) -> FramedState<'u> {
        self.universe.orrery.get_ship_state(self.id)
    }

    pub fn get_orbit(&self) -> OrbitPatch {
        let orbit = self.ship.state.get_orbit();
        let start_anomaly = self.ship.state.get_universal_anomaly();
        let end_anomaly = self
            .universe
            .upcoming_events
            .get(&self.id)
            .as_ref()
            .map(|ev| ev.anomaly);

        OrbitPatch {
            orbit,
            start_anomaly,
            end_anomaly,
            parent_id: self.ship.parent_id,
        }
    }

    pub fn get_parent_id(&self) -> BodyID {
        self.ship.parent_id
    }
}

impl<'u> Universe {
    pub fn new(start_time: f64) -> Self {
        Universe {
            orrery: Orrery::new(start_time),
            upcoming_events: HashMap::new(),
            rev_event_stack: vec![],
        }
    }

    pub fn body_ids(&self) -> impl Iterator<Item = &BodyID> {
        self.orrery.body_ids()
    }

    pub fn get_body(&self, id: BodyID) -> BodyRef {
        BodyRef {
            universe: self,
            id,
            body: &self.orrery.get_body(id),
        }
    }

    pub fn bodies(&'u self) -> impl Iterator<Item = BodyRef<'u>> {
        self.orrery.bodies().map(move |(id, body)| BodyRef {
            universe: self,
            id: *id,
            body,
        })
    }

    pub fn ship_ids(&self) -> impl Iterator<Item = &ShipID> {
        self.orrery.ship_ids()
    }

    pub fn get_ship(&self, id: ShipID) -> ShipRef {
        ShipRef {
            universe: self,
            id,
            ship: &self.orrery.get_ship(id),
        }
    }

    pub fn ships(&'u self) -> impl Iterator<Item = ShipRef<'u>> {
        self.orrery.ships().map(move |(id, ship)| ShipRef {
            universe: self,
            id: *id,
            ship,
        })
    }

    pub fn update_time(&mut self, delta_t: f64) {
        if delta_t > 0.0 {
            self.advance_t(delta_t);
        } else {
            self.rewind_t(-delta_t);
        }
    }

    fn advance_t(&mut self, delta_t: f64) {
        assert!(delta_t > 0.0);

        // Figure out the next events
        self.set_upcoming_events();

        // See if there's an event we need to process
        let end_time = self.orrery.get_time() + delta_t;
        match self.get_next_event() {
            Some((ship_id, event)) if event.time < end_time => {
                let event_time = event.time;

                self.orrery.update_time(event_time - self.orrery.get_time());
                // Apply the event, clear it from the ship, and push it onto the event stack
                let reverse_event = self.orrery.process_event(ship_id, event);
                self.rev_event_stack.push(reverse_event);
                self.upcoming_events.remove(&ship_id);

                self.advance_t(end_time - event_time);
            }
            _ => self.orrery.update_time(delta_t),
        }
    }

    fn rewind_t(&mut self, delta_t: f64) {
        assert!(delta_t >= 0.0);

        // Don't rewind past zero
        let delta_t = nalgebra::clamp(delta_t, 0.0, self.orrery.get_time());
        let new_time = self.orrery.get_time() - delta_t;

        // Find the most recent event we processed, and if we need to, revert it
        match self.rev_event_stack.last() {
            Some(reverse_event) if reverse_event.event.time >= new_time => {
                let reverse_event = reverse_event.clone();
                let event_time = reverse_event.event.time;

                self.orrery.update_time(event_time - self.orrery.get_time());
                self.orrery.revert_event(&reverse_event);
                self.rev_event_stack.pop();
                self.upcoming_events
                    .insert(reverse_event.ship_id, reverse_event.event);
                self.rewind_t(event_time - new_time);
            }
            _ => self.orrery.update_time(-delta_t),
        }
    }

    fn set_upcoming_events(&mut self) {
        for id in self.orrery.ship_ids().copied() {
            if self.upcoming_events.contains_key(&id) {
                continue;
            }

            if let Some(event) = self.orrery.compute_next_event(id) {
                self.upcoming_events.insert(id, event);
            }
        }
    }

    fn get_next_event(&self) -> Option<(ShipID, Event)> {
        // Note: can't easily use Iterator::min_by_key because f64 doesn't implement Ord.
        let event = self
            .upcoming_events
            .iter()
            .min_by(|a, b| a.1.compare_time(&b.1));

        event.map(|(id, event)| (*id, event.clone()))
    }
}
