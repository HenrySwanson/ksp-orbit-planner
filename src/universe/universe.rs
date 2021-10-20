use std::collections::HashMap;

use std::f64::consts::PI;

use super::body::{Body, BodyID, BodyInfo, BodyState};
use super::event::Event;
use super::event_search::UpcomingEvents;
use super::orbit::OrbitPatch;
use super::orrery::{FramedState, Orrery};
use super::ship::{Ship, ShipID};

pub struct BodyRef<'u> {
    universe: &'u Universe,
    body: &'u Body,
}

pub struct ShipRef<'u> {
    universe: &'u Universe,
    ship: &'u Ship,
}

pub struct Universe {
    pub orrery: Orrery,
    upcoming_events: UpcomingEvents,
    prev_event_stack: Vec<Event>,
}

impl<'u> BodyRef<'u> {
    pub fn id(&self) -> BodyID {
        self.body.id
    }

    pub fn info(&self) -> &BodyInfo {
        &self.body.info
    }

    // note -- the FramedState can outlive the BodyRef! But it can't outlive the
    // universe (lifetime 'u)
    pub fn state(&self) -> FramedState<'u> {
        self.universe.orrery.get_body_state(self.body.id)
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
        self.body.parent_id()
    }
}

impl<'u> ShipRef<'u> {
    pub fn id(&self) -> ShipID {
        self.ship.id
    }

    pub fn state(&self) -> FramedState<'u> {
        self.universe.orrery.get_ship_state(self.ship.id)
    }

    pub fn get_orbit(&self) -> OrbitPatch {
        let orbit = self.ship.state.get_orbit();
        let next_event = self.universe.upcoming_events.get_next_event(self.ship.id);
        let start_anomaly = self.ship.state.get_universal_anomaly();
        let end_anomaly = match next_event {
            Some(event) => {
                let mut end_s = event.point.anomaly;
                if end_s < start_anomaly {
                    let beta = -2.0 * orbit.energy();
                    assert!(beta > 0.0);
                    // Since this is an ellipse, the eccentric anomaly makes sense.
                    // We want E to increase by 2pi, and s = E / sqrt(beta)
                    end_s += 2.0 * PI / beta.sqrt()
                }
                Some(end_s)
            }
            None => None,
        };

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
            upcoming_events: UpcomingEvents::new(),
            prev_event_stack: vec![],
        }
    }

    pub fn get_body(&self, id: BodyID) -> BodyRef {
        BodyRef {
            universe: self,
            body: &self.orrery.get_body(id),
        }
    }

    pub fn bodies(&'u self) -> impl Iterator<Item = BodyRef<'u>> {
        self.orrery.bodies().map(move |body| BodyRef {
            universe: self,
            body,
        })
    }

    pub fn get_ship(&self, id: ShipID) -> ShipRef {
        ShipRef {
            universe: self,
            ship: &self.orrery.get_ship(id),
        }
    }

    pub fn ships(&'u self) -> impl Iterator<Item = ShipRef<'u>> {
        self.orrery.ships().map(move |ship| ShipRef {
            universe: self,
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

        // Figure out any upcoming events
        self.search_for_events();

        // See if there's an event we need to process
        let end_time = self.orrery.get_time() + delta_t;
        match self.upcoming_events.get_next_event_global() {
            Some(event) if event.point.time < end_time => {
                let ship_id = event.ship_id;
                let event_time = event.point.time;

                // Fast-forward time up until the event
                self.orrery.update_time(event_time - self.orrery.get_time());

                // Apply the event, clear it from "upcoming", and push it onto
                // the completed event stack.
                self.orrery.process_event(&event);
                self.prev_event_stack.push(event.clone());
                self.upcoming_events.clear_events(ship_id);

                // Advance for the remaining amount of time (recursive!)
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

        // Check the most recent event, and see if we need to revert it
        match self.prev_event_stack.last() {
            Some(event) if event.point.time >= new_time => {
                let event_time = event.point.time;

                // Rewind up until the event
                self.orrery.update_time(event_time - self.orrery.get_time());

                // Pop the event off of the stack and put it back into upcoming
                self.orrery.revert_event(&event);
                self.upcoming_events
                    .insert_event(event.ship_id, event.clone());
                self.prev_event_stack.pop();

                // Rewind the remaining amount of time (recursive!)
                self.rewind_t(event_time - new_time);
            }
            _ => self.orrery.update_time(-delta_t),
        }
    }

    fn search_for_events(&mut self) {
        for id in self.orrery.ships().map(|s| s.id) {
            if self.upcoming_events.get_next_event(id).is_some() {
                continue;
            }

            // Check for an SOI escape event
            if let Some(event) = self.orrery.search_for_soi_escape(id) {
                self.upcoming_events.insert_event(id, event);
            }

            // Check for SOI encounter events
            for body in self.orrery.bodies() {
                if let Some(event) = self.orrery.search_for_soi_encounter(id, body.id) {
                    self.upcoming_events.insert_event(id, event);
                }
            }
        }
    }
}
