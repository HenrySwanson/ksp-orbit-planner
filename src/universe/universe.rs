use nalgebra::{Point3, UnitQuaternion, Vector3};

use std::collections::HashMap;

use super::body::{Body, BodyInfo, BodyState};
use super::event::{Event, EventKind, ReverseEvent};
use super::frame::FrameTransform;
use super::orbit::Orbit;
use super::ship::Ship;
use super::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Root,
    BodyInertial(BodyID),
    ShipInertial(ShipID),
    ShipOrbital(ShipID),
}

pub struct FramedState<'u> {
    universe: &'u Universe,
    position: Point3<f64>,
    velocity: Vector3<f64>,
    native_frame: Frame,
}

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

#[derive(Debug, Clone)]
pub struct OrbitPatch {
    pub orbit: Orbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub parent_id: BodyID,
}

pub struct Universe {
    bodies: HashMap<BodyID, Body>,
    next_body_id: usize,
    ships: HashMap<ShipID, Ship>,
    next_ship_id: usize,
    time: f64,
    event_stack: Vec<ReverseEvent>,
}

impl FramedState<'_> {
    pub fn get_position(&self, frame: Frame) -> Point3<f64> {
        self.universe
            .convert_frames(self.native_frame, frame)
            .convert_point(&self.position)
    }

    pub fn get_velocity(&self, frame: Frame) -> Vector3<f64> {
        self.universe
            .convert_frames(self.native_frame, frame)
            .convert_velocity(&self.position, &self.velocity)
    }
}

impl<'u> BodyRef<'u> {
    pub fn info(&self) -> &BodyInfo {
        &self.body.info
    }

    // note -- the FramedState can outlive the BodyRef! But it can't outlive the
    // universe (lifetime 'u)
    pub fn state(&self) -> FramedState<'u> {
        let (p, v, frame) = match &self.body.state {
            BodyState::FixedAtOrigin => (Vector3::zeros(), Vector3::zeros(), Frame::Root),
            BodyState::Orbiting { state, parent_id } => (
                state.get_position(),
                state.get_velocity(),
                Frame::BodyInertial(*parent_id),
            ),
        };

        FramedState {
            universe: self.universe,
            position: Point3::from(p),
            velocity: v,
            native_frame: frame,
        }
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
        let state = &self.ship.state;

        FramedState {
            universe: self.universe,
            position: Point3::from(state.get_position()),
            velocity: state.get_velocity(),
            native_frame: Frame::BodyInertial(self.ship.parent_id),
        }
    }

    pub fn get_orbit(&self) -> OrbitPatch {
        let orbit = self.ship.state.get_orbit();
        let start_anomaly = self.ship.state.get_universal_anomaly();
        let end_anomaly = self.ship.next_event.as_ref().map(|ev| ev.anomaly);

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
            bodies: HashMap::new(),
            next_body_id: 0,
            ships: HashMap::new(),
            next_ship_id: 0,
            time: start_time,
            event_stack: vec![],
        }
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn body_ids(&self) -> impl Iterator<Item = &BodyID> {
        self.bodies.keys()
    }

    pub fn bodies(&'u self) -> impl Iterator<Item = BodyRef<'u>> {
        self.bodies.iter().map(move |(id, body)| BodyRef {
            universe: self,
            id: *id,
            body,
        })
    }

    pub fn get_body(&self, id: BodyID) -> BodyRef {
        BodyRef {
            universe: self,
            id,
            body: &self.bodies[&id],
        }
    }

    pub fn add_body(
        &mut self,
        body_info: BodyInfo,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        parent_id: BodyID,
    ) -> BodyID {
        let parent_mu = self.bodies[&parent_id].info.mu;
        let state = BodyState::Orbiting {
            parent_id,
            state: CartesianState::new(position, velocity, parent_mu),
        };
        self.insert_new_body(body_info, state)
    }

    pub fn add_fixed_body(&mut self, body_info: BodyInfo) -> BodyID {
        self.insert_new_body(body_info, BodyState::FixedAtOrigin)
    }

    fn insert_new_body(&mut self, info: BodyInfo, state: BodyState) -> BodyID {
        let body = Body { info, state };

        let new_id = BodyID(self.next_body_id);
        self.next_body_id += 1;

        self.bodies.insert(new_id, body);
        new_id
    }

    pub fn ship_ids(&self) -> impl Iterator<Item = &ShipID> {
        self.ships.keys()
    }

    pub fn ships(&'u self) -> impl Iterator<Item = ShipRef<'u>> {
        self.ships.iter().map(move |(id, ship)| ShipRef {
            universe: self,
            id: *id,
            ship,
        })
    }

    pub fn get_ship(&self, id: ShipID) -> ShipRef {
        ShipRef {
            universe: self,
            id,
            ship: &self.ships[&id],
        }
    }

    pub fn add_ship(
        &mut self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        parent_id: BodyID,
    ) -> ShipID {
        let parent_mu = self.bodies[&parent_id].info.mu;
        let ship = Ship {
            state: CartesianState::new(position, velocity, parent_mu),
            parent_id,
            next_event: None,
        };

        let new_id = ShipID(self.next_ship_id);
        self.next_ship_id += 1;

        self.ships.insert(new_id, ship);
        new_id
    }

    pub fn convert_frames(&self, src: Frame, dst: Frame) -> FrameTransform<f64> {
        // TODO : do this in a more clever way
        let src_to_root = self.convert_from_root(src).inverse();
        let root_to_dst = self.convert_from_root(dst);
        src_to_root.append_transformation(&root_to_dst)
    }

    fn convert_from_root(&self, frame: Frame) -> FrameTransform<f64> {
        match frame {
            Frame::Root => FrameTransform::identity(),
            Frame::BodyInertial(k) => {
                match &self.bodies[&k].state {
                    BodyState::FixedAtOrigin => {
                        // This is equivalent to the root frame; return the identity
                        FrameTransform::identity()
                    }
                    BodyState::Orbiting { parent_id, state } => {
                        // Get the parent and compute the transform from its reference frame to root
                        let parent_frame = Frame::BodyInertial(*parent_id);
                        let root_to_parent = self.convert_from_root(parent_frame);

                        // Get the transform from our frame to the parent's
                        let parent_to_self = FrameTransform::from_active(
                            UnitQuaternion::identity(),
                            state.get_position(),
                            state.get_velocity(),
                            Vector3::zeros(),
                        );
                        root_to_parent.append_transformation(&parent_to_self)
                    }
                }
            }
            Frame::ShipInertial(k) => {
                let ship = &self.ships[&k];
                let parent_frame = Frame::BodyInertial(ship.parent_id);
                let root_to_parent = self.convert_from_root(parent_frame);

                let parent_to_self = FrameTransform::from_active(
                    UnitQuaternion::identity(),
                    ship.state.get_position(),
                    ship.state.get_velocity(),
                    Vector3::zeros(),
                );
                root_to_parent.append_transformation(&parent_to_self)
            }
            Frame::ShipOrbital(k) => {
                let ship = &self.ships[&k];
                let root_to_parent = self.convert_from_root(Frame::ShipInertial(k));
                // TODO oops! I've been using quaternion-based (Isometry3) and matrix-based (Rotation3)
                // things in the same code. let's pick one and unify
                let orbit = ship.state.get_orbit();
                let orientation = crate::math::geometry::always_find_rotation(
                    &orbit.normal_vector(),
                    &ship.state.get_velocity(),
                    1e-20,
                );
                let parent_to_self = FrameTransform::from_active(
                    UnitQuaternion::from_rotation_matrix(&orientation),
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(), // TODO: this is wrong but it doesn't matter right now
                );
                root_to_parent.append_transformation(&parent_to_self)
            }
        }
    }

    pub fn get_soi_radius(&self, id: BodyID) -> Option<f64> {
        // it's given by a (m/M)^(2/5)
        let body = &self.bodies[&id];

        match &body.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { parent_id, state } => {
                let parent_body = &self.bodies[parent_id];

                let mu_1 = parent_body.info.mu;
                let mu_2 = body.info.mu;

                let sma = state.get_orbit().semimajor_axis();
                assert!(
                    sma > 0.0,
                    "SOI radius approximation only works with elliptical orbits"
                );

                Some(sma * (mu_2 / mu_1).powf(0.4))
            }
        }
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

        // TODO wow i really gotta figure out how to not make a vector
        // every time :\ borrow checker hard lol
        // Maybe separate body collection and ship collection into separate things?
        // After all, the body calcs will never depend on the ships, ever.
        let ship_ids: Vec<_> = self.ship_ids().copied().collect();
        for id in ship_ids.into_iter() {
            if self.ships.get_mut(&id).unwrap().next_event.is_some() {
                continue;
            }
            let ev = self.determine_next_event(id);
            self.ships.get_mut(&id).unwrap().next_event = ev;
        }

        // See if there's an event we need to process
        let end_time = self.time + delta_t;
        match self.get_next_event() {
            Some((ship_id, event)) if event.time < end_time => {
                let event_time = event.time;
                self.update_t_no_events(event_time - self.time);
                self.process_event(ship_id, event);
                self.advance_t(end_time - event_time);
            }
            _ => self.update_t_no_events(delta_t),
        }
    }

    fn rewind_t(&mut self, delta_t: f64) {
        assert!(delta_t >= 0.0);

        // Don't rewind past zero
        let delta_t = nalgebra::clamp(delta_t, 0.0, self.time);
        let new_time = self.time - delta_t;

        // Find the most recent event we processed, and if we need to, revert it
        match self.event_stack.last() {
            Some(reverse_event) if reverse_event.event.time >= new_time => {
                let reverse_event = reverse_event.clone();
                let event_time = reverse_event.event.time;

                self.update_t_no_events(event_time - self.time);
                self.revert_event(reverse_event);
                self.event_stack.pop();
                self.rewind_t(event_time - new_time);
            }
            _ => self.update_t_no_events(-delta_t),
        }
    }

    fn update_t_no_events(&mut self, delta_t: f64) {
        // here, delta_t can be positive, negative, or zero

        for body in self.bodies.values_mut() {
            match &mut body.state {
                BodyState::FixedAtOrigin => {}
                BodyState::Orbiting { state, .. } => state.update_t(delta_t),
            }
        }

        for ship in self.ships.values_mut() {
            ship.state.update_t(delta_t);
        }

        self.time += delta_t;
    }

    fn determine_next_event(&self, ship_id: ShipID) -> Option<Event> {
        let ship = &self.ships[&ship_id];
        let body_id = ship.parent_id;
        let soi_escape = match self.get_soi_radius(body_id) {
            Some(soi_radius) => ship.state.find_soi_escape_event(soi_radius, self.time),
            None => None,
        };

        // TODO SOI encounter
        soi_escape
    }

    fn get_next_event(&self) -> Option<(ShipID, Event)> {
        // Note: can't easily use Iterator::min_by_key because f64 doesn't implement Ord.
        let mut next_event: Option<(ShipID, Event)> = None;
        for (id, ship) in self.ships.iter() {
            let event = match &ship.next_event {
                Some(e) => e,
                None => continue,
            };

            match next_event {
                Some((_, ref best_event)) if best_event.time < event.time => {} // keep it
                _ => next_event = Some((*id, event.clone())),                   // replace it
            }
        }
        next_event
    }

    fn process_event(&mut self, ship_id: ShipID, event: Event) {
        // Dispatch to the appropriate handler
        let reverse_event = match event.kind {
            EventKind::EnteringSOI(body_id) => {
                let prev_body_id = self.ships[&ship_id].parent_id;
                self.change_soi(ship_id, body_id);
                ReverseEvent {
                    event,
                    ship_id,
                    previous_soi_body: Some(prev_body_id),
                }
            }
            EventKind::ExitingSOI => {
                let current_body = self.ships[&ship_id].parent_id;
                let parent_id = match self.bodies[&current_body].state {
                    BodyState::FixedAtOrigin => panic!("Cannot leave SOI of root body"),
                    BodyState::Orbiting { parent_id, .. } => parent_id,
                };
                self.change_soi(ship_id, parent_id);
                ReverseEvent {
                    event,
                    ship_id,
                    previous_soi_body: Some(current_body),
                }
            }
        };

        // Clear the event from the ship, and put it on the event stack
        self.ships.get_mut(&ship_id).unwrap().next_event = None;
        self.event_stack.push(reverse_event);
    }

    fn revert_event(&mut self, reverse_event: ReverseEvent) {
        // Dispatch to the appropriate handler
        let ship_id = reverse_event.ship_id;
        match reverse_event.event.kind {
            EventKind::EnteringSOI(_) => {
                self.change_soi(
                    ship_id,
                    reverse_event
                        .previous_soi_body
                        .expect("Reverse event for EnteringSOI must have a previous body id"),
                );
            }
            EventKind::ExitingSOI => {
                self.change_soi(
                    ship_id,
                    reverse_event
                        .previous_soi_body
                        .expect("Reverse event for ExitingSOI must have a previous body id"),
                );
            }
        }

        // Put the event back on the ship
        self.ships.get_mut(&ship_id).unwrap().next_event = Some(reverse_event.event);
    }

    fn change_soi(&mut self, ship_id: ShipID, new_body: BodyID) {
        let new_frame = Frame::BodyInertial(new_body);
        let parent_mu = self.bodies[&new_body].info.mu;

        // Get the new state of the ship
        let state = self.get_ship(ship_id).state();
        let new_position = state.get_position(new_frame);
        let new_velocity = state.get_velocity(new_frame);

        // Re-root the ship to the new body
        let ship = self.ships.get_mut(&ship_id).unwrap();
        ship.state = CartesianState::new(new_position.coords, new_velocity, parent_mu);
        ship.parent_id = new_body;
    }
}
