use nalgebra::{Point3, UnitQuaternion, Vector3};

use std::collections::HashMap;

use super::body::{Body, BodyID, BodyInfo, BodyState};
use super::event::{Event, EventKind, ReverseEvent};
use super::frame::FrameTransform;
use super::ship::{Ship, ShipID};
use super::state::CartesianState;

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Root,
    BodyInertial(BodyID),
    ShipInertial(ShipID),
    ShipOrbital(ShipID),
}

pub struct FramedState<'orr> {
    orrery: &'orr Orrery,
    position: Point3<f64>,
    velocity: Vector3<f64>,
    native_frame: Frame,
}

pub struct Orrery {
    bodies: HashMap<BodyID, Body>,
    next_body_id: usize,
    ships: HashMap<ShipID, Ship>,
    next_ship_id: usize,
    time: f64,
}

impl<'orr> FramedState<'orr> {
    pub fn get_position(&self, frame: Frame) -> Point3<f64> {
        self.orrery
            .convert_frames(self.native_frame, frame)
            .convert_point(&self.position)
    }

    pub fn get_velocity(&self, frame: Frame) -> Vector3<f64> {
        self.orrery
            .convert_frames(self.native_frame, frame)
            .convert_velocity(&self.position, &self.velocity)
    }
}

impl<'orr> Orrery {
    pub fn new(start_time: f64) -> Self {
        Orrery {
            bodies: HashMap::new(),
            next_body_id: 0,
            ships: HashMap::new(),
            next_ship_id: 0,
            time: start_time,
        }
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        self.bodies.values()
    }

    pub fn get_body(&self, id: BodyID) -> &Body {
        &self.bodies[&id]
    }

    pub fn child_bodies(&self, id: BodyID) -> impl Iterator<Item = &Body> {
        self.bodies()
            .filter(move |body| body.parent_id() == Some(id))
    }

    pub fn sibling_bodies(&self, id: BodyID) -> impl Iterator<Item = &Body> {
        let parent_id = self.bodies[&id].parent_id();
        self.bodies()
            .filter(move |body| body.parent_id() == parent_id && body.id != id)
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
        let id = BodyID(self.next_body_id);
        self.next_body_id += 1;

        let body = Body { id, info, state };

        self.bodies.insert(id, body);
        id
    }

    pub fn ships(&self) -> impl Iterator<Item = &Ship> {
        self.ships.values()
    }

    pub fn get_ship(&self, id: ShipID) -> &Ship {
        &self.ships[&id]
    }

    pub fn add_ship(
        &mut self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        parent_id: BodyID,
    ) -> ShipID {
        let new_id = ShipID(self.next_ship_id);
        self.next_ship_id += 1;
        let parent_mu = self.bodies[&parent_id].info.mu;
        let ship = Ship {
            id: new_id,
            state: CartesianState::new(position, velocity, parent_mu),
            parent_id,
        };

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

    pub fn get_body_state(&'orr self, id: BodyID) -> FramedState<'orr> {
        let (p, v, frame) = match &self.bodies[&id].state {
            BodyState::FixedAtOrigin => (Vector3::zeros(), Vector3::zeros(), Frame::Root),
            BodyState::Orbiting { state, parent_id } => (
                state.get_position(),
                state.get_velocity(),
                Frame::BodyInertial(*parent_id),
            ),
        };

        FramedState {
            orrery: self,
            position: Point3::from(p),
            velocity: v,
            native_frame: frame,
        }
    }

    pub fn get_ship_state(&'orr self, id: ShipID) -> FramedState<'orr> {
        let ship = &self.ships[&id];

        FramedState {
            orrery: self,
            position: Point3::from(ship.state.get_position()),
            velocity: ship.state.get_velocity(),
            native_frame: Frame::BodyInertial(ship.parent_id),
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
        // delta_t can be positive, negative, or zero

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
    pub fn change_soi(&mut self, ship_id: ShipID, new_body: BodyID) {
        let new_frame = Frame::BodyInertial(new_body);
        let parent_mu = self.bodies[&new_body].info.mu;

        // Get the new state of the ship
        let ship = &self.ships[&ship_id];
        let state = FramedState {
            orrery: &self,
            position: Point3::from(ship.state.get_position()),
            velocity: ship.state.get_velocity(),
            native_frame: Frame::BodyInertial(ship.parent_id),
        };

        let new_position = state.get_position(new_frame);
        let new_velocity = state.get_velocity(new_frame);

        // Re-root the ship to the new body
        let ship = self.ships.get_mut(&ship_id).unwrap();
        ship.state = CartesianState::new(new_position.coords, new_velocity, parent_mu);
        ship.parent_id = new_body;
    }

    // TODO add search distance
    pub fn compute_next_event(&self, ship_id: ShipID) -> Option<Event> {
        let ship = &self.ships[&ship_id];
        let parent_body_id = ship.parent_id;
        let mut events = vec![];

        // Look for an escape event
        events.push(match self.get_soi_radius(parent_body_id) {
            Some(soi_radius) => {
                ship.state
                    .find_soi_escape_event(soi_radius, self.time)
                    .map(|point| Event {
                        ship_id,
                        kind: EventKind::ExitingSOI,
                        point,
                    })
            }
            None => None,
        });

        // Look for encounter events with co-orbiting bodies
        for body in self.bodies.values() {
            let state = match &body.state {
                BodyState::Orbiting { parent_id, state } if *parent_id == parent_body_id => state,
                _ => continue,
            };
            let soi_radius = self.get_soi_radius(body.id).unwrap();

            let event_pt = ship
                .state
                .find_soi_encounter_event(state, soi_radius, self.time);
            events.push(event_pt.map(|point| Event {
                ship_id,
                kind: EventKind::EnteringSOI(body.id),
                point,
            }));
        }

        events
            .into_iter()
            .flatten()
            .min_by(|a, b| a.point.compare_time(&b.point))
    }

    pub fn process_event(&mut self, event: Event) -> ReverseEvent {
        // Dispatch to the appropriate handler
        let ship_id = event.ship_id;
        match event.kind {
            EventKind::EnteringSOI(body_id) => {
                let prev_body_id = self.ships[&ship_id].parent_id;
                self.change_soi(ship_id, body_id);
                ReverseEvent {
                    event,
                    previous_soi_body: Some(prev_body_id),
                }
            }
            EventKind::ExitingSOI => {
                let current_body = self.ships[&ship_id].parent_id;
                let parent_id = self.bodies[&current_body]
                    .parent_id()
                    .expect("Cannot leave SOI of root body");
                self.change_soi(ship_id, parent_id);
                ReverseEvent {
                    event,
                    previous_soi_body: Some(current_body),
                }
            }
        }
    }

    pub fn revert_event(&mut self, reverse_event: &ReverseEvent) {
        // Dispatch to the appropriate handler
        let ship_id = reverse_event.event.ship_id;
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
    }
}
