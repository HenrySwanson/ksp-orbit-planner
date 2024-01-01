use std::collections::HashMap;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::astro::orbit::{Orbit, PointMass, TimedOrbit};
use crate::astro::state::CartesianState;
use crate::math::frame::FrameTransform;

mod body;
mod ship;

pub use body::{Body, BodyID, BodyInfo};
pub use ship::{Ship, ShipID};

use super::events::{Event, EventData};

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

#[derive(Debug, Clone)]
struct BodyState {
    body: Body,
    orbit: Option<TimedOrbit<Body, ()>>,
}

// TODO: should the BodyInfo live in some other struct that
// does not clone, and lives forever?
#[derive(Debug, Clone)]
pub struct Orrery {
    bodies: HashMap<BodyID, BodyState>,
    next_body_id: usize,
    ships: HashMap<ShipID, Ship>,
    next_ship_id: usize,
}

impl FramedState<'_> {
    pub fn get_position(&self, frame: Frame, time: f64) -> Point3<f64> {
        self.orrery
            .convert_frames(self.native_frame, frame, time)
            .convert_point(&self.position)
    }

    pub fn get_velocity(&self, frame: Frame, time: f64) -> Vector3<f64> {
        self.orrery
            .convert_frames(self.native_frame, frame, time)
            .convert_velocity(&self.position, &self.velocity)
    }
}

impl BodyState {
    fn two_body_orbit(&self) -> Option<TimedOrbit<&Body, &Body>> {
        self.orbit.as_ref().map(|orbit| {
            // Plug self into orbit
            orbit.as_ref().with_secondary(&self.body)
        })
    }
}

impl Orrery {
    pub fn new() -> Self {
        Orrery {
            bodies: HashMap::new(),
            next_body_id: 0,
            ships: HashMap::new(),
            next_ship_id: 0,
        }
    }

    pub fn get_parent(&self, id: BodyID) -> Option<BodyID> {
        self.bodies[&id]
            .orbit
            .as_ref()
            .map(|orbit| orbit.primary().id)
    }

    pub fn orbit_of_body(&self, id: BodyID) -> Option<TimedOrbit<&Body, &Body>> {
        self.bodies[&id].two_body_orbit()
    }

    pub fn orbit_of_ship(&self, id: ShipID) -> TimedOrbit<&Body, ShipID> {
        let ship = &self.ships[&id];
        ship.orbit.as_ref().with_secondary(id)
    }

    pub fn bodies(&self) -> impl Iterator<Item = &Body> + '_ {
        self.bodies.values().map(|x| &x.body)
    }

    pub fn body_orbits(&self) -> impl Iterator<Item = TimedOrbit<&Body, &Body>> + '_ {
        self.bodies.values().filter_map(BodyState::two_body_orbit)
    }

    pub fn get_body(&self, id: BodyID) -> &Body {
        &self.bodies[&id].body
    }

    pub fn add_body(
        &mut self,
        body_info: BodyInfo,
        orbit: Orbit<PointMass, ()>,
        time_at_periapsis: f64,
        parent_id: BodyID,
    ) -> BodyID {
        let orbit = TimedOrbit::from_orbit(
            orbit.map_primary(|p| {
                let body = self.bodies[&parent_id].body.clone();
                debug_assert_eq!(body.info.mu, p.mu());
                body
            }),
            time_at_periapsis,
        );
        self.insert_new_body(body_info, Some(orbit))
    }

    pub fn add_fixed_body(&mut self, body_info: BodyInfo) -> BodyID {
        self.insert_new_body(body_info, None)
    }

    fn insert_new_body(&mut self, info: BodyInfo, orbit: Option<TimedOrbit<Body, ()>>) -> BodyID {
        let id = BodyID(self.next_body_id);
        self.next_body_id += 1;

        let body = BodyState {
            body: Body { id, info },
            orbit,
        };

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
        current_time: f64,
        parent_id: BodyID,
    ) -> ShipID {
        let new_id = ShipID(self.next_ship_id);
        self.next_ship_id += 1;

        let primary = self.bodies[&parent_id].body.clone();

        let ship = Ship {
            id: new_id,
            orbit: TimedOrbit::from_state(
                CartesianState::new(primary, position, velocity),
                current_time,
            ),
        };

        self.ships.insert(new_id, ship);
        new_id
    }

    pub fn convert_frames(&self, src: Frame, dst: Frame, time: f64) -> FrameTransform<f64> {
        // TODO : do this in a more clever way
        let src_to_root = self.convert_from_root(src, time).inverse();
        let root_to_dst = self.convert_from_root(dst, time);
        src_to_root.append_transformation(&root_to_dst)
    }

    fn convert_from_root(&self, frame: Frame, time: f64) -> FrameTransform<f64> {
        match frame {
            Frame::Root => FrameTransform::identity(),
            Frame::BodyInertial(k) => {
                match &self.bodies[&k].orbit {
                    None => {
                        // This is equivalent to the root frame; return the identity
                        FrameTransform::identity()
                    }
                    Some(orbit) => {
                        // Get the parent and compute the transform from its reference frame to root
                        let parent_frame = Frame::BodyInertial(orbit.primary().id);
                        let root_to_parent = self.convert_from_root(parent_frame, time);

                        // Get the transform from our frame to the parent's
                        let parent_to_self = FrameTransform::from_active(
                            UnitQuaternion::identity(),
                            orbit.state_at_time(time).position(),
                            orbit.state_at_time(time).velocity(),
                            Vector3::zeros(),
                        );
                        root_to_parent.append_transformation(&parent_to_self)
                    }
                }
            }
            Frame::ShipInertial(k) => {
                let ship = &self.ships[&k];
                let parent_frame = Frame::BodyInertial(ship.parent_id());
                let root_to_parent = self.convert_from_root(parent_frame, time);

                let parent_to_self = FrameTransform::from_active(
                    UnitQuaternion::identity(),
                    ship.orbit.state_at_time(time).position(),
                    ship.orbit.state_at_time(time).velocity(),
                    Vector3::zeros(),
                );
                root_to_parent.append_transformation(&parent_to_self)
            }
            Frame::ShipOrbital(k) => {
                let ship = &self.ships[&k];
                let root_to_parent = self.convert_from_root(Frame::ShipInertial(k), time);
                let orientation = crate::math::geometry::always_find_rotation(
                    &ship.orbit.normal_vector(),
                    &ship.orbit.state_at_time(time).velocity(),
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

    pub fn get_body_state(&self, id: BodyID, time: f64) -> FramedState<'_> {
        let (p, v, frame) = match &self.bodies[&id].orbit {
            None => (Vector3::zeros(), Vector3::zeros(), Frame::Root),
            Some(orbit) => (
                orbit.state_at_time(time).position(),
                orbit.state_at_time(time).velocity(),
                Frame::BodyInertial(orbit.primary().id),
            ),
        };

        FramedState {
            orrery: self,
            position: Point3::from(p),
            velocity: v,
            native_frame: frame,
        }
    }

    pub fn get_ship_state(&self, id: ShipID, time: f64) -> FramedState<'_> {
        let ship = &self.ships[&id];

        FramedState {
            orrery: self,
            position: Point3::from(ship.orbit.state_at_time(time).position()),
            velocity: ship.orbit.state_at_time(time).velocity(),
            native_frame: Frame::BodyInertial(ship.parent_id()),
        }
    }

    pub fn get_soi_radius(&self, id: BodyID) -> Option<f64> {
        let orbit = self.bodies[&id].two_body_orbit()?;
        Some(orbit.soi_radius())
    }

    pub fn change_soi(&mut self, ship_id: ShipID, new_parent_id: BodyID, event_time: f64) {
        let new_frame = Frame::BodyInertial(new_parent_id);
        let new_parent_body = &self.bodies[&new_parent_id];

        // Get the new state of the ship
        let ship = &self.ships[&ship_id];
        let old_parent_id = ship.parent_id();
        let state = FramedState {
            orrery: self,
            position: Point3::from(ship.orbit.state_at_time(event_time).position()),
            velocity: ship.orbit.state_at_time(event_time).velocity(),
            native_frame: Frame::BodyInertial(old_parent_id),
        };

        let new_position = state.get_position(new_frame, event_time);
        let new_velocity = state.get_velocity(new_frame, event_time);

        // Re-root the ship to the new body
        let ship = self.ships.get_mut(&ship_id).unwrap();
        ship.orbit = TimedOrbit::from_state(
            CartesianState::new(
                new_parent_body.body.clone(),
                new_position.coords,
                new_velocity,
            ),
            event_time,
        );
        println!(
            "Rerooted ship {} from {} to {}",
            ship_id.0,
            self.bodies[&old_parent_id].body.info.name,
            self.bodies[&new_parent_id].body.info.name
        );
    }

    pub fn process_event(&mut self, event: &Event) {
        // Dispatch to the appropriate handler
        let ship_id = event.ship_id;
        match &event.data {
            EventData::EnteringSOI(soi_change) | EventData::ExitingSOI(soi_change) => {
                self.change_soi(ship_id, soi_change.new, event.point.time);
            }
        }
    }

    pub fn revert_event(&mut self, event: &Event) {
        // Dispatch to the appropriate handler
        let ship_id = event.ship_id;
        match &event.data {
            EventData::EnteringSOI(soi_change) | EventData::ExitingSOI(soi_change) => {
                self.change_soi(ship_id, soi_change.old, event.point.time);
            }
        }
    }
}
