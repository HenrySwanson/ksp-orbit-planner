use kiss3d::nalgebra as na;
use na::{Isometry3, Point3, Translation3, Vector3};

use std::collections::HashMap;

use crate::body::{Body, BodyInfo, BodyState};
use crate::maneuver::Maneuver;
use crate::ship::Ship;
use crate::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Root,
    BodyInertial(BodyID),
}

pub struct Universe {
    pub bodies: HashMap<BodyID, Body>,
    next_body_id: usize,
    pub ships: HashMap<ShipID, Ship>,
    next_ship_id: usize,
}

impl Universe {
    pub fn new() -> Self {
        Universe {
            bodies: HashMap::new(),
            next_body_id: 0,
            ships: HashMap::new(),
            next_ship_id: 0,
        }
    }

    pub fn body_ids(&self) -> impl Iterator<Item = &BodyID> {
        self.bodies.keys()
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

    pub fn add_ship(
        &mut self,
        schedule: Vec<Maneuver>,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        parent_id: BodyID,
    ) -> ShipID {
        let parent_mu = self.bodies[&parent_id].info.mu;
        let ship = Ship {
            state: CartesianState::new(position, velocity, parent_mu),
            parent_id,
            schedule,
        };

        let new_id = ShipID(self.next_ship_id);
        self.next_ship_id += 1;

        self.ships.insert(new_id, ship);
        new_id
    }

    fn insert_new_body(&mut self, info: BodyInfo, state: BodyState) -> BodyID {
        let body = Body { info, state };

        let new_id = BodyID(self.next_body_id);
        self.next_body_id += 1;

        self.bodies.insert(new_id, body);
        new_id
    }

    pub fn get_body_position(&self, id: BodyID, desired_frame: Frame) -> Point3<f64> {
        let (position, original_frame) = self.bodies[&id].get_position_with_frame();
        self.convert_frames(original_frame, desired_frame)
            .transform_point(&position)
    }

    pub fn get_ship_position(&self, id: ShipID, desired_frame: Frame) -> Point3<f64> {
        let ship = &self.ships[&id];
        let position = Point3::from(*ship.state.get_position());
        self.convert_frames(Frame::BodyInertial(ship.parent_id), desired_frame)
            .transform_point(&position)
    }

    pub fn convert_frames(&self, src: Frame, dst: Frame) -> Isometry3<f64> {
        // TODO : do this in a more clever way
        let transform_to_root = self.convert_frame_to_root(src);
        let transform_from_root = self.convert_frame_to_root(dst).inverse();
        transform_from_root * transform_to_root
    }

    fn convert_frame_to_root(&self, src: Frame) -> Isometry3<f64> {
        match src {
            Frame::Root => Isometry3::identity(),
            Frame::BodyInertial(k) => {
                match &self.bodies[&k].state {
                    BodyState::FixedAtOrigin => {
                        // This is equivalent to the root frame; return the identity
                        Isometry3::identity()
                    }
                    BodyState::Orbiting { parent_id, state } => {
                        // Get the parent and compute the transform from its reference frame to root
                        let parent_frame = Frame::BodyInertial(*parent_id);
                        let parent_transform = self.convert_frame_to_root(parent_frame);
                        let vector_from_parent = state.get_position().clone();
                        parent_transform * Translation3::from(vector_from_parent)
                    }
                }
            }
        }
    }

    pub fn advance_t(&mut self, delta_t: f64) {
        for body in self.bodies.values_mut() {
            match &mut body.state {
                BodyState::FixedAtOrigin => {}
                BodyState::Orbiting { state, .. } => state.advance_t(delta_t),
            }
        }

        for ship in self.ships.values_mut() {
            ship.state.advance_t(delta_t);
        }
    }
}
