use kiss3d::nalgebra as na;
use na::{Isometry3, Point3, Translation3, Vector3};

use std::collections::HashMap;

use super::body::{Body, BodyInfo, BodyState};
use super::maneuver::Maneuver;
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
    body: &'u Body,
}

pub struct ShipRef<'u> {
    universe: &'u Universe,
    ship: &'u Ship,
}

pub struct Universe {
    bodies: HashMap<BodyID, Body>,
    next_body_id: usize,
    ships: HashMap<ShipID, Ship>,
    next_ship_id: usize,
}

impl FramedState<'_> {
    pub fn get_position(&self, frame: Frame) -> Point3<f64> {
        self.universe
            .convert_frames(self.native_frame, frame)
            .transform_point(&self.position)
    }

    pub fn get_velocity(&self, frame: Frame) -> Vector3<f64> {
        self.universe
            .convert_frames(self.native_frame, frame)
            .transform_vector(&self.velocity)
    }
}

impl<'u> BodyRef<'u> {
    pub fn info(&self) -> &BodyInfo {
        &self.body.info
    }

    // note -- the FramedState can outlive the BodyRef!
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

    pub fn get_orbit(&self) -> Option<Orbit> {
        match &self.body.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { state, .. } => Some(state.get_orbit()),
        }
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

    pub fn get_orbit(&self) -> Orbit {
        self.ship.state.get_orbit()
    }

    pub fn get_parent_id(&self) -> BodyID {
        self.ship.parent_id
    }
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

    pub fn get_body(&self, id: BodyID) -> BodyRef {
        BodyRef {
            universe: self,
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

    pub fn get_ship(&self, id: ShipID) -> ShipRef {
        ShipRef {
            universe: self,
            ship: &self.ships[&id],
        }
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
                        let vector_from_parent = state.get_position();
                        parent_transform * Translation3::from(vector_from_parent)
                    }
                }
            }
            Frame::ShipInertial(k) => {
                let ship = &self.ships[&k];
                let parent_transform =
                    self.convert_frame_to_root(Frame::BodyInertial(ship.parent_id));
                let vector_from_parent = ship.state.get_position();
                parent_transform * Translation3::from(vector_from_parent)
            }
            Frame::ShipOrbital(k) => {
                let ship = &self.ships[&k];
                let orbit = ship.state.get_orbit();
                let parent_transform = self.convert_frame_to_root(Frame::ShipInertial(k));
                // TODO oops! I've been using quaternion-based (Isometry3) and matrix-based (Rotation3)
                // things in the same code. let's pick one and unify
                let orientation = crate::math::geometry::always_find_rotation(
                    &orbit.normal_vector(),
                    &ship.state.get_velocity(),
                    1e-20,
                );
                parent_transform * na::UnitQuaternion::from_rotation_matrix(&orientation)
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
