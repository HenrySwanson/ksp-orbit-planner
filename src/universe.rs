use kiss3d::nalgebra as na;
use na::{Isometry3, Point3, Translation3, Vector3};

use std::collections::HashMap;

use crate::state::{CartesianState, State};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

// All the immutable info about a body
pub struct BodyInfo {
    pub name: String,
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

pub struct Body {
    pub info: BodyInfo,
    pub state: State,
}

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Root,
    BodyInertial(BodyID),
}

pub struct Universe {
    pub bodies: HashMap<BodyID, Body>,
    next_id: usize,
}

impl Universe {
    pub fn new() -> Self {
        Universe {
            bodies: HashMap::new(),
            next_id: 0,
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
        let state = CartesianState::new(position, velocity, parent_mu);
        self.insert_new_body(body_info, State::Orbiting(parent_id, state))
    }

    pub fn add_fixed_body(&mut self, body_info: BodyInfo) -> BodyID {
        self.insert_new_body(body_info, State::FixedAtOrigin)
    }

    fn insert_new_body(&mut self, info: BodyInfo, state: State) -> BodyID {
        let body = Body { info, state };

        let new_id = BodyID(self.next_id);
        self.next_id += 1;

        self.bodies.insert(new_id, body);
        new_id
    }

    pub fn get_body_position(&self, id: BodyID, desired_frame: Frame) -> Point3<f64> {
        let (position, original_frame) = self.bodies[&id].state.get_position();
        self.convert_frames(original_frame, desired_frame)
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
                    State::FixedAtOrigin => {
                        // This is equivalent to the root frame; return the identity
                        Isometry3::identity()
                    }
                    State::Orbiting(parent_id, state) => {
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
}
