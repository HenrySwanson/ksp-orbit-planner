use kiss3d::nalgebra as na;
use na::{Isometry3, Point3, Translation3, Vector3};
use std::rc::Rc;

use crate::body::{Body, BodyInfo};
use crate::orbit::Orbit;
use crate::state::{CartesianState, State};

#[derive(Debug, Clone, Copy)]
pub enum Frame {
    Root,
    BodyInertial(usize),
}

pub struct Universe {
    pub root_body: usize,
    pub bodies: Vec<Body>,
}

impl Universe {
    pub fn new(root_body: BodyInfo) -> Self {
        let body = Body {
            info: Rc::new(root_body),
            state: State::FixedAtOrigin,
        };

        Universe {
            root_body: 0,
            bodies: vec![body],
        }
    }

    pub fn add_body(
        &mut self,
        body_info: BodyInfo,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        parent_id: usize,
    ) -> usize {
        let parent_body = self.bodies.get(parent_id).unwrap();
        let state =
            CartesianState::new(position, velocity, 0.0, parent_id, parent_body.info.clone());

        let body = Body {
            info: Rc::new(body_info),
            state: State::Orbiting(state),
        };

        self.bodies.push(body);
        self.bodies.len() - 1
    }

    pub fn get_body_position(&self, id: usize, desired_frame: Frame) -> Point3<f64> {
        let (position, original_frame) = self.bodies[id].state.get_position();
        self.convert_frames(original_frame, desired_frame)
            .transform_point(&position)
    }

    pub fn get_body_orbit(&self, id: usize) -> Option<Orbit> {
        // TODO: how am i going to handle moons? reference frames?
        let state = match &self.bodies[id].state {
            State::FixedAtOrigin => return None,
            State::Orbiting(state) => state,
        };

        let parent_body = &self.bodies[state.get_parent_id()];
        Some(Orbit::from_cartesian(
            state.get_position(),
            state.get_velocity(),
            parent_body.info.mu,
        ))
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
                match &self.bodies[k].state {
                    State::FixedAtOrigin => {
                        // This is equivalent to the root frame; return the identity
                        Isometry3::identity()
                    }
                    State::Orbiting(state) => {
                        // Get the parent and compute the transform from its reference frame to root
                        let parent_frame = Frame::BodyInertial(state.get_parent_id());
                        let parent_transform = self.convert_frame_to_root(parent_frame);
                        let vector_from_parent = state.get_position().clone();
                        parent_transform * Translation3::from(vector_from_parent)
                    }
                }
            }
        }
    }
}
