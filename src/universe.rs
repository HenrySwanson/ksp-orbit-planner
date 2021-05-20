use kiss3d::nalgebra as na;
use na::{Isometry3, Point3, Translation3, Vector3};
use std::rc::Rc;

use crate::body::{Body, BodyInfo};
use crate::orbit::Orbit;
use crate::state::State;

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
            state: None,
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
        let state = State::new(position, velocity, 0.0, parent_id, parent_body.info.clone());

        let body = Body {
            info: Rc::new(body_info),
            state: Some(state),
        };

        self.bodies.push(body);
        self.bodies.len() - 1
    }

    pub fn get_body_position_with_frame(&self, id: usize) -> (Point3<f64>, Frame) {
        match &self.bodies[id].state {
            None => (Point3::origin(), Frame::Root),
            Some(state) => {
                let position = state.get_position().clone();
                let parent_id = state.get_parent_id();
                (Point3::from(position), Frame::BodyInertial(parent_id))
            }
        }
    }

    pub fn get_body_position(&self, id: usize, desired_frame: Frame) -> Point3<f64> {
        let (position, original_frame) = self.get_body_position_with_frame(id);
        self.convert_frames(original_frame, desired_frame)
            .transform_point(&position)
    }

    pub fn get_body_orbit(&self, id: usize) -> Option<Orbit> {
        // TODO: how am i going to handle moons? reference frames?
        let state = match &self.bodies[id].state {
            Some(state) => state,
            None => return None,
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
        // First, convert it to the root frame
        let transform_to_root = self.convert_frame_to_root(src);
        let transform_from_root = self.convert_frame_to_root(dst).inverse();
        transform_from_root * transform_to_root
    }

    fn convert_frame_to_root(&self, src: Frame) -> Isometry3<f64> {
        match src {
            Frame::Root => Isometry3::identity(),
            Frame::BodyInertial(k) => {
                // Get the parent and compute the transform from its reference frame to root
                let body = &self.bodies[k];
                match &body.state {
                    None => {
                        // We are the root body, return the identity
                        Isometry3::identity()
                    }
                    Some(state) => {
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
