use kiss3d::nalgebra as na;
use na::{Point3, Vector3};
use std::rc::Rc;

use crate::body::{Body, BodyInfo};
use crate::state::State;

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

    pub fn get_body_position(&self, id: usize) -> Point3<f64> {
        let mut position = Point3::origin();
        let mut current_id = id;
        loop {
            let body = &self.bodies[current_id];
            match &body.state {
                Some(state) => {
                    position += state.get_position();
                    current_id = state.get_parent_id();
                }
                None => return position,
            };
        }
    }
}
