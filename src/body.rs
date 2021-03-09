use kiss3d::nalgebra as na;
use na::{Point3, Vector3};

use std::rc::Rc;

use crate::state::State;

// All the immutable info about a body
pub struct BodyInfo {
    pub mu: f64,
    pub radius: f32,
    pub color: Vector3<f32>,
}

pub struct Body {
    pub info: Rc<BodyInfo>,
    pub state: Option<State>,
}

impl Body {
    pub fn get_position_as_pt(&self) -> Point3<f64> {
        match &self.state {
            Some(state) => Point3::origin() + state.get_position(),
            None => Point3::origin(),
        }
    }
}
