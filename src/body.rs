use kiss3d::nalgebra as na;
use na::Point3;

use std::rc::Rc;

use crate::state::State;

// All the immutable info about a body
pub struct BodyInfo {
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

pub struct Body {
    pub info: Rc<BodyInfo>,
    pub state: State,
}
