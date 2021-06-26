use kiss3d::nalgebra as na;
use na::Point3;

use super::state::CartesianState;
use super::universe::BodyID;

// All the immutable info about a body
pub struct BodyInfo {
    pub name: String,
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

pub enum BodyState {
    FixedAtOrigin,
    Orbiting {
        parent_id: BodyID,
        state: CartesianState,
    },
}

pub struct Body {
    pub info: BodyInfo,
    pub state: BodyState,
}
