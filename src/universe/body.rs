use nalgebra::Point3;

use super::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

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
    pub id: BodyID,
    pub info: BodyInfo,
    pub state: BodyState,
}
