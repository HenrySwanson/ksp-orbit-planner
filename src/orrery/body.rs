use nalgebra::Point3;

use super::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

// All the immutable info about a body
#[derive(Debug, Clone)]
pub struct BodyInfo {
    pub name: String,
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

#[derive(Debug, Clone)]
pub enum BodyState {
    FixedAtOrigin,
    Orbiting {
        parent_id: BodyID,
        state: CartesianState,
    },
}

#[derive(Debug, Clone)]
pub struct Body {
    pub id: BodyID,
    pub info: BodyInfo,
    pub state: BodyState,
}

impl Body {
    pub fn parent_id(&self) -> Option<BodyID> {
        match self.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { parent_id, .. } => Some(parent_id),
        }
    }

    pub fn state(&self) -> Option<&CartesianState> {
        match &self.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { state, .. } => Some(state),
        }
    }
}
