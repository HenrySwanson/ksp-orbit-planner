use kiss3d::nalgebra as na;
use na::Point3;

use super::orbit::Orbit;
use super::state::CartesianState;
use super::universe::{BodyID, Frame};

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

impl Body {
    pub fn get_position_with_frame(&self) -> (Point3<f64>, Frame) {
        match &self.state {
            BodyState::FixedAtOrigin => (Point3::origin(), Frame::Root),
            BodyState::Orbiting { parent_id, state } => (
                Point3::from(*state.get_position()),
                Frame::BodyInertial(*parent_id),
            ),
        }
    }

    pub fn get_orbit(&self) -> Option<Orbit> {
        match &self.state {
            BodyState::FixedAtOrigin => None,
            BodyState::Orbiting { state, .. } => Some(state.get_orbit()),
        }
    }
}
