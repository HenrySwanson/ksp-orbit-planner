use nalgebra::Point3;

use crate::astro::orbit::{HasMass, TimedOrbit};

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
pub struct PrimaryBody {
    pub id: BodyID,
    pub info: BodyInfo,
}

#[derive(Debug, Clone)]
pub struct Body {
    pub id: BodyID,
    pub info: BodyInfo,
    orbit: Option<TimedOrbit<PrimaryBody, ()>>,
}

impl HasMass for PrimaryBody {
    fn mu(&self) -> f64 {
        self.info.mu
    }
}

impl Body {
    pub fn new(id: BodyID, info: BodyInfo, orbit: Option<TimedOrbit<PrimaryBody, ()>>) -> Self {
        Self { id, info, orbit }
    }

    pub fn parent_id(&self) -> Option<BodyID> {
        self.orbit().map(|orbit| orbit.orbit().primary().id)
    }

    pub fn orbit(&self) -> Option<&TimedOrbit<PrimaryBody, ()>> {
        self.orbit.as_ref()
    }
}
