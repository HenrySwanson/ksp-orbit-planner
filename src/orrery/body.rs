use nalgebra::Point3;

use crate::astro::orbit::{HasMass, TimedOrbit};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct BodyID(pub usize);

// All the immutable info about a body
// TODO: merge with Body?
#[derive(Debug, Clone)]
pub struct BodyInfo {
    pub name: String,
    pub mu: f64,
    pub radius: f32,
    pub color: Point3<f32>,
}

#[derive(Debug, Clone)]
pub struct Body {
    pub id: BodyID,
    pub info: BodyInfo,
}

// TODO: don't leak outside of this module
#[derive(Debug, Clone)]
pub(super) struct BodyWrapper {
    pub id: BodyID,
    pub info: BodyInfo,
    orbit: Option<TimedOrbit<Body, ()>>,
}

impl HasMass for Body {
    fn mu(&self) -> f64 {
        self.info.mu
    }
}

impl BodyWrapper {
    pub fn new(id: BodyID, info: BodyInfo, orbit: Option<TimedOrbit<Body, ()>>) -> Self {
        Self { id, info, orbit }
    }

    pub fn parent_id(&self) -> Option<BodyID> {
        self.orbit.as_ref().map(|orbit| orbit.orbit().primary().id)
    }

    pub fn orbit(&self) -> Option<TimedOrbit<Body, Body>> {
        self.orbit.as_ref().map(|orbit| {
            let secondary = Body {
                id: self.id,
                info: self.info.clone(),
            };
            orbit.clone().with_secondary(secondary)
        })
    }

    pub fn to_primary_body(&self) -> Body {
        Body {
            id: self.id,
            info: self.info.clone(),
        }
    }
}
