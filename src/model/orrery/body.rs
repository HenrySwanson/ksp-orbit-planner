use nalgebra::Point3;

use crate::astro::orbit::HasMass;

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

impl HasMass for Body {
    fn mu(&self) -> f64 {
        self.info.mu
    }
}
