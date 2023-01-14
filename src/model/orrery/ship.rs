use crate::astro::orbit::TimedOrbit;

use super::{Body, BodyID};

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

// TODO un-pub fields
#[derive(Debug, Clone)]
pub struct Ship {
    pub id: ShipID,
    pub orbit: TimedOrbit<Body, ()>,
}

impl Ship {
    pub fn parent_id(&self) -> BodyID {
        self.orbit.orbit().primary().id
    }
}
