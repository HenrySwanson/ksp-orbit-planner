use super::body::OrbitingData;
use super::BodyID;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

// TODO un-pub fields
#[derive(Debug, Clone)]
pub struct Ship {
    pub id: ShipID,
    pub orbit_data: OrbitingData,
}

impl Ship {
    pub fn parent_id(&self) -> BodyID {
        self.orbit_data.parent_id()
    }
}
