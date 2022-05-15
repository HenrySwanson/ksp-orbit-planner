use super::body::BodyID;
use super::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

// TODO un-pub fields
#[derive(Debug, Clone)]
pub struct Ship {
    pub id: ShipID,
    pub state: CartesianState,
    pub parent_id: BodyID,
}
