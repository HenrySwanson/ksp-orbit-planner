use super::body::BodyID;
use super::state::CartesianState;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShipID(pub usize);

// TODO un-pub fields
pub struct Ship {
    pub state: CartesianState,
    pub parent_id: BodyID,
}
