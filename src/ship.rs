use crate::maneuver::Maneuver;
use crate::state::CartesianState;
use crate::universe::BodyID;

// TODO un-pub fields
pub struct Ship {
    pub state: CartesianState,
    pub parent_id: BodyID,
    pub schedule: Vec<Maneuver>, // TODO does this belong in here?
}
