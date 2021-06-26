use super::maneuver::Maneuver;
use super::state::CartesianState;
use super::universe::BodyID;

// TODO un-pub fields
pub struct Ship {
    pub state: CartesianState,
    pub parent_id: BodyID,
    pub schedule: Vec<Maneuver>, // TODO does this belong in here?
}
