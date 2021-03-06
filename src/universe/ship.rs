use super::event::Event;
use super::state::CartesianState;
use super::universe::BodyID;

// TODO un-pub fields
pub struct Ship {
    pub state: CartesianState,
    pub parent_id: BodyID,
    pub next_event: Option<Event>,
}
