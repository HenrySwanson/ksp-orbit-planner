use crate::events::event_search::{
    search_for_soi_encounter, search_for_soi_escape, EventTag, UpcomingEvents,
};
use crate::events::Event;
use crate::orrery::Orrery;

pub struct Timeline {
    // Invariants:
    //   - The `start_time` of each segment are sorted in ascending order
    //   - Only the last segment has SegmentEnd::Open
    segments: Vec<Segment>,
}

#[derive(Debug)]
struct Segment {
    start_time: f64,
    start_state: Orrery,
    end: SegmentEnd,
}

#[derive(Debug)]
enum SegmentEnd {
    Closed(Event),
    Open(UpcomingEvents),
}

#[derive(Debug)]
enum SegmentLookup {
    Closed(usize),
    Open(usize),
    BeforeStart,
}

impl Timeline {
    pub fn new(orrery: Orrery) -> Self {
        let initial_segment = Segment {
            start_time: 0.0,
            start_state: orrery,
            end: SegmentEnd::Open(UpcomingEvents::new()),
        };
        Self {
            segments: vec![initial_segment],
        }
    }

    fn lookup_segment(&self, time: f64) -> SegmentLookup {
        // Get the first segment to strictly precede this time.
        let next_segment_idx = self.segments.partition_point(|s| s.start_time <= time);
        // If that's the first segment, then this time is before the model starts.
        if next_segment_idx == 0 {
            return SegmentLookup::BeforeStart;
        }

        let segment_idx = next_segment_idx - 1;
        let segment = &self.segments[segment_idx];
        assert!(time - segment.start_time >= 0.0);

        // Check whether this is the last segment
        if next_segment_idx == self.segments.len() {
            SegmentLookup::Open(segment_idx)
        } else {
            SegmentLookup::Closed(segment_idx)
        }
    }

    pub fn get_orrery_at(&self, time: f64) -> Option<Orrery> {
        match self.lookup_segment(time) {
            SegmentLookup::Closed(idx) | SegmentLookup::Open(idx) => {
                let mut orrery = self.segments[idx].start_state.clone();
                orrery.update_time(time - self.segments[idx].start_time);
                Some(orrery)
            }
            SegmentLookup::BeforeStart => None,
        }
    }

    pub fn extend_end_time(&mut self, time: f64) {
        let segment = match self.lookup_segment(time) {
            SegmentLookup::Open(s) => &mut self.segments[s],
            SegmentLookup::Closed(_) | SegmentLookup::BeforeStart => return,
        };

        // Wow this is distasteful. See if you can improve it!
        let upcoming = match &mut segment.end {
            SegmentEnd::Open(x) => x,
            other => panic!("Last segment had {:?} as its end", other),
        };

        // Use the event search to look for an event
        let delta_t = time - segment.start_time;
        search_for_events(&segment.start_state, upcoming, delta_t);
        let event = upcoming.get_next_event_global();

        // If we find an event, we should add a new segment! Otherwise do nothing,
        // the UpcomingEvents struct will have already saved our progress
        // TODO: if we find an event, we must search again!!!!!!
        // or, wait, does the segmenting save us? do we just render wrong for a hot second?
        // idk.... it might...
        if let Some(event) = event {
            println!("Extend to {}: found event {:?}", time, event);

            // Clear the events for the ship
            let event = event.clone();
            upcoming.clear_events(event.ship_id);

            // Get the new state
            let event_time = event.point.time;
            let mut state = segment.start_state.clone();
            state.update_time(event_time - segment.start_time);
            state.process_event(&event);

            // Close out the old segment
            segment.end = SegmentEnd::Closed(event);

            // Create the new segment and push it
            let new_segment = Segment {
                start_time: event_time,
                start_state: state,
                end: SegmentEnd::Open(UpcomingEvents::new()),
            };
            self.segments.push(new_segment);
        }
    }
}

// TODO: copied from universe.rs; clean it up for our purposes later
fn search_for_events(orrery: &Orrery, upcoming_events: &mut UpcomingEvents, delta_t: f64) {
    for id in orrery.ships().map(|s| s.id) {
        if upcoming_events.get_next_event(id).is_some() {
            continue;
        }

        let start_time = orrery.get_time();
        let end_time = start_time + delta_t;

        // TODO this "check if we should search, and then separately search" is not
        // a great pattern. They should be bundled together better.

        // Check for an SOI escape event
        upcoming_events.update(id, EventTag::EscapeSOI, end_time, |_| {
            search_for_soi_escape(orrery, id)
        });

        // Check for SOI encounter events
        for body in orrery.bodies() {
            upcoming_events.update(
                id,
                EventTag::EncounterSOI(body.id),
                end_time,
                |searched_until| {
                    search_for_soi_encounter(
                        orrery,
                        id,
                        body.id,
                        searched_until.unwrap_or(start_time),
                        end_time,
                    )
                },
            );
        }
    }
}
