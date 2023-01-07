use crate::events::Event;
use crate::events::{search_for_soi_encounter, search_for_soi_escape, EventTag, UpcomingEvents};
use crate::orrery::Orrery;

#[derive(Debug)]
pub struct Timeline {
    // Invariants:
    //   - The `start_time` of each closed segment are sorted in ascending order,
    //     and the open segment is later than all of them.
    closed_segments: Vec<ClosedSegment>,
    open_segment: OpenSegment,
}

#[derive(Debug)]
struct ClosedSegment {
    start_time: f64,
    orrery: Orrery,
    ending_event: Event,
}

#[derive(Debug)]
struct OpenSegment {
    start_time: f64,
    orrery: Orrery,
    upcoming_events: UpcomingEvents,
}

#[derive(Debug)]
enum SegmentLookup {
    Closed(usize),
    Open,
    BeforeStart,
}

impl Timeline {
    // TODO: take time as input
    pub fn new(orrery: Orrery) -> Self {
        let start_time = 0.0;
        Self {
            closed_segments: vec![],
            open_segment: OpenSegment::new(start_time, orrery),
        }
    }

    fn lookup_segment(&self, time: f64) -> SegmentLookup {
        // Check whether it's in the open segment
        if time >= self.open_segment.start_time {
            return SegmentLookup::Open;
        }

        // Otherwise, look for the first closed segment to strictly precede this time.
        // This is the segment right after the one we're looking for.
        let next_segment_idx = self
            .closed_segments
            .partition_point(|s| s.start_time <= time);

        // If that's the first segment, then this time is before the model starts.
        if next_segment_idx == 0 {
            return SegmentLookup::BeforeStart;
        }

        // Otherwise, return the segment before.
        let segment_idx = next_segment_idx - 1;
        let segment = &self.closed_segments[segment_idx];
        assert!(time >= segment.start_time);
        SegmentLookup::Closed(segment_idx)
    }

    pub fn get_orrery_at(&self, time: f64) -> Option<&Orrery> {
        match self.lookup_segment(time) {
            SegmentLookup::Closed(idx) => {
                let orrery = &self.closed_segments[idx].orrery;
                Some(orrery)
            }
            SegmentLookup::Open => {
                let orrery = &self.open_segment.orrery;
                Some(orrery)
            }
            SegmentLookup::BeforeStart => None,
        }
    }

    pub fn extend_end_time(&mut self, time: f64) {
        // Check whether this time lies in the open segment; if not, return early
        if time < self.open_segment.start_time {
            return;
        }

        let event = self.open_segment.extend_until(time);

        // If we find an event, we should add a new segment! Otherwise do nothing,
        // the UpcomingEvents struct will have already saved our progress
        // TODO: if we find an event, we must search again!!!!!!
        // or, wait, does the segmenting save us? do we just render wrong for a hot second?
        // idk.... it might...
        if let Some(event) = event {
            let event = event.clone();
            println!("Extend to {}: found event {:?}", time, event);

            let closed_segment = self.open_segment.close_and_advance(event);
            self.closed_segments.push(closed_segment);
        }
    }
}

impl OpenSegment {
    fn new(start_time: f64, orrery: Orrery) -> Self {
        Self {
            start_time,
            orrery,
            upcoming_events: UpcomingEvents::new(start_time),
        }
    }

    fn extend_until(&mut self, time: f64) -> Option<&Event> {
        search_for_events(
            &self.orrery,
            &mut self.upcoming_events,
            self.start_time,
            time,
        );
        self.upcoming_events.get_next_event_global()
    }

    fn close_and_advance(&mut self, event: Event) -> ClosedSegment {
        let event_time = event.point.time;

        // Make a new open segment
        let mut new_open = OpenSegment::new(event_time, self.orrery.clone());
        new_open.orrery.process_event(&event);

        let old_open = std::mem::replace(self, new_open);

        // Turn old_open into a closed segment
        ClosedSegment {
            start_time: old_open.start_time,
            orrery: old_open.orrery,
            ending_event: event,
        }
    }
}

// TODO: copied from universe.rs; clean it up for our purposes later
fn search_for_events(
    orrery: &Orrery,
    upcoming_events: &mut UpcomingEvents,
    start_time: f64,
    end_time: f64,
) {
    for id in orrery.ships().map(|s| s.id) {
        // TODO: i don't think i need this, and it may actually be wrong
        if upcoming_events.get_next_event(id).is_some() {
            continue;
        }

        assert!(end_time >= start_time);

        // TODO this "check if we should search, and then separately search" is not
        // a great pattern. They should be bundled together better.

        // Check for an SOI escape event
        upcoming_events.update(id, EventTag::EscapeSOI, end_time, |_, _| {
            search_for_soi_escape(orrery, id)
        });

        // Check for SOI encounter events
        for body in orrery.bodies() {
            upcoming_events.update(
                id,
                EventTag::EncounterSOI(body.id),
                end_time,
                |search_start, search_end: f64| {
                    search_for_soi_encounter(orrery, id, body.id, search_start, search_end)
                },
            );
        }
    }
}
