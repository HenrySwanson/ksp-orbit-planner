use self::upcoming_events::EventSearchHorizons;
use super::events::{search_for_soi_encounter, search_for_soi_escape, Event, EventTag};
use super::orrery::Orrery;

mod upcoming_events;

/// Models the state of the universe as a sequence of [Orrery]s separated by
/// [Event]s.
///
/// The timeline consists of a sequence of [ClosedSegment]s followed by an
/// [OpenSegment]; segments are considered half-open, including the start time
/// but not the end.
#[derive(Debug)]
pub struct Timeline {
    // Invariants:
    //   - The `start_time` of each closed segment are sorted in ascending order, and the open
    //     segment is later than all of them.
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
    search_horizons: EventSearchHorizons,
}

#[derive(Debug)]
enum SegmentLookup {
    Closed(usize),
    Open,
    BeforeStart,
}

impl Timeline {
    /// Create a new Timeline with the given starting state.
    pub fn new(orrery: Orrery, start_time: f64) -> Self {
        Self {
            closed_segments: vec![],
            open_segment: OpenSegment::new(start_time, orrery),
        }
    }

    /// Search the timeline for the segment containing the given time.
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

    /// Get the orrery corresponding to the given time.
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

    /// Return the start time of this timeline.
    pub fn start_time(&self) -> f64 {
        if let Some(closed_segment) = self.closed_segments.first() {
            closed_segment.start_time
        } else {
            self.open_segment.start_time
        }
    }

    /// Search until the given time for any new events, potentially creating
    /// new segments if events are found.
    pub fn extend_until(&mut self, time: f64) {
        // Search for the next event. If we find one, add a new segment and repeat!
        // Otherwise, do nothing; the UpcomingEvents struct will save our progress.
        while let Some(closed_segment) = self.open_segment.split_at_next_event(time) {
            let event = &closed_segment.ending_event;
            println!(
                "When extending end time to {}, found event at time {} for ship {}: {:?}",
                time, event.point.time, event.ship_id.0, event.data
            );

            self.closed_segments.push(closed_segment);
        }
    }

    pub fn events(&self) -> impl Iterator<Item = &Event> {
        self.closed_segments.iter().map(|seg| &seg.ending_event)
    }
}

impl OpenSegment {
    fn new(start_time: f64, orrery: Orrery) -> Self {
        Self {
            start_time,
            orrery,
            search_horizons: EventSearchHorizons::new(start_time),
        }
    }

    fn split_at_next_event(&mut self, time: f64) -> Option<ClosedSegment> {
        self.search_for_events_until(time);
        let event = self.search_horizons.get_next_event()?.clone();
        let event_time = event.point.time;

        // Make a new open segment to replace this one
        let mut new_open = OpenSegment::new(event_time, self.orrery.clone());
        new_open.orrery.process_event(&event);

        // Swap in the new one, and decompose the old one into a closed segment
        let old_open = std::mem::replace(self, new_open);
        let closed_segment = ClosedSegment {
            start_time: old_open.start_time,
            orrery: old_open.orrery,
            ending_event: event,
        };

        Some(closed_segment)
    }

    fn search_for_events_until(&mut self, end_time: f64) {
        // Don't search unless the window is non-empty
        if self.start_time >= end_time {
            return;
        }

        for id in self.orrery.ships().map(|s| s.id) {
            // TODO: can i skip the search if i've advanced all horizons far enough?

            // Check for an SOI escape event
            self.search_horizons
                .search_until(id, EventTag::EscapeSOI, end_time, |_, _| {
                    search_for_soi_escape(&self.orrery, id)
                });

            // Check for SOI encounter events
            for body in self.orrery.bodies() {
                self.search_horizons.search_until(
                    id,
                    EventTag::EncounterSOI(body.id),
                    end_time,
                    |search_start, search_end: f64| {
                        search_for_soi_encounter(
                            &self.orrery,
                            id,
                            body.id,
                            search_start,
                            search_end,
                        )
                    },
                );
            }
        }
    }
}
