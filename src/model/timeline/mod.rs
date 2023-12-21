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

    /// Search until the given time for any new events, potentially creating a
    /// new segment if an event is found.
    pub fn extend_until(&mut self, time: f64) {
        let closed_segment = self.open_segment.split_at_next_event(time);

        // If we find an event, we should add a new segment! Otherwise do nothing,
        // the UpcomingEvents struct will have already saved our progress
        // TODO: if we find an event, we must search again!!!!!!
        // or, wait, does the segmenting save us? do we just render wrong for a hot
        // second? idk.... it might...
        if let Some(closed_segment) = closed_segment {
            let event = &closed_segment.ending_event;
            println!(
                "When extending end time to {}, found event at time {} for ship {}: {:?}",
                time, event.point.time, event.ship_id.0, event.data
            );

            self.closed_segments.push(closed_segment);
        }
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

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    use super::*;
    use crate::file::read_file;
    use crate::model::events::{EventData, SOIChange};
    use crate::model::orrery::BodyID;

    const KERBIN: BodyID = BodyID(4);
    const MUN: BodyID = BodyID(5);
    const KERBOL: BodyID = BodyID(0);

    const ENCOUNTER_MUN: EventData = {
        EventData::EnteringSOI(SOIChange {
            old: KERBIN,
            new: MUN,
        })
    };
    const ESCAPE_MUN: EventData = {
        EventData::ExitingSOI(SOIChange {
            old: MUN,
            new: KERBIN,
        })
    };
    const ESCAPE_KERBIN: EventData = {
        EventData::ExitingSOI(SOIChange {
            old: KERBIN,
            new: KERBOL,
        })
    };

    /// This particular scenario is one I've been using for a really long time.
    /// It goes like this:
    /// - 0d: Ship proceedes in moderately elliptical trajectory (e = 0.7, SMA =
    ///   20M)
    /// - 13d: Encounters Mun and switches to smaller orbit (e = 0.73, SMA =
    ///   8.7M)
    /// - 14d: Immediate re-encounter, enlarges orbit again (e = 0.7, SMA = 21M)
    /// - 22d: Tightens orbit to small and narrow (e = 0.83, SMA = 7.3M)
    /// - 31d: Re-enlarges orbit (e = 0.69, SMA = 17M)
    /// - 45d: Just grazes Mun, slight modification of orbit (e = 0.66, SMA =
    ///   14M)
    /// - 49d: Bounces off the Mun (e = 0.74, SMA = 9.5M)
    /// - 55d: Clips through the Mun and drops almost into Kerbin (e = 0.92, SMA
    ///   = 6.7M)
    /// - 58d: Bounces off the Mun, and enters a hyperbolic orbit (e = 1.95, SMA
    ///   = -12M)
    /// - 60d: Escapes Kerbin's orbit, and starts orbiting the Sun (e = 0.097,
    ///   SMA = 15B)
    #[test]
    fn test_favorite_scenario() {
        let expected_events = vec![
            // 13d
            (1167224.3810535548, ENCOUNTER_MUN),
            (1176541.0255763677, ESCAPE_MUN),
            // 14d
            (1288753.3454258977, ENCOUNTER_MUN),
            (1298160.1769034935, ESCAPE_MUN),
            // 22d
            (1903256.1219919417, ENCOUNTER_MUN),
            (1913118.229924371, ESCAPE_MUN),
            // 31d
            (2727926.8998953775, ENCOUNTER_MUN),
            (2737612.195569021, ESCAPE_MUN),
            // 45d
            (3891256.0700824754, ENCOUNTER_MUN),
            (3896079.039262741, ESCAPE_MUN),
            // 49d
            (4263267.2755853385, ENCOUNTER_MUN),
            (4272708.9974184595, ESCAPE_MUN),
            // 55d
            (4792775.67788562, ENCOUNTER_MUN),
            (4803927.607622115, ESCAPE_MUN),
            // 59d
            (5066549.0475938115, ENCOUNTER_MUN),
            (5075614.547186624, ESCAPE_MUN),
            // 60d
            (5199986.651638684, ESCAPE_KERBIN),
        ];

        let mut orrery = read_file("ksp-bodies.txt");
        orrery.add_ship(Vector3::x() * 6000000.0, Vector3::y() * 1000.0, 0.0, KERBIN);

        let mut timeline = Timeline::new(orrery, 0.0);

        // TODO: fix the extend time to work with just one extension,
        // this is atrocious :)
        let mut end_time = 0.0;
        while end_time < (65 * 86400) as f64 {
            timeline.extend_until(end_time);
            end_time += 86400.0;
        }

        let events: Vec<_> = timeline
            .closed_segments
            .iter()
            .map(|seg| seg.ending_event.clone())
            .collect();

        for (event, expected_event) in std::iter::zip(events, expected_events) {
            assert_eq!(event.data, expected_event.1);
            assert_relative_eq!(event.point.time, expected_event.0);
        }
    }
}
